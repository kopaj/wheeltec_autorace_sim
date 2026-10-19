"""Optional integration test against the user's ORIGINAL generator and SDFs.

Set WHEELTEC_EVALUATION_FIXTURES to a directory containing
generate_multicorner_track.py, racetrack.sdf, roboworks.sdf, racetrack_world.sdf.
Only copies in a TemporaryDirectory are modified.
"""
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import cv2

from wheeltec_autorace_evaluation.core import Reference
from wheeltec_autorace_evaluation.exporter import export, entity


@unittest.skipUnless(os.getenv('WHEELTEC_EVALUATION_FIXTURES'), 'Original generator/SDF fixture path not provided')
class ExportTests(unittest.TestCase):
    def test_exact_track_and_mismatch_rejection(self):
        source = Path(os.environ['WHEELTEC_EVALUATION_FIXTURES']).resolve()
        spec = importlib.util.spec_from_file_location('generator_for_test', source/'generate_multicorner_track.py')
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root/'models/racetrack/model.sdf'
            robot = root/'models/roboworks/model.sdf'
            image = model.parent/'materials/textures/course.png'
            world = root/'world.sdf'
            metadata = root/'metadata.json'
            for p in (model, robot, image):
                p.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source/'racetrack.sdf', model)
            shutil.copyfile(source/'roboworks.sdf', robot)
            shutil.copyfile(source/'racetrack_world.sdf', world)
            tree = ET.parse(world)
            entity(tree.getroot().find('world'), 'racetrack').find('pose').text = '2 -3 0 0 0 0.4'
            tree.write(world)
            with redirect_stdout(io.StringIO()):
                generator.generate(image, None, 13.8, 10, 42, .85, .8, 10000, .65,
                                   model, world, metadata)
            args = SimpleNamespace(metadata=str(metadata), generator=str(source/'generate_multicorner_track.py'),
                                   output=str(root/'reference.json'), texture=None, model=None,
                                   world=None, robot_model=None, max_attempts=10000, variation=.65)
            with redirect_stdout(io.StringIO()):
                export(args)
            ref = Reference.load(args.output)
            self.assertLess(abs(ref.project(*ref.data['world_spawn_pose'][:2])['cte_m']), .002)
            self.assertGreater(ref.length, 10)
            self.assertLess(ref.data['max_segment_length_m'], .021)
            # A stale/modified asset must fail both at export and at trial start.
            pixels = cv2.imread(str(image))
            pixels[0, 0] = [100, 100, 100]
            cv2.imwrite(str(image), pixels)
            with self.assertRaisesRegex(ValueError, 'Texture mismatch'):
                export(args)
            with self.assertRaisesRegex(ValueError, 'Asset changed'):
                Reference.load(args.output)
            # Reference edits cannot bypass its own identity check.
            data = json.loads(Path(args.output).read_text())
            data['lane_width_m'] = .9
            Path(args.output).write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                Reference.load(args.output, verify_assets=False)


if __name__ == '__main__':
    unittest.main()
