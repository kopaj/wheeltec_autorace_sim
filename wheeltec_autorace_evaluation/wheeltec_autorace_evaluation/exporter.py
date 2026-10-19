"""Reconstruct a known generated track, accepting only pixel-identical texture."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from .core import Reference, content_digest, digest_file


def write_reference(output, document, overwrite=False):
    """Publish a complete JSON atomically; preserve the previous file on failure."""
    output = Path(output).expanduser()
    payload = json.dumps(document, indent=2, allow_nan=False) + '\n'
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         dir=output.parent, prefix='.'+output.name+'.',
                                         suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if overwrite:
            os.replace(temporary, output)
        else:
            # Atomic create-if-absent; keep the manual export's existing behavior.
            os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def pose(element):
    p = element.find('pose')
    if p is None:
        return np.zeros(6)
    if p.attrib:
        raise ValueError('Only default SDF pose semantics are supported; no relative_to/degrees')
    values = np.array([float(v) for v in p.text.split()])
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('Invalid SDF pose')
    return values


def entity(world, name):
    found = [e for e in world if e.tag in ('model', 'include') and
             (e.get('name', '') if e.tag == 'model' else e.findtext('name', '')).strip() == name]
    if len(found) != 1:
        raise ValueError('Expected exactly one top-level entity: ' + name)
    return found[0]


def export(args):
    metadata_path = Path(args.metadata).expanduser().resolve(strict=True)
    metadata = json.loads(metadata_path.read_text())
    generator_path = Path(args.generator).expanduser().resolve(strict=True)
    spec = importlib.util.spec_from_file_location('wheeltec_original_generator', generator_path)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    paths = {}
    for key, override in (('texture', args.texture), ('model', args.model), ('world', args.world)):
        field = {'texture': 'output', 'model': 'model_sdf', 'world': 'world_sdf'}[key]
        value = override or metadata.get(field)
        if not value:
            raise ValueError('Missing --' + key + '; provide the installed asset used by Gazebo')
        paths[key] = Path(value).expanduser().resolve(strict=True)
    robot_path = args.robot_model or paths['model'].parent.parent / 'roboworks/model.sdf'
    paths['robot_model'] = Path(robot_path).expanduser().resolve(strict=True)
    paths['generator'] = generator_path
    size = float(metadata['track_size_m'])
    width = float(metadata['lane_width_m'])
    line = float(metadata['line_width_m'])
    generator.LANE_WIDTH_M, generator.LINE_WIDTH_M = width, line
    result = generator.generate_orthogonal_vertices(
        size, int(metadata['requested_turns']), int(metadata['seed']),
        float(metadata['corner_radius_m']), float(metadata['min_straight_m']),
        args.max_attempts, args.variation)
    vertices, center, left, right = result[:4]
    image = np.zeros((generator.IMAGE_SIZE_PX, generator.IMAGE_SIZE_PX, 3), dtype=np.uint8)
    image[:] = generator.BACKGROUND_BGR
    line_px = max(2, int(round(line / size * generator.IMAGE_SIZE_PX)))
    for boundary in (left, right):
        cv2.polylines(image, [generator.metric_to_pixel(boundary, size).reshape((-1, 1, 2))],
                      True, generator.LANE_BGR, line_px, cv2.LINE_AA)
    existing = cv2.imread(str(paths['texture']), cv2.IMREAD_COLOR)
    if existing is None or not np.array_equal(image, existing):
        raise ValueError('Texture mismatch: metadata/generator/variation is not the loaded track. '
                         'No reference exported. Use the original --variation and correct asset paths.')

    root = ET.parse(paths['world']).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError('Expected SDF world')
    track_entity = entity(world, 'racetrack')
    robot_entity = entity(world, 'roboworks')
    for e, name in ((track_entity, 'racetrack'), (robot_entity, 'roboworks')):
        expected = 'package://wheeltec_autorace_description/models/' + name
        if e.tag != 'include' or e.findtext('uri', '').strip() != expected:
            raise ValueError('Expected original top-level package include for ' + name)
    track_pose = pose(track_entity)
    spawn = pose(robot_entity)
    if np.any(np.abs(track_pose[3:5]) > 1e-9) or np.any(np.abs(spawn[3:5]) > 1e-9):
        raise ValueError('Only horizontal track and planar initial vehicle pose are supported')
    track = ET.parse(paths['model']).getroot().find('model')
    visual = track.find("link/visual[@name='track_visual']")
    if visual is None or track.find('link') is None:
        raise ValueError('Expected racetrack track_visual')
    for element in (track, track.find('link'), visual):
        p = pose(element)
        if np.any(np.abs(p[[0, 1, 3, 4, 5]]) > 1e-9):
            raise ValueError('Track model/link/visual XY and rotation must be zero')
    plane = visual.find('geometry/plane')
    if plane is None or not np.allclose([float(v) for v in plane.findtext('size').split()],
                                      [size, size], atol=1e-6, rtol=0):
        raise ValueError('Track visual plane dimensions do not match metadata')
    if not np.allclose([float(v) for v in plane.findtext('normal').split()], [0, 0, 1]):
        raise ValueError('Unsupported plane normal')
    # Verify that the supplied image is the asset addressed by this track model.
    albedo = visual.findtext('material/pbr/metal/albedo_map', '').strip()
    prefix = 'model://racetrack/'
    if not albedo.startswith(prefix) or (paths['model'].parent / albedo[len(prefix):]).resolve() != paths['texture']:
        raise ValueError('Texture path is not the model albedo_map')
    robot = ET.parse(paths['robot_model']).getroot().find('model')
    rl = robot.find("link[@name='rear_left_wheel']")
    rr = robot.find("link[@name='rear_right_wheel']")
    if rl is None or rr is None or not np.allclose((pose(rl)[:2]+pose(rr)[:2])/2, [0, 0], atol=1e-7):
        raise ValueError('Robot model origin is not the known rear axle midpoint')
    if np.any(np.abs(pose(robot)) > 1e-9):
        raise ValueError('Robot model-level pose must be zero')
    c, s = math.cos(track_pose[5]), math.sin(track_pose[5])
    rotation = np.array([[c, -s], [s, c]])
    world_center = (center-size/2) @ rotation.T + track_pose[:2]
    recommended = generator.recommended_spawn_pose(vertices, float(metadata['corner_radius_m']), size)
    gate = np.array(recommended[:2]) @ rotation.T + track_pose[:2]
    gate_yaw = float(recommended[3]) + track_pose[5]
    document = {
        'schema': 'wheeltec_reference_v1', 'world_name': world.get('name'),
        'model_name': 'roboworks', 'frame_id': 'world/' + world.get('name'),
        'lane_width_m': width, 'line_width_m': line,
        'centerline_world_xy_m': world_center.tolist(),
        'gate_world_xy_m': gate.tolist(), 'gate_heading_rad': gate_yaw,
        'world_spawn_pose': spawn.tolist(), 'track_world_pose': track_pose.tolist(),
        'generation_metadata': metadata, 'reconstruction_variation': args.variation,
        'texture_verification': 'pixel-identical BGR rendering from original generator',
        'reference_sampling': 'original generator closed polyline, nominal spacing 0.02 m',
        'assets': {k: {'path': str(p), 'sha256': digest_file(p)} for k, p in paths.items()},
    }
    ref = Reference(document)
    spawn_projection = ref.project(*spawn[:2])
    if abs(spawn_projection['cte_m']) > ref.inner_half_width:
        raise ValueError('Robot spawn is outside reference lane; check world/metadata pairing')
    document['reference_length_m'] = ref.length
    document['max_segment_length_m'] = float(np.max(ref.lengths))
    document['reference_id'] = content_digest(document)
    output = Path(args.output).expanduser()
    if output.resolve() in {metadata_path, *paths.values()}:
        raise ValueError('Reference output must not overwrite a generation input or asset')
    write_reference(output, document, overwrite=getattr(args, 'overwrite', False))
    print(f'Exported verified reference: {output}\nLength: {ref.length:.3f} m; '
          f'ID: {document["reference_id"]}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata', default='/tmp/wheeltec_random_track.json')
    parser.add_argument('--generator', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--overwrite', action='store_true',
                        help='Atomically replace the output only after all checks succeed')
    for key in ('texture', 'model', 'world', 'robot-model'):
        parser.add_argument('--'+key)
    parser.add_argument('--variation', type=float, default=0.65,
                        help='Must match generation; old metadata omits this parameter')
    parser.add_argument('--max-attempts', type=int, default=50000)
    args = parser.parse_args()
    try:
        export(args)
    except (ValueError, OSError, RuntimeError, KeyError) as exc:
        parser.exit(2, f'Reference export refused: {exc}\n')
