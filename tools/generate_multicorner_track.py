#!/usr/bin/env python3
"""
Wheeltec / Roboworks kompatibilis, derékszög-domináns 2D tesztpálya-generátor.

Fő célok:
- a pálya fizikai mérete 9x9 m és 15x15 m között állítható;
- minden pályában legalább 5 kanyar legyen;
- a kanyarok maximális irányváltozása 90 fok;
- a generált fő kanyarok alapértelmezetten pontosan 90 fokosak;
- két 90 fokos kanyar között mindig maradjon tényleges egyenes szakasz;
- a pálya egyszerű, zárt, önmetszésmentes legyen;
- a kanyar sugara maradjon a Wheeltec Ackermann modell fizikai korlátján belül;
- külön seed-del eltérő, de reprodukálható pályák készüljenek;
- nincs külső fehér keret vagy más felesleges fehér objektum.

A generátor véletlen, összefüggő rácsalakzat (polyomino) külső kontúrjából épít
ortogonális középvonalat. Emiatt a tényleges kanyarok 90 fokosak, de minden két
lekerekített kanyar között legalább --min-straight méter egyenes marad.

Megjegyzés:
Egyszerű zárt ortogonális poligonnak páros számú csúcsa van. Ha páratlan
--turns értéket kérsz, a generátor automatikusan a következő páros számra kerekít.
Például --turns 9 -> 10 tényleges kanyar.
"""

import argparse
import json
import math
import secrets
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Robot
# ---------------------------------------------------------------------------

WHEEL_BASE_M = 0.262
STEERING_LIMIT_RAD = 0.4
ROBOT_MIN_RADIUS_M = WHEEL_BASE_M / math.tan(STEERING_LIMIT_RAD)


# ---------------------------------------------------------------------------
# Pálya-alapértékek
# ---------------------------------------------------------------------------

MIN_TRACK_SIZE_M = 9.0
MAX_TRACK_SIZE_M = 15.0
DEFAULT_TRACK_SIZE_M = 9.1

MIN_TURNS = 5
MAX_TURNS = 24
DEFAULT_TURNS = 10

IMAGE_SIZE_PX = 2048

LANE_WIDTH_M = 0.65
LINE_WIDTH_M = 0.04

CORNER_RADIUS_M = 0.85
DEFAULT_MIN_STRAIGHT_M = 0.80

BACKGROUND_BGR = (32, 0, 24)
LANE_BGR = (255, 255, 255)
CENTERLINE_DEBUG_BGR = (0, 255, 255)
VERTEX_DEBUG_BGR = (0, 255, 0)


# ---------------------------------------------------------------------------
# Alap geometria
# ---------------------------------------------------------------------------

def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("Nullhosszu szakasz a palyageometriaban.")
    return v / n


def left_normal(v: np.ndarray) -> np.ndarray:
    return np.array([-v[1], v[0]], dtype=np.float64)


def signed_turn_angle(u_in: np.ndarray, u_out: np.ndarray) -> float:
    dot = float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))
    cross = float(u_in[0] * u_out[1] - u_in[1] * u_out[0])
    return math.atan2(cross, dot)


def orientation(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    ab = b - a
    ac = c - a
    return float(ab[0] * ac[1] - ab[1] * ac[0])


def on_segment(a: np.ndarray, b: np.ndarray, p: np.ndarray, eps=1e-9) -> bool:
    return (
        min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
        and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
        and abs(orientation(a, b, p)) <= eps
    )


def segments_intersect(a, b, c, d, eps=1e-9) -> bool:
    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)

    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and (
        (o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)
    ):
        return True

    if abs(o1) <= eps and on_segment(a, b, c, eps):
        return True
    if abs(o2) <= eps and on_segment(a, b, d, eps):
        return True
    if abs(o3) <= eps and on_segment(c, d, a, eps):
        return True
    if abs(o4) <= eps and on_segment(c, d, b, eps):
        return True

    return False


def polygon_self_intersects(vertices: np.ndarray) -> bool:
    n = len(vertices)
    for i in range(n):
        a = vertices[i]
        b = vertices[(i + 1) % n]

        for j in range(i + 1, n):
            if j == i:
                continue
            if (j + 1) % n == i or (i + 1) % n == j:
                continue

            c = vertices[j]
            d = vertices[(j + 1) % n]
            if segments_intersect(a, b, c, d):
                return True

    return False


def append_line(points, start, end, spacing=0.02):
    length = float(np.linalg.norm(end - start))
    count = max(2, int(math.ceil(length / spacing)) + 1)

    for s in np.linspace(0.0, 1.0, count, endpoint=False):
        points.append(start * (1.0 - s) + end * s)


def append_arc(points, center, start, end, turn_sign, radius, spacing=0.02):
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])

    if turn_sign > 0:
        while a1 <= a0:
            a1 += 2.0 * math.pi
    else:
        while a1 >= a0:
            a1 -= 2.0 * math.pi

    arc_angle = abs(a1 - a0)
    arc_length = radius * arc_angle
    count = max(3, int(math.ceil(arc_length / spacing)) + 1)

    for a in np.linspace(a0, a1, count, endpoint=False):
        points.append(
            center
            + radius * np.array([math.cos(a), math.sin(a)], dtype=np.float64)
        )


def corner_geometry(vertices: np.ndarray, radius: float):
    n = len(vertices)

    tangent_starts = []
    tangent_ends = []
    centers = []
    turn_signs = []
    turn_angles_rad = []
    tangent_distances = []

    for i in range(n):
        prev = vertices[(i - 1) % n]
        v = vertices[i]
        nxt = vertices[(i + 1) % n]

        u_in = unit(v - prev)
        u_out = unit(nxt - v)

        theta = signed_turn_angle(u_in, u_out)
        abs_deg = abs(math.degrees(theta))

        # Az ortogonális kontúr minden valódi csúcsa pontosan 90 fokos.
        if abs(abs_deg - 90.0) > 1e-6:
            raise RuntimeError(
                f"Nem derékszögű csúcs a(z) {i}. helyen: {abs_deg:.6f} fok."
            )

        turn_sign = 1.0 if theta > 0.0 else -1.0
        tangent_distance = radius  # 90 foknál R * tan(45°) = R

        start = v - u_in * tangent_distance
        end = v + u_out * tangent_distance
        center = start + turn_sign * left_normal(u_in) * radius

        tangent_starts.append(start)
        tangent_ends.append(end)
        centers.append(center)
        turn_signs.append(turn_sign)
        turn_angles_rad.append(theta)
        tangent_distances.append(tangent_distance)

    return (
        tangent_starts,
        tangent_ends,
        centers,
        turn_signs,
        np.asarray(turn_angles_rad, dtype=np.float64),
        np.asarray(tangent_distances, dtype=np.float64),
    )


def build_rounded_centerline(vertices: np.ndarray, radius: float, min_straight_m: float):
    (
        tangent_starts,
        tangent_ends,
        centers,
        turn_signs,
        turn_angles_rad,
        tangent_distances,
    ) = corner_geometry(vertices, radius)

    segment_lengths = []
    remaining_straights = []

    for i in range(len(vertices)):
        length = float(np.linalg.norm(vertices[(i + 1) % len(vertices)] - vertices[i]))
        remaining = length - tangent_distances[i] - tangent_distances[(i + 1) % len(vertices)]

        segment_lengths.append(length)
        remaining_straights.append(remaining)

        if remaining < min_straight_m - 1e-8:
            raise RuntimeError(
                f"Tul rovid egyenes a(z) {i}. es {(i + 1) % len(vertices)}. "
                f"90 fokos kanyar kozott: {remaining:.3f} m < {min_straight_m:.3f} m."
            )

    rounded = []

    for i in range(len(vertices)):
        prev_i = (i - 1) % len(vertices)

        append_line(rounded, tangent_ends[prev_i], tangent_starts[i])
        append_arc(
            rounded,
            centers[i],
            tangent_starts[i],
            tangent_ends[i],
            turn_signs[i],
            radius,
        )

    return (
        np.asarray(rounded, dtype=np.float64),
        np.asarray(segment_lengths, dtype=np.float64),
        np.asarray(remaining_straights, dtype=np.float64),
        turn_angles_rad,
    )


def tangent_and_normal(points: np.ndarray):
    prev = np.roll(points, 1, axis=0)
    nxt = np.roll(points, -1, axis=0)

    tangent = nxt - prev
    length = np.linalg.norm(tangent, axis=1)
    length = np.maximum(length, 1e-12)

    tx = tangent[:, 0] / length
    ty = tangent[:, 1] / length

    nx = -ty
    ny = tx

    return np.column_stack((tx, ty)), np.column_stack((nx, ny))


def offset_boundaries(centerline: np.ndarray, lane_width: float):
    _, normal = tangent_and_normal(centerline)
    half = lane_width / 2.0
    return centerline + half * normal, centerline - half * normal


def metric_to_pixel(points: np.ndarray, track_size_m: float):
    scale = (IMAGE_SIZE_PX - 1) / track_size_m
    px = np.rint(points[:, 0] * scale).astype(np.int32)
    py = np.rint((track_size_m - points[:, 1]) * scale).astype(np.int32)
    return np.column_stack((px, py))


def polyline_length(points: np.ndarray) -> float:
    nxt = np.roll(points, -1, axis=0)
    return float(np.sum(np.linalg.norm(nxt - points, axis=1)))


def check_bounds(track_size_m: float, stroke_margin_m: float, *curves):
    for curve in curves:
        if (
            np.min(curve[:, 0]) < stroke_margin_m
            or np.max(curve[:, 0]) > track_size_m - stroke_margin_m
            or np.min(curve[:, 1]) < stroke_margin_m
            or np.max(curve[:, 1]) > track_size_m - stroke_margin_m
        ):
            raise RuntimeError(
                "A savhatar vagy a feher vonal vastagsaga kikerulne a texturabol."
            )


# ---------------------------------------------------------------------------
# Polyomino / ortogonális véletlen pályagenerálás
# ---------------------------------------------------------------------------

def canonical_edge(a, b):
    return (a, b) if a <= b else (b, a)


def polyomino_boundary_loop(cells: set[tuple[int, int]]):
    """A cellahalmaz egyetlen külső rácskontúrját adja vissza."""
    edges = set()

    for x, y in cells:
        corners = [
            (x, y),
            (x + 1, y),
            (x + 1, y + 1),
            (x, y + 1),
        ]

        for a, b in zip(corners, corners[1:] + corners[:1]):
            edge = canonical_edge(a, b)
            if edge in edges:
                edges.remove(edge)
            else:
                edges.add(edge)

    adjacency = defaultdict(list)
    for a, b in edges:
        adjacency[a].append(b)
        adjacency[b].append(a)

    # Diagonális érintkezés / nem egyszerű kontúr esetén lehet 4 fokú csúcs.
    if not adjacency or any(len(neighbors) != 2 for neighbors in adjacency.values()):
        return None

    unseen = set(edges)
    loops = []

    while unseen:
        first_edge = next(iter(unseen))
        start = first_edge[0]
        current = start
        previous = None
        loop = []

        while True:
            loop.append(current)
            neighbors = adjacency[current]
            nxt = neighbors[0] if neighbors[0] != previous else neighbors[1]

            unseen.discard(canonical_edge(current, nxt))
            previous, current = current, nxt

            if current == start:
                break

            if len(loop) > len(edges) + 2:
                return None

        loops.append(loop)

    # Több loop lyukat jelentene. Olyan pályát most nem használunk.
    if len(loops) != 1:
        return None

    points = loops[0]

    # Egymás után következő kollineáris rácspontok összevonása, hogy csak a
    # valódi 90 fokos kanyarok maradjanak csúcsként.
    changed = True
    while changed and len(points) >= 4:
        changed = False
        reduced = []
        n = len(points)

        for i, p in enumerate(points):
            prev = points[(i - 1) % n]
            nxt = points[(i + 1) % n]

            collinear = (
                prev[0] == p[0] == nxt[0]
                or prev[1] == p[1] == nxt[1]
            )

            if collinear:
                changed = True
            else:
                reduced.append(p)

        points = reduced

    return points


def candidate_growth_cells(
    grid_size: int,
    rng: np.random.Generator,
    variation: float,
):
    """Véletlen összefüggő cellahalmazt növeszt a rácson."""
    start = (
        int(rng.integers(0, grid_size)),
        int(rng.integers(0, grid_size)),
    )
    cells = {start}

    # A variation a kitöltött cellák számának szórását befolyásolja.
    min_cells = 2
    max_cells = max(min_cells, grid_size * grid_size - 1)
    center_fraction = 0.40 + 0.35 * variation
    center_count = int(round(min_cells + center_fraction * (max_cells - min_cells)))
    spread = max(1, int(round((0.15 + 0.35 * variation) * max_cells)))

    desired_cells = int(np.clip(
        center_count + rng.integers(-spread, spread + 1),
        min_cells,
        max_cells,
    ))

    while len(cells) < desired_cells:
        frontier = set()

        for x, y in cells:
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                q = (x + dx, y + dy)
                if (
                    0 <= q[0] < grid_size
                    and 0 <= q[1] < grid_size
                    and q not in cells
                ):
                    frontier.add(q)

        if not frontier:
            break

        options = list(frontier)
        chosen = options[int(rng.integers(0, len(options)))]
        cells.add(chosen)

    return cells


def actual_turn_target(requested_turns: int) -> int:
    target = max(MIN_TURNS, requested_turns)
    if target % 2 != 0:
        target += 1
    return target


def generate_orthogonal_vertices(
    track_size_m: float,
    requested_turns: int,
    seed: int,
    corner_radius_m: float,
    min_straight_m: float,
    max_attempts: int,
    variation: float,
):
    target_turns = actual_turn_target(requested_turns)

    # 90 fokos fillet mindkét oldalon R hosszt vesz el egy nyers szakaszból.
    min_raw_segment_m = 2.0 * corner_radius_m + min_straight_m

    # A középvonal és a teljes fehér jelölés maradjon biztonságosan a textúrán belül.
    geometry_margin_m = LANE_WIDTH_M / 2.0 + LINE_WIDTH_M / 2.0 + 0.20
    usable_extent_m = track_size_m - 2.0 * geometry_margin_m

    grid_size = int(math.floor(usable_extent_m / min_raw_segment_m))

    if grid_size < 2:
        raise RuntimeError(
            "A palya merete tul kicsi a kert kanyarsugarhoz es egyenes-hosszhoz."
        )

    # A teljes rendelkezésre álló területet használjuk; így a cellaél legalább
    # min_raw_segment_m hosszú, gyakran valamivel hosszabb.
    cell_size_m = usable_extent_m / grid_size
    offset_m = geometry_margin_m

    rng = np.random.default_rng(seed)
    last_seen_turns = None

    for attempt in range(1, max_attempts + 1):
        cells = candidate_growth_cells(grid_size, rng, variation)
        grid_loop = polyomino_boundary_loop(cells)

        if grid_loop is None:
            continue

        if len(grid_loop) != target_turns:
            last_seen_turns = len(grid_loop)
            continue

        vertices = np.asarray(
            [
                [
                    offset_m + x * cell_size_m,
                    offset_m + y * cell_size_m,
                ]
                for x, y in grid_loop
            ],
            dtype=np.float64,
        )

        if polygon_self_intersects(vertices):
            continue

        try:
            center, segment_lengths, remaining_straights, turn_angles_rad = (
                build_rounded_centerline(
                    vertices,
                    corner_radius_m,
                    min_straight_m,
                )
            )

            left, right = offset_boundaries(center, LANE_WIDTH_M)
            stroke_margin_m = LINE_WIDTH_M / 2.0 + 0.02
            check_bounds(track_size_m, stroke_margin_m, center, left, right)

            # Védőellenőrzés: minden kanyar pontosan 90 fokos legyen.
            angles_deg = np.abs(np.degrees(turn_angles_rad))
            if not np.all(np.abs(angles_deg - 90.0) < 1e-6):
                continue

            return (
                vertices,
                center,
                left,
                right,
                segment_lengths,
                remaining_straights,
                turn_angles_rad,
                attempt,
                grid_size,
                cell_size_m,
                len(cells),
                target_turns,
            )

        except RuntimeError:
            continue

    raise RuntimeError(
        f"{max_attempts} probalkozasbol sem sikerult {target_turns} darab 90 fokos "
        f"kanyart generalni {track_size_m:.1f} x {track_size_m:.1f} m-en. "
        f"A racs {grid_size}x{grid_size}; utolso talalt kontur: {last_seen_turns}.\n"
        "Probald masik --seed ertekkel, kisebb --turns ertekkel, nagyobb "
        "--track-size merettel vagy nagyobb --max-attempts ertekkel."
    )


# ---------------------------------------------------------------------------
# Spawn / renderelés
# ---------------------------------------------------------------------------

def recommended_spawn_pose(vertices: np.ndarray, radius: float, track_size_m: float, z_m=0.15):
    (
        tangent_starts,
        tangent_ends,
        _centers,
        _turn_signs,
        _turn_angles_rad,
        _tangent_distances,
    ) = corner_geometry(vertices, radius)

    best_length = -1.0
    best_start = None
    best_end = None

    for i in range(len(vertices)):
        start = tangent_ends[i]
        end = tangent_starts[(i + 1) % len(vertices)]
        length = float(np.linalg.norm(end - start))

        if length > best_length:
            best_length = length
            best_start = start
            best_end = end

    direction = unit(best_end - best_start)
    midpoint = 0.5 * (best_start + best_end)

    world_x = float(midpoint[0] - track_size_m / 2.0)
    world_y = float(midpoint[1] - track_size_m / 2.0)
    yaw = math.atan2(float(direction[1]), float(direction[0]))

    return world_x, world_y, z_m, yaw, best_length



def _xml_tree(path: Path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def _parse_pose_text(text: str | None):
    values = [0.0] * 6
    if text:
        parts = text.split()
        for i, part in enumerate(parts[:6]):
            values[i] = float(part)
    return values


def _format_pose(values):
    return (
        f"{values[0]:.6f} {values[1]:.6f} {values[2]:.6f} "
        f"{values[3]:.6f} {values[4]:.6f} {values[5]:.6f}"
    )


def _find_world_entity(root, keyword: str):
    keyword = keyword.lower()

    for include in root.iter("include"):
        name = (include.findtext("name") or "").lower()
        uri = (include.findtext("uri") or "").lower()
        if keyword in name or keyword in uri:
            return include

    for model in root.iter("model"):
        name = (model.get("name") or "").lower()
        if keyword in name:
            return model

    return None


def _get_or_create_pose(entity):
    pose = entity.find("pose")
    if pose is None:
        pose = ET.SubElement(entity, "pose")
    return pose


def update_racetrack_model_size(model_sdf: Path, track_size_m: float):
    """A racetrack fő sík/box geometriájának X/Y méretét frissíti."""
    tree = _xml_tree(model_sdf)
    root = tree.getroot()

    plane_sizes = list(root.findall(".//geometry/plane/size"))

    if plane_sizes:
        targets = plane_sizes
    else:
        # Ha box alapú a pálya, csak a legnagyobb alapterületű box(ok) méretét
        # módosítjuk, így kisebb segédgeometriákat nem írunk át véletlenül.
        box_sizes = list(root.findall(".//geometry/box/size"))
        parsed = []
        for elem in box_sizes:
            try:
                vals = [float(v) for v in (elem.text or "").split()]
            except ValueError:
                continue
            if len(vals) >= 2:
                parsed.append((elem, vals, vals[0] * vals[1]))

        if not parsed:
            raise RuntimeError(
                f"Nem talaltam <plane><size> vagy hasznalhato <box><size> elemet: {model_sdf}"
            )

        max_area = max(item[2] for item in parsed)
        targets = [item[0] for item in parsed if item[2] >= 0.95 * max_area]

    for elem in targets:
        vals = (elem.text or "").split()
        if len(vals) == 2:
            elem.text = f"{track_size_m:.3f} {track_size_m:.3f}"
        elif len(vals) >= 3:
            elem.text = (
                f"{track_size_m:.3f} {track_size_m:.3f} "
                + " ".join(vals[2:])
            )
        else:
            raise RuntimeError(f"Ervenytelen geometry size: {elem.text!r}")

    ET.indent(tree, space="  ")
    tree.write(model_sdf, encoding="utf-8", xml_declaration=True)


def update_world_spawn_pose(
    world_sdf: Path,
    local_spawn_pose,
):
    """
    A robot local racetrack koordinátás spawnját világkoordinátára transzformálja,
    majd frissíti a roboworks <pose> értékét.

    A racetrack world pose X/Y/yaw értékét figyelembe vesszük, ezért a pálya
    nem szükséges, hogy a világ origójában álljon.
    """
    tree = _xml_tree(world_sdf)
    root = tree.getroot()

    track_entity = _find_world_entity(root, "racetrack")
    robot_entity = _find_world_entity(root, "roboworks")

    if robot_entity is None:
        raise RuntimeError(
            f"Nem talaltam roboworks include/model elemet a world fajlban: {world_sdf}"
        )

    track_pose_values = [0.0] * 6
    if track_entity is not None:
        track_pose = track_entity.find("pose")
        track_pose_values = _parse_pose_text(
            None if track_pose is None else track_pose.text
        )

    lx, ly, lz, lroll, lpitch, lyaw = local_spawn_pose
    tx, ty, tz, troll, tpitch, tyaw = track_pose_values

    c = math.cos(tyaw)
    s = math.sin(tyaw)

    world_x = tx + c * lx - s * ly
    world_y = ty + s * lx + c * ly
    world_z = tz + lz
    world_yaw = math.atan2(
        math.sin(tyaw + lyaw),
        math.cos(tyaw + lyaw),
    )

    world_pose = [
        world_x,
        world_y,
        world_z,
        lroll + troll,
        lpitch + tpitch,
        world_yaw,
    ]

    pose_elem = _get_or_create_pose(robot_entity)
    pose_elem.text = _format_pose(world_pose)

    ET.indent(tree, space="  ")
    tree.write(world_sdf, encoding="utf-8", xml_declaration=True)

    return world_pose


def generate(
    output_path: Path,
    debug_path: Path | None,
    track_size_m: float,
    turns: int,
    seed: int,
    corner_radius_m: float,
    min_straight_m: float,
    max_attempts: int,
    variation: float,
    model_sdf: Path | None = None,
    world_sdf: Path | None = None,
    metadata_output: Path | None = None,
):
    if not (MIN_TRACK_SIZE_M <= track_size_m <= MAX_TRACK_SIZE_M):
        raise ValueError(
            f"--track-size csak {MIN_TRACK_SIZE_M:.1f} es {MAX_TRACK_SIZE_M:.1f} m kozott lehet."
        )

    if turns < MIN_TURNS:
        raise ValueError(f"Legalabb {MIN_TURNS} kanyart kerj.")

    if turns > MAX_TURNS:
        raise ValueError(f"Legfeljebb {MAX_TURNS} kanyart kerj.")

    if not (0.0 <= variation <= 1.0):
        raise ValueError("--variation 0.0 es 1.0 kozott lehet.")

    if corner_radius_m <= ROBOT_MIN_RADIUS_M:
        raise ValueError(
            "A kanyar sugara kisebb vagy egyenlo a robot elmeleti minimumanal: "
            f"R={corner_radius_m:.3f} m, R_min={ROBOT_MIN_RADIUS_M:.3f} m."
        )

    (
        vertices,
        center,
        left,
        right,
        segment_lengths,
        remaining_straights,
        turn_angles_rad,
        attempt,
        grid_size,
        cell_size_m,
        occupied_cells,
        actual_turns,
    ) = generate_orthogonal_vertices(
        track_size_m,
        turns,
        seed,
        corner_radius_m,
        min_straight_m,
        max_attempts,
        variation,
    )

    image = np.zeros((IMAGE_SIZE_PX, IMAGE_SIZE_PX, 3), dtype=np.uint8)
    image[:, :] = BACKGROUND_BGR

    left_px = metric_to_pixel(left, track_size_m)
    right_px = metric_to_pixel(right, track_size_m)
    center_px = metric_to_pixel(center, track_size_m)
    vertex_px = metric_to_pixel(vertices, track_size_m)

    line_width_px = max(
        2,
        int(round(LINE_WIDTH_M / track_size_m * IMAGE_SIZE_PX)),
    )

    cv2.polylines(
        image,
        [left_px.reshape((-1, 1, 2))],
        True,
        LANE_BGR,
        line_width_px,
        cv2.LINE_AA,
    )

    cv2.polylines(
        image,
        [right_px.reshape((-1, 1, 2))],
        True,
        LANE_BGR,
        line_width_px,
        cv2.LINE_AA,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Nem sikerult elmenteni: {output_path}")

    if debug_path is not None:
        debug = image.copy()

        cv2.polylines(
            debug,
            [center_px.reshape((-1, 1, 2))],
            True,
            CENTERLINE_DEBUG_BGR,
            max(2, line_width_px // 6),
            cv2.LINE_AA,
        )

        for p in vertex_px:
            cv2.circle(
                debug,
                tuple(p),
                7,
                VERTEX_DEBUG_BGR,
                -1,
                cv2.LINE_AA,
            )

            cv2.putText(
                debug,
                "90",
                (int(p[0]) + 8, int(p[1]) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 180, 255),
                1,
                cv2.LINE_AA,
            )

        debug_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(debug_path), debug):
            raise RuntimeError(f"Nem sikerult elmenteni: {debug_path}")

    safety_factor = corner_radius_m / ROBOT_MIN_RADIUS_M
    inner_boundary_radius = corner_radius_m - LANE_WIDTH_M / 2.0
    visible_dark_gap = LANE_WIDTH_M - LINE_WIDTH_M
    angles_deg = np.abs(np.degrees(turn_angles_rad))

    spawn_x, spawn_y, spawn_z, spawn_yaw, spawn_straight_length = recommended_spawn_pose(
        vertices,
        corner_radius_m,
        track_size_m,
    )

    local_spawn_pose = [
        spawn_x,
        spawn_y,
        spawn_z,
        0.0,
        0.0,
        spawn_yaw,
    ]
    world_spawn_pose = list(local_spawn_pose)

    if model_sdf is not None:
        update_racetrack_model_size(model_sdf, track_size_m)

    if world_sdf is not None:
        world_spawn_pose = update_world_spawn_pose(
            world_sdf,
            local_spawn_pose,
        )

    metadata = {
        "track_size_m": float(track_size_m),
        "requested_turns": int(turns),
        "actual_turns": int(actual_turns),
        "seed": int(seed),
        "corner_radius_m": float(corner_radius_m),
        "min_straight_m": float(min_straight_m),
        "lane_width_m": float(LANE_WIDTH_M),
        "line_width_m": float(LINE_WIDTH_M),
        "local_spawn_pose": [float(v) for v in local_spawn_pose],
        "world_spawn_pose": [float(v) for v in world_spawn_pose],
        "spawn_straight_length_m": float(spawn_straight_length),
        "output": str(output_path),
        "debug_output": None if debug_path is None else str(debug_path),
        "model_sdf": None if model_sdf is None else str(model_sdf),
        "world_sdf": None if world_sdf is None else str(world_sdf),
    }

    if metadata_output is not None:
        metadata_output.parent.mkdir(parents=True, exist_ok=True)
        metadata_output.write_text(
            json.dumps(metadata, indent=2) + "\n",
            encoding="utf-8",
        )

    print("=== Wheeltec right-angle-heavy racetrack ===")
    print(f"Texture:                   {IMAGE_SIZE_PX} x {IMAGE_SIZE_PX} px")
    print(f"Physical size:             {track_size_m:.2f} x {track_size_m:.2f} m")
    print(f"Requested turns:           {turns}")
    print(f"Actual 90-deg turns:       {actual_turns}")
    print(f"Right-angle ratio:         100%")
    print(f"Seed:                      {seed}")
    print(f"Generation attempt:        {attempt}/{max_attempts}")
    print(f"Grid:                      {grid_size} x {grid_size}")
    print(f"Grid cell size:            {cell_size_m:.3f} m")
    print(f"Occupied cells:            {occupied_cells}")
    print()
    print(f"Lane width:                {LANE_WIDTH_M:.2f} m")
    print(f"White line width:          {LINE_WIDTH_M:.2f} m")
    print(f"Dark gap between lines:    {visible_dark_gap:.2f} m")
    print()
    print(f"Robot theoretical R_min:   {ROBOT_MIN_RADIUS_M:.3f} m")
    print(f"Track corner radius:       {corner_radius_m:.3f} m")
    print(f"Radius safety factor:      {safety_factor:.2f} x")
    print(f"Inner boundary radius:     {inner_boundary_radius:.3f} m")
    print(f"Min corner angle:          {np.min(angles_deg):.1f} deg")
    print(f"Max corner angle:          {np.max(angles_deg):.1f} deg")
    print()
    print(f"Shortest raw segment:      {np.min(segment_lengths):.3f} m")
    print(f"Shortest straight between")
    print(f"two rounded 90-deg turns:  {np.min(remaining_straights):.3f} m")
    print(f"Required min straight:     {min_straight_m:.3f} m")
    print(f"Approx. centerline length: {polyline_length(center):.3f} m")
    print()
    print(f"Output: {output_path}")

    if debug_path is not None:
        print(f"Debug:  {debug_path}")

    print()
    print("Gazebo model.sdf size requirement:")
    print(f"  <size>{track_size_m:.2f} {track_size_m:.2f}</size>")
    print()
    print("Recommended robot spawn pose (longest straight center):")
    print(f"  straight length: {spawn_straight_length:.3f} m")
    print(
        f"  <pose>{spawn_x:.3f} {spawn_y:.3f} {spawn_z:.3f} "
        f"0 0 {spawn_yaw:.6f}</pose>"
    )

    if world_sdf is not None:
        print("World roboworks pose:")
        print(f"  <pose>{_format_pose(world_spawn_pose)}</pose>")

    if metadata_output is not None:
        print(f"Metadata: {metadata_output}")

    if turns % 2 != 0:
        print()
        print(
            f"MEGJEGYZES: --turns {turns} paratlan volt, ezert az ortogonalis zart "
            f"palya {actual_turns} kanyarral keszult."
        )

    if visible_dark_gap < 0.32:
        print()
        print("FIGYELEM:")
        print(
            f"  A LANE_WIDTH_M={LANE_WIDTH_M:.2f} m es LINE_WIDTH_M={LINE_WIDTH_M:.2f} m "
            f"csak {visible_dark_gap:.2f} m sotet savot hagy a ket feher vonal kozott."
        )

    return metadata


def _even_turn_choices(min_turns: int, max_turns: int):
    lo = max(MIN_TURNS, min_turns)
    hi = min(MAX_TURNS, max_turns)

    if lo % 2:
        lo += 1
    if hi % 2:
        hi -= 1

    return list(range(lo, hi + 1, 2))


def generate_random(
    output_path: Path,
    debug_path: Path | None,
    min_track_size_m: float,
    max_track_size_m: float,
    min_random_turns: int,
    max_random_turns: int,
    corner_radius_m: float,
    min_straight_m: float,
    max_attempts: int,
    variation: float,
    model_sdf: Path | None,
    world_sdf: Path | None,
    metadata_output: Path | None,
    random_layout_attempts: int,
):
    if min_track_size_m > max_track_size_m:
        raise ValueError("--min-track-size nem lehet nagyobb, mint --max-track-size.")

    min_track_size_m = max(MIN_TRACK_SIZE_M, min_track_size_m)
    max_track_size_m = min(MAX_TRACK_SIZE_M, max_track_size_m)

    choices = _even_turn_choices(min_random_turns, max_random_turns)
    if not choices:
        raise ValueError("Nincs ervenyes paros kanyarszam a random tartomanyban.")

    last_error = None

    for _ in range(max(1, random_layout_attempts)):
        # 0.1 m-es lépések: könnyen olvasható és reprodukálható SDF méretek.
        steps = int(round((max_track_size_m - min_track_size_m) * 10.0))
        track_size_m = min_track_size_m + secrets.randbelow(steps + 1) / 10.0

        # Nagyobb pályán több kanyart kérünk; az elérhető tartomány felső feléből
        # választunk, hogy a tesztpályák valóban technikásak legyenek.
        size_ratio = (
            (track_size_m - MIN_TRACK_SIZE_M)
            / max(1e-9, MAX_TRACK_SIZE_M - MIN_TRACK_SIZE_M)
        )
        suggested_cap = int(round(10 + 10 * size_ratio))
        if suggested_cap % 2:
            suggested_cap -= 1

        feasible = [t for t in choices if t <= suggested_cap]
        if not feasible:
            feasible = choices[:]

        upper_half = feasible[max(0, len(feasible) // 2):]
        candidate_turns = upper_half[secrets.randbelow(len(upper_half))]
        seed = secrets.randbits(31)

        try:
            return generate(
                output_path=output_path,
                debug_path=debug_path,
                track_size_m=track_size_m,
                turns=candidate_turns,
                seed=seed,
                corner_radius_m=corner_radius_m,
                min_straight_m=min_straight_m,
                max_attempts=max_attempts,
                variation=variation,
                model_sdf=model_sdf,
                world_sdf=world_sdf,
                metadata_output=metadata_output,
            )
        except (RuntimeError, ValueError) as exc:
            last_error = exc

    # Biztonsági fallback: közepes méret, mérsékelt, de továbbra is technikás
    # derékszögű pálya. Így egy ritka random generálási hiba miatt nem indul el
    # régi / inkonzisztens pályával a szimuláció.
    fallback_size = min(max(12.0, min_track_size_m), max_track_size_m)
    fallback_turns = min((t for t in choices if t <= 12), default=choices[0])
    fallback_seed = secrets.randbits(31)

    try:
        return generate(
            output_path=output_path,
            debug_path=debug_path,
            track_size_m=fallback_size,
            turns=fallback_turns,
            seed=fallback_seed,
            corner_radius_m=corner_radius_m,
            min_straight_m=min_straight_m,
            max_attempts=max(max_attempts, 12000),
            variation=variation,
            model_sdf=model_sdf,
            world_sdf=world_sdf,
            metadata_output=metadata_output,
        )
    except Exception as fallback_error:
        raise RuntimeError(
            "A random palyageneralas es a fallback generalas is sikertelen. "
            f"Utolso random hiba: {last_error}; fallback hiba: {fallback_error}"
        ) from fallback_error


def main():
    parser = argparse.ArgumentParser(
        description="Derékszög-domináns Wheeltec 2D tesztpálya-generátor."
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("course_multicorner.png"),
        help="A Gazebo altal hasznalhato normal texturakep.",
    )

    parser.add_argument(
        "--debug-output",
        type=Path,
        default=Path("course_multicorner_debug.png"),
        help="Debug kep kozepvonallal es 90 fokos csucspontokkal.",
    )

    parser.add_argument(
        "--track-size",
        type=float,
        default=DEFAULT_TRACK_SIZE_M,
        help=f"Fizikai negyzetes palyameret meterben ({MIN_TRACK_SIZE_M}-{MAX_TRACK_SIZE_M}).",
    )

    parser.add_argument(
        "--turns",
        type=int,
        default=DEFAULT_TURNS,
        help=(
            f"Kert kanyarszam ({MIN_TURNS}-{MAX_TURNS}). "
            "Paratlan erteket a kovetkezo parosra kerekiti."
        ),
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=1,
        help="Veletlen seed. Azonos seed + parameterek ugyanazt a palyat adjak.",
    )

    parser.add_argument(
        "--corner-radius",
        type=float,
        default=CORNER_RADIUS_M,
        help="A lekerekitett 90 fokos kanyarok kozepvonali sugara meterben.",
    )

    parser.add_argument(
        "--min-straight",
        type=float,
        default=DEFAULT_MIN_STRAIGHT_M,
        help="Minimalis tenyleges egyenes ket lekerekitett 90 fokos kanyar kozott meterben.",
    )

    parser.add_argument(
        "--variation",
        type=float,
        default=0.65,
        help=(
            "A veletlen cellaalakzat valtozatossaga 0.0-1.0 kozott. "
            "Nagyobb ertek valtozatosabb kitoltest ad."
        ),
    )

    parser.add_argument(
        "--max-attempts",
        type=int,
        default=6000,
        help="Ennyi veletlen geometriat probaljon, mielott hibat ad.",
    )

    parser.add_argument(
        "--random",
        action="store_true",
        help="Minden futasnal uj random palyameretet, kanyarszamot es seedet valaszt.",
    )

    parser.add_argument(
        "--min-track-size",
        type=float,
        default=MIN_TRACK_SIZE_M,
        help="Random mod minimalis palyamerete meterben.",
    )

    parser.add_argument(
        "--max-track-size",
        type=float,
        default=MAX_TRACK_SIZE_M,
        help="Random mod maximalis palyamerete meterben.",
    )

    parser.add_argument(
        "--min-random-turns",
        type=int,
        default=6,
        help="Random mod minimalis kert kanyarszama.",
    )

    parser.add_argument(
        "--max-random-turns",
        type=int,
        default=20,
        help="Random mod maximalis kert kanyarszama.",
    )

    parser.add_argument(
        "--random-layout-attempts",
        type=int,
        default=40,
        help="Hany kulonbozo random meret/kanyarszam/seed kombinaciot probaljon.",
    )

    parser.add_argument(
        "--model-sdf",
        type=Path,
        default=None,
        help="Ha megadod, a racetrack model fo geometry size erteket automatikusan frissiti.",
    )

    parser.add_argument(
        "--world-sdf",
        type=Path,
        default=None,
        help="Ha megadod, a roboworks pose erteket automatikusan a generalt palyahoz allitja.",
    )

    parser.add_argument(
        "--metadata-output",
        type=Path,
        default=None,
        help="JSON fajl a generalt palya meretevel, seeddel es spawn pose-zal.",
    )

    args = parser.parse_args()

    if args.random:
        generate_random(
            output_path=args.output,
            debug_path=args.debug_output,
            min_track_size_m=args.min_track_size,
            max_track_size_m=args.max_track_size,
            min_random_turns=args.min_random_turns,
            max_random_turns=args.max_random_turns,
            corner_radius_m=args.corner_radius,
            min_straight_m=args.min_straight,
            max_attempts=max(1, args.max_attempts),
            variation=args.variation,
            model_sdf=args.model_sdf,
            world_sdf=args.world_sdf,
            metadata_output=args.metadata_output,
            random_layout_attempts=max(1, args.random_layout_attempts),
        )
    else:
        generate(
            output_path=args.output,
            debug_path=args.debug_output,
            track_size_m=args.track_size,
            turns=args.turns,
            seed=args.seed,
            corner_radius_m=args.corner_radius,
            min_straight_m=args.min_straight,
            max_attempts=max(1, args.max_attempts),
            variation=args.variation,
            model_sdf=args.model_sdf,
            world_sdf=args.world_sdf,
            metadata_output=args.metadata_output,
        )


if __name__ == "__main__":
    main()
