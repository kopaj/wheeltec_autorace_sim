#!/usr/bin/env python3
"""
Wheeltec / Roboworks kompatibilis 2D versenypálya generátor.

A cél:
- több kanyar legyen, mint az előző "ovális" biztonságos pályán;
- legyenek 90 fokos irányváltások is;
- két 90 fokos fordító között mindig maradjon tényleges egyenes szakasz;
- a kanyar sugara maradjon a Wheeltec Ackermann modell fizikai korlátján belül;
- ne legyen külső fehér keret vagy más felesleges fehér objektum.

Robotparaméterek:
    steering_limit = 0.4 rad
    wheel_base     = 0.262 m

Elméleti minimális fordulási sugár:
    R_min = wheel_base / tan(steering_limit) ~= 0.620 m

A generált pálya minden 90 fokos ívének középvonali sugara:
    R_corner = 0.85 m

Ez kb. 1.37x sugártartalékot ad a robot elméleti minimumához képest.
"""

import argparse
import math
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
# Pálya
# ---------------------------------------------------------------------------

TRACK_SIZE_M = 9.1
IMAGE_SIZE_PX = 2048

LANE_WIDTH_M = 0.65
LINE_WIDTH_M = 0.04

# A 90 fokos fordítók középvonali sugara.
CORNER_RADIUS_M = 0.85

BACKGROUND_BGR = (32, 0, 24)
LANE_BGR = (255, 255, 255)

# Zárt, egyszerű ortogonális középvonal-poligon.
#
# Forma:
# - nagy külső téglalap
# - felül egy széles belső "öböl"
#
# Ez összesen 8 db 90 fokos irányváltást ad.
# A rövidebb szakaszok is elég hosszúak ahhoz, hogy az ívek között
# valódi egyenes rész maradjon.
CENTERLINE_VERTICES = np.array([
    [1.00, 1.00],
    [8.10, 1.00],
    [8.10, 8.10],
    [5.80, 8.10],
    [5.80, 5.40],
    [3.30, 5.40],
    [3.30, 8.10],
    [1.00, 8.10],
], dtype=np.float64)


# ---------------------------------------------------------------------------
# Segédfüggvények
# ---------------------------------------------------------------------------

def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Nullhosszu szakasz a palyageometriaban.")
    return v / n


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


def build_rounded_centerline(vertices: np.ndarray, radius: float):
    n = len(vertices)

    # Ellenőrizzük, mennyi tényleges egyenes marad az ívek között.
    segment_lengths = []
    remaining_straights = []

    for i in range(n):
        p0 = vertices[i]
        p1 = vertices[(i + 1) % n]

        length = float(np.linalg.norm(p1 - p0))
        segment_lengths.append(length)

        # Mindkét végéről radius hossz vágódik le egy 90 fokos ív miatt.
        remaining_straights.append(length - 2.0 * radius)

    min_remaining = min(remaining_straights)

    if min_remaining <= 0.0:
        raise RuntimeError(
            "A CORNER_RADIUS_M tul nagy: ket szomszedos kanyar osszeerne."
        )

    rounded = []

    tangent_starts = []
    tangent_ends = []
    centers = []
    turn_signs = []

    for i in range(n):
        prev = vertices[(i - 1) % n]
        v = vertices[i]
        nxt = vertices[(i + 1) % n]

        u_in = unit(v - prev)
        u_out = unit(nxt - v)

        dot = float(np.dot(u_in, u_out))
        if abs(dot) > 1e-6:
            raise RuntimeError(
                "Ez a generator jelenleg 90 fokos ortogonalis sarkokat var."
            )

        cross = float(u_in[0] * u_out[1] - u_in[1] * u_out[0])
        turn_sign = 1.0 if cross > 0.0 else -1.0

        start = v - u_in * radius
        end = v + u_out * radius

        # 90 fokos fillet közepe.
        center = v - u_in * radius + u_out * radius

        tangent_starts.append(start)
        tangent_ends.append(end)
        centers.append(center)
        turn_signs.append(turn_sign)

    # A path az előző sarok végéről megy az aktuális sarok kezdetéig,
    # majd bejárja az aktuális negyedkört.
    for i in range(n):
        prev_i = (i - 1) % n

        append_line(
            rounded,
            tangent_ends[prev_i],
            tangent_starts[i],
        )

        append_arc(
            rounded,
            centers[i],
            tangent_starts[i],
            tangent_ends[i],
            turn_signs[i],
            radius,
        )

    points = np.asarray(rounded, dtype=np.float64)

    return points, segment_lengths, remaining_straights


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

    left = centerline + half * normal
    right = centerline - half * normal

    return left, right


def metric_to_pixel(points: np.ndarray):
    scale = (IMAGE_SIZE_PX - 1) / TRACK_SIZE_M

    px = np.rint(points[:, 0] * scale).astype(np.int32)
    py = np.rint((TRACK_SIZE_M - points[:, 1]) * scale).astype(np.int32)

    return np.column_stack((px, py))


def polyline_length(points: np.ndarray) -> float:
    nxt = np.roll(points, -1, axis=0)
    return float(np.sum(np.linalg.norm(nxt - points, axis=1)))


def check_bounds(*curves):
    for curve in curves:
        if (
            np.min(curve[:, 0]) < 0.0
            or np.max(curve[:, 0]) > TRACK_SIZE_M
            or np.min(curve[:, 1]) < 0.0
            or np.max(curve[:, 1]) > TRACK_SIZE_M
        ):
            raise RuntimeError(
                "A savhatar kikerul a 9.1 x 9.1 m-es texturabol."
            )


def generate(output_path: Path, debug_path: Path | None):
    center, segment_lengths, remaining_straights = build_rounded_centerline(
        CENTERLINE_VERTICES,
        CORNER_RADIUS_M,
    )

    left, right = offset_boundaries(center, LANE_WIDTH_M)

    check_bounds(center, left, right)

    # A belső sávhatár görbületi sugara kisebb, mint a középvonalé.
    # Ez csak a vizuális vonalra vonatkozik; a robot célja a középvonal.
    inner_boundary_radius = CORNER_RADIUS_M - LANE_WIDTH_M / 2.0

    safety_factor = CORNER_RADIUS_M / ROBOT_MIN_RADIUS_M
    min_straight = min(remaining_straights)

    if CORNER_RADIUS_M <= ROBOT_MIN_RADIUS_M:
        raise RuntimeError(
            "A kanyar sugara kisebb vagy egyenlo a robot elmeleti minimumanal."
        )

    image = np.zeros((IMAGE_SIZE_PX, IMAGE_SIZE_PX, 3), dtype=np.uint8)
    image[:, :] = BACKGROUND_BGR

    left_px = metric_to_pixel(left)
    right_px = metric_to_pixel(right)
    center_px = metric_to_pixel(center)

    line_width_px = max(
        2,
        int(round(LINE_WIDTH_M / TRACK_SIZE_M * IMAGE_SIZE_PX)),
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
            (0, 255, 255),
            max(2, line_width_px // 2),
            cv2.LINE_AA,
        )

        # A debug képen piros ponttal megjelöljük a geometriai csomópontokat.
        vertex_px = metric_to_pixel(CENTERLINE_VERTICES)

        for p in vertex_px:
            cv2.circle(
                debug,
                tuple(p),
                7,
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )

        debug_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(debug_path), debug)

    print("=== Wheeltec multi-corner racetrack ===")
    print(f"Texture:                   {IMAGE_SIZE_PX} x {IMAGE_SIZE_PX} px")
    print(f"Physical size:             {TRACK_SIZE_M:.2f} x {TRACK_SIZE_M:.2f} m")
    print(f"Lane width:                {LANE_WIDTH_M:.2f} m")
    print(f"White line width:          {LINE_WIDTH_M:.3f} m")
    print(f"Number of 90-deg turns:    {len(CENTERLINE_VERTICES)}")
    print()
    print(f"Robot theoretical R_min:   {ROBOT_MIN_RADIUS_M:.3f} m")
    print(f"Track corner radius:       {CORNER_RADIUS_M:.3f} m")
    print(f"Radius safety factor:      {safety_factor:.2f} x")
    print(f"Inner boundary radius:     {inner_boundary_radius:.3f} m")
    print()
    print(f"Shortest raw segment:      {min(segment_lengths):.3f} m")
    print(f"Shortest straight between")
    print(f"two rounded turns:         {min_straight:.3f} m")
    print(f"Approx. centerline length: {polyline_length(center):.3f} m")
    print()
    print(f"Output: {output_path}")

    if debug_path is not None:
        print(f"Debug:  {debug_path}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("course_multicorner.png"),
    )

    parser.add_argument(
        "--debug-output",
        type=Path,
        default=Path("course_multicorner_debug.png"),
    )

    args = parser.parse_args()

    generate(args.output, args.debug_output)


if __name__ == "__main__":
    main()
