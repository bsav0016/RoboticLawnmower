"""Checks on a field definition before any waypoints are generated.

A mistyped digit in a corner coordinate used to produce a plan with about 175,000 waypoints for a field roughly
87 km wide. These limits turn that kind of mistake into a readable message instead.
"""
import math

from lawnmower.geometry import to_meters

# The flat-earth projection in geometry.py is only claimed for small areas (under about 1 km). A lawn is far smaller.
MAX_EXTENT_M = 500.0
# Corners closer together than this are treated as the same point.
MIN_CORNER_GAP_M = 0.5
# Below this area the "field" is a line or a typo, not something to mow.
MIN_AREA_M2 = 1.0
# Closer passes than this make no sense against consumer GPS error, and a zero spacing would never finish.
MIN_SPACING_M = 0.1
# A plan with more waypoints than this is almost certainly a mistake (a 500 m square at 1 m spacing is about 1,000).
MAX_WAYPOINTS = 5000


class FieldError(ValueError):
    """The field definition is not usable. The message says what to fix."""


def _check_corner(name, lat, lon):
    if not (math.isfinite(lat) and math.isfinite(lon)):
        raise FieldError(f"{name} is not a number: ({lat}, {lon}).")
    if not -90 <= lat <= 90:
        raise FieldError(f"{name} has a latitude of {lat}, which is outside -90 to 90.")
    if not -180 <= lon <= 180:
        raise FieldError(f"{name} has a longitude of {lon}, which is outside -180 to 180.")


def _polygon_area(points):
    """Absolute area of a simple polygon given as ordered (x, y) points (shoelace formula)."""
    total = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        total += x1 * y2 - x2 * y1
    return abs(total) / 2


def check_corners(corners, spacing_m):
    """Raise FieldError if four (lat, lon) corners and a pass spacing cannot make a sensible plan.

    Returns the estimated number of waypoints, so a caller can report it."""
    if len(corners) != 4:
        raise FieldError(f"A field needs exactly 4 corners, got {len(corners)}.")
    if not (math.isfinite(spacing_m) and spacing_m >= MIN_SPACING_M):
        raise FieldError(f"Pass spacing is {spacing_m} m; it must be at least {MIN_SPACING_M} m.")

    names = "ABCD"
    for name, (lat, lon) in zip(names, corners):
        _check_corner(f"Corner {name}", lat, lon)

    ref_lat = sum(c[0] for c in corners) / 4
    ref_lon = sum(c[1] for c in corners) / 4
    pts = [to_meters(lat, lon, ref_lat, ref_lon) for lat, lon in corners]

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    width, height = max(xs) - min(xs), max(ys) - min(ys)
    if max(width, height) > MAX_EXTENT_M:
        raise FieldError(
            f"The corners span {max(width, height) / 1000:.2f} km, more than the {MAX_EXTENT_M:.0f} m limit. "
            "Check the corner coordinates for a mistyped digit."
        )

    for i in range(4):
        for j in range(i + 1, 4):
            if math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) < MIN_CORNER_GAP_M:
                raise FieldError(f"Corners {names[i]} and {names[j]} are the same point (less than {MIN_CORNER_GAP_M} m apart).")

    # Same corner order the planner uses: around the centroid (which also means the edges can never cross).
    cx, cy = sum(xs) / 4, sum(ys) / 4
    ordered = sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    area = _polygon_area(ordered)
    if area < MIN_AREA_M2:
        raise FieldError(f"The four corners enclose only {area:.2f} m2; they are almost in a line.")

    # Same count as generate_lawnmower: passes across the shorter dimension, two waypoints each.
    def edge(a, b):
        return math.hypot(ordered[b][0] - ordered[a][0], ordered[b][1] - ordered[a][1])

    short_side = edge(3, 0) if edge(0, 1) >= edge(1, 2) else edge(0, 1)
    waypoints = 2 * max(1, math.ceil(short_side / spacing_m) + 1)
    if waypoints > MAX_WAYPOINTS:
        raise FieldError(
            f"That plan would have {waypoints} waypoints, more than the {MAX_WAYPOINTS} limit. "
            "Check the corners and the pass spacing."
        )
    return waypoints
