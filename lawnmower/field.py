"""Checks on a field definition before any waypoints are generated.

A mistyped digit in a corner coordinate used to produce a plan with about 175,000 waypoints for a field roughly
87 km wide. These limits turn that kind of mistake into a readable message instead.
"""
import math
from dataclasses import dataclass

from lawnmower.geometry import to_meters
from lawnmower.polygon import counter_clockwise, is_simple, signed_area

# Meters between parallel passes when the field does not say.
DEFAULT_SPACING_M = 1.0

# The flat-earth projection in geometry.py is only claimed for small areas (under about 1 km). A lawn is far smaller.
MAX_EXTENT_M = 500.0
# Corners closer together than this are treated as the same point.
MIN_CORNER_GAP_M = 0.5
# Below this area the "field" is a line or a typo, not something to mow.
MIN_AREA_M2 = 1.0
# How far the boundary may be moved inwards.
MAX_INSET_M = 20.0
# Closer passes than this make no sense against consumer GPS error, and a zero spacing would never finish.
MIN_SPACING_M = 0.1
# A plan with more waypoints than this is almost certainly a mistake (a 500 m square at 1 m spacing is about 1,000).
MAX_WAYPOINTS = 5000


class FieldError(ValueError):
    """The field definition is not usable. The message says what to fix."""


@dataclass(frozen=True)
class Field:
    """What to mow: the boundary as (lat, lon) points, the spacing between passes, how far to keep the passes inside
    the boundary (0 puts the end of each pass on it), and optionally the compass heading of the passes."""
    boundary: tuple
    pass_spacing_m: float = DEFAULT_SPACING_M
    inset_m: float = 0.0
    heading_deg: float = None

    @property
    def corners(self):
        """The four corners, for the planner (which only handles four-sided fields for now)."""
        if len(self.boundary) != 4:
            raise FieldError(
                f"The boundary has {len(self.boundary)} points; only four-sided fields can be planned right now."
            )
        return list(self.boundary)


_SETTINGS = ("boundary", "pass_spacing_m", "inset_m", "heading_deg")


def parse_field(data):
    """Turn the contents of a field.json file (already decoded from JSON) into a Field.

    {"boundary": [[lat, lon], ...], "pass_spacing_m": 1.0, "inset_m": 0, "heading_deg": 90}
    Only "boundary" is required. heading_deg is the compass direction the passes run (0 = north, 90 = east); left
    out, they run along the longest side.

    Raises FieldError for anything that is not understood, so a misspelt setting is not silently ignored."""
    if not isinstance(data, dict):
        raise FieldError('The field file must be an object such as {"boundary": [[lat, lon], ...]}.')
    for key in data:
        if key == "exclusions":
            raise FieldError('"exclusions" (no-go zones) are not supported yet.')
        if key not in _SETTINGS:
            raise FieldError(f'Unknown setting "{key}". The settings are: {", ".join(_SETTINGS)}.')
    if "boundary" not in data:
        raise FieldError('The field file needs a "boundary": a list of [lat, lon] points.')

    raw = data["boundary"]
    if not isinstance(raw, list) or len(raw) < 3:
        raise FieldError('"boundary" must be a list of at least 3 [lat, lon] points.')
    boundary = []
    for index, point in enumerate(raw, start=1):
        ok = (isinstance(point, list) and len(point) == 2
              and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in point))
        if not ok:
            raise FieldError(f'Boundary point {index} is {point!r}; it must be two numbers, [lat, lon].')
        boundary.append((float(point[0]), float(point[1])))

    spacing = data.get("pass_spacing_m", DEFAULT_SPACING_M)
    if isinstance(spacing, bool) or not isinstance(spacing, (int, float)):
        raise FieldError(f'"pass_spacing_m" must be a number, not {spacing!r}.')

    inset = data.get("inset_m", 0.0)
    if isinstance(inset, bool) or not isinstance(inset, (int, float)) or not 0 <= inset <= MAX_INSET_M:
        raise FieldError(f'"inset_m" must be a number from 0 to {MAX_INSET_M:g}, not {inset!r}.')

    heading = data.get("heading_deg")
    if heading is not None:
        if isinstance(heading, bool) or not isinstance(heading, (int, float)) or not math.isfinite(heading):
            raise FieldError(f'"heading_deg" must be a number, not {heading!r}.')
        heading = float(heading) % 180  # a pass can be driven either way, so 90 and 270 are the same line
    return Field(boundary=tuple(boundary), pass_spacing_m=float(spacing), inset_m=float(inset), heading_deg=heading)


def _check_corner(name, lat, lon):
    if not (math.isfinite(lat) and math.isfinite(lon)):
        raise FieldError(f"{name} is not a number: ({lat}, {lon}).")
    if not -90 <= lat <= 90:
        raise FieldError(f"{name} has a latitude of {lat}, which is outside -90 to 90.")
    if not -180 <= lon <= 180:
        raise FieldError(f"{name} has a longitude of {lon}, which is outside -180 to 180.")


def _polygon_area(points):
    """Absolute area of a simple polygon given as ordered (x, y) points."""
    return abs(signed_area(points))


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


def check_boundary(boundary, spacing_m):
    """Raise FieldError if a boundary of any number of (lat, lon) points cannot make a sensible plan.

    The points are an ordered ring. (The four-sided planner sorts its corners itself; this one does not.)
    Returns the boundary as counter-clockwise (x, y) metre points around a local reference, plus that reference."""
    if len(boundary) < 3:
        raise FieldError(f"A field needs at least 3 boundary points, got {len(boundary)}.")
    if not (math.isfinite(spacing_m) and spacing_m >= MIN_SPACING_M):
        raise FieldError(f"Pass spacing is {spacing_m} m; it must be at least {MIN_SPACING_M} m.")
    for index, (lat, lon) in enumerate(boundary, start=1):
        _check_corner(f"Boundary point {index}", lat, lon)

    ref_lat = sum(p[0] for p in boundary) / len(boundary)
    ref_lon = sum(p[1] for p in boundary) / len(boundary)
    pts = [to_meters(lat, lon, ref_lat, ref_lon) for lat, lon in boundary]

    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    extent = max(max(xs) - min(xs), max(ys) - min(ys))
    if extent > MAX_EXTENT_M:
        raise FieldError(
            f"The boundary spans {extent / 1000:.2f} km, more than the {MAX_EXTENT_M:.0f} m limit. "
            "Check the coordinates for a mistyped digit."
        )
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        if math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) < MIN_CORNER_GAP_M:
            raise FieldError(
                f"Boundary points {i + 1} and {j + 1} are the same point (less than {MIN_CORNER_GAP_M} m apart)."
            )
    area = _polygon_area(pts)
    if area < MIN_AREA_M2:
        raise FieldError(f"The boundary encloses only {area:.2f} m2; the points are almost in a line.")
    if not is_simple(pts):
        raise FieldError("The boundary crosses itself. List the points in order around the field.")
    return counter_clockwise(pts), (ref_lat, ref_lon)
