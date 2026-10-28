"""Checks on a field definition before any waypoints are generated.

A mistyped digit in a corner coordinate used to produce a plan with about 175,000 waypoints for a field roughly
87 km wide. These limits turn that kind of mistake into a readable message instead.
"""
import math
from dataclasses import dataclass

from lawnmower.geometry import to_meters
from lawnmower.polygon import counter_clockwise, edges_cross, is_simple, signed_area

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
# More no-go zones than this is almost certainly a mistake.
MAX_EXCLUSIONS = 50
# A cutting deck wider than this is almost certainly a typo.
MAX_CUT_WIDTH_M = 10.0
# Closer passes than this make no sense against consumer GPS error, and a zero spacing would never finish.
MIN_SPACING_M = 0.1
# A plan with more waypoints than this is almost certainly a mistake (a 500 m square at 1 m spacing is about 1,000).
MAX_WAYPOINTS = 5000


class FieldError(ValueError):
    """The field definition is not usable. The message says what to fix."""


@dataclass(frozen=True)
class Field:
    """What to mow: the boundary as (lat, lon) points, the spacing between passes, how far to keep the passes inside
    the boundary (0 puts the end of each pass on it, and the same distance is kept from no-go zones), optionally the
    compass heading of the passes, the no-go zones (each a ring of (lat, lon) points inside the boundary), and how far
    beyond the end of a pass the rover swings out to turn round (0 = turn straight onto the next pass).

    pass_spacing_m is the distance between passes. It is either given directly, or worked out from the cutting width
    as cut_width_m - overlap_m (cut_width_m and overlap_m are kept here only to show in the preview)."""
    boundary: tuple
    pass_spacing_m: float = DEFAULT_SPACING_M
    inset_m: float = 0.0
    heading_deg: float = None
    exclusions: tuple = ()
    turn_depth_m: float = 0.0
    cut_width_m: float = None
    overlap_m: float = 0.0

    @property
    def corners(self):
        """The four corners, for the planner (which only handles four-sided fields for now)."""
        if len(self.boundary) != 4:
            raise FieldError(
                f"The boundary has {len(self.boundary)} points; only four-sided fields can be planned right now."
            )
        return list(self.boundary)


_SETTINGS = ("boundary", "pass_spacing_m", "inset_m", "heading_deg", "exclusions", "turn_depth_m",
             "cut_width_m", "overlap_m")


def _parse_ring(raw, what, name):
    """A list of at least 3 [lat, lon] pairs as a tuple of (lat, lon) tuples."""
    if not isinstance(raw, list) or len(raw) < 3:
        raise FieldError(f"{what} must be a list of at least 3 [lat, lon] points.")
    points = []
    for index, point in enumerate(raw, start=1):
        ok = (isinstance(point, list) and len(point) == 2
              and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in point))
        if not ok:
            raise FieldError(f"{name} point {index} is {point!r}; it must be two numbers, [lat, lon].")
        points.append((float(point[0]), float(point[1])))
    return tuple(points)


def parse_field(data):
    """Turn the contents of a field.json file (already decoded from JSON) into a Field.

    {"boundary": [[lat, lon], ...], "pass_spacing_m": 1.0, "inset_m": 0, "heading_deg": 90,
     "exclusions": [[[lat, lon], ...], ...], "turn_depth_m": 0, "cut_width_m": 0.8, "overlap_m": 0.1}
    Only "boundary" is required. heading_deg is the compass direction the passes run (0 = north, 90 = east); left
    out, they run along the longest side. Each exclusion is a no-go zone: a ring of at least 3 points inside the
    boundary. cut_width_m and overlap_m replace pass_spacing_m (spacing = cut width - overlap); turn_depth_m is how
    far past the end of a pass the rover swings out to turn, and cannot be more than inset_m.

    Raises FieldError for anything that is not understood, so a misspelt setting is not silently ignored."""
    if not isinstance(data, dict):
        raise FieldError('The field file must be an object such as {"boundary": [[lat, lon], ...]}.')
    for key in data:
        if key not in _SETTINGS:
            raise FieldError(f'Unknown setting "{key}". The settings are: {", ".join(_SETTINGS)}.')
    if "boundary" not in data:
        raise FieldError('The field file needs a "boundary": a list of [lat, lon] points.')

    boundary = _parse_ring(data["boundary"], '"boundary"', "Boundary")
    raw_zones = data.get("exclusions", [])
    if not isinstance(raw_zones, list) or len(raw_zones) > MAX_EXCLUSIONS:
        raise FieldError(f'"exclusions" must be a list of at most {MAX_EXCLUSIONS} no-go zones.')
    exclusions = tuple(_parse_ring(zone, f'Exclusion {i}', f"Exclusion {i}") for i, zone in enumerate(raw_zones, 1))

    cut_width, overlap = data.get("cut_width_m"), data.get("overlap_m")
    if cut_width is not None and "pass_spacing_m" in data:
        raise FieldError('Set either "pass_spacing_m" or "cut_width_m" (the spacing is the cut width minus the overlap).')
    if overlap is not None and cut_width is None:
        raise FieldError('"overlap_m" needs "cut_width_m".')
    if cut_width is not None:
        if isinstance(cut_width, bool) or not isinstance(cut_width, (int, float)) or not 0 < cut_width <= MAX_CUT_WIDTH_M:
            raise FieldError(f'"cut_width_m" must be a number above 0 and at most {MAX_CUT_WIDTH_M:g}, not {cut_width!r}.')
        overlap = 0.0 if overlap is None else overlap
        if isinstance(overlap, bool) or not isinstance(overlap, (int, float)) or overlap < 0:
            raise FieldError(f'"overlap_m" must be a number of 0 or more, not {overlap!r}.')
        spacing = cut_width - overlap
        if spacing < MIN_SPACING_M:
            raise FieldError(
                f'An overlap of {overlap:g} m on a {cut_width:g} m cut leaves a pass spacing of {spacing:g} m; '
                f'it must be at least {MIN_SPACING_M} m.'
            )
    else:
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

    turn_depth = data.get("turn_depth_m", 0.0)
    if isinstance(turn_depth, bool) or not isinstance(turn_depth, (int, float)) or not 0 <= turn_depth <= MAX_INSET_M:
        raise FieldError(f'"turn_depth_m" must be a number from 0 to {MAX_INSET_M:g}, not {turn_depth!r}.')
    if turn_depth > inset:
        raise FieldError(
            f'"turn_depth_m" ({turn_depth:g}) cannot be more than "inset_m" ({inset:g}): the turns happen in the '
            'margin between the passes and the boundary.'
        )
    return Field(boundary=boundary, pass_spacing_m=float(spacing), inset_m=float(inset), heading_deg=heading,
                 exclusions=exclusions, turn_depth_m=float(turn_depth),
                 cut_width_m=None if cut_width is None else float(cut_width),
                 overlap_m=0.0 if overlap is None else float(overlap))


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
    if edges_cross(pts):
        raise FieldError("The boundary crosses itself. List the points in order around the field.")
    area = _polygon_area(pts)
    if area < MIN_AREA_M2:
        raise FieldError(f"The boundary encloses only {area:.2f} m2; the points are almost in a line.")
    if not is_simple(pts):
        raise FieldError("The boundary crosses itself. List the points in order around the field.")
    return counter_clockwise(pts), (ref_lat, ref_lon)


def check_exclusion(index, points, ref):
    """Check one no-go zone and return it as a counter-clockwise ring of (x, y) metres around `ref` (lat, lon).

    Raises FieldError, naming the zone, if its points are out of range, it is a line or a speck, it crosses itself,
    or it is much bigger than the field limit allows."""
    name = f"Exclusion {index}"
    if len(points) < 3:
        raise FieldError(f"{name} needs at least 3 points, got {len(points)}.")
    for k, (lat, lon) in enumerate(points, start=1):
        _check_corner(f"{name} point {k}", lat, lon)
    ring = [to_meters(lat, lon, ref[0], ref[1]) for lat, lon in points]
    xs, ys = [p[0] for p in ring], [p[1] for p in ring]
    if max(max(xs) - min(xs), max(ys) - min(ys)) > MAX_EXTENT_M:
        raise FieldError(f"{name} spans more than {MAX_EXTENT_M:.0f} m. Check its coordinates for a mistyped digit.")
    n = len(ring)
    for i in range(n):
        j = (i + 1) % n
        if math.hypot(ring[i][0] - ring[j][0], ring[i][1] - ring[j][1]) < MIN_CORNER_GAP_M:
            raise FieldError(f"{name} points {i + 1} and {j + 1} are the same point (less than {MIN_CORNER_GAP_M} m apart).")
    if edges_cross(ring):
        raise FieldError(f"{name} crosses itself. List its points in order around the zone.")
    area = _polygon_area(ring)
    if area < MIN_AREA_M2:
        raise FieldError(f"{name} encloses only {area:.2f} m2; its points are almost in a line.")
    if not is_simple(ring):
        raise FieldError(f"{name} crosses itself. List its points in order around the zone.")
    return counter_clockwise(ring)
