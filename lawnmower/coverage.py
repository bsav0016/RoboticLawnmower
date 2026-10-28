"""Coverage planning for a polygon field: parallel passes clipped to the boundary, in boustrophedon order.

Pure maths, no hardware. The field is projected into local metres, optionally moved inwards (so the rover is kept
off the edge, where GPS error could take it outside), and cut by parallel lines `spacing_m` apart. Each line
becomes one pass; passes are driven in alternating directions, with a short connecting waypoint wherever the next
pass starts somewhere other than where the last one ended, so the rover never cuts across the outside of a bend.

Limit for now: every line must cross the field once (the field is "monotone" in the pass direction). L-shapes,
triangles, trapezoids and pentagons qualify; a field that needs a pass split into pieces (a no-go zone in the
middle, a deep notch) does not, and raises FieldError.
"""
import math

from lawnmower.field import MAX_WAYPOINTS, FieldError, check_boundary
from lawnmower.geometry import to_gps, to_meters
from lawnmower.polygon import PolygonError, inset

# A pass shorter than this is a sliver at a sharp tip, not worth driving.
MIN_PASS_M = 0.2
# Rows are nudged this far inside the field so a row exactly on a vertex or edge still crosses it cleanly.
_EDGE_NUDGE_M = 1e-6
# Two points closer than this along a pass are the same place.
_SAME_PLACE_M = 0.01


class _NotMonotone(Exception):
    """A pass line crossed the field more than once at the heading being tried."""


def _order_quad(boundary):
    """For four points the order is not significant (as with the four-corner planner): sort around the centre."""
    ref_lat = sum(p[0] for p in boundary) / 4
    ref_lon = sum(p[1] for p in boundary) / 4
    def angle(p):
        x, y = to_meters(p[0], p[1], ref_lat, ref_lon)
        return math.atan2(y, x)
    return sorted(boundary, key=angle)


def _heading_candidates(ring):
    """Compass headings of the ring's edges, longest first, without repeats (a pass can be driven either way)."""
    edges = []
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        length = math.hypot(x2 - x1, y2 - y1)
        edges.append((length, math.degrees(math.atan2(x2 - x1, y2 - y1)) % 180))
    headings = []
    for _, heading in sorted(edges, reverse=True):
        if all(min(abs(heading - h), 180 - abs(heading - h)) > 1.0 for h in headings):
            headings.append(heading)
    return headings


def _rows(ring, heading_deg, spacing_m):
    """Passes for one heading, in a frame where the passes run along +a and successive rows step along +b.

    Returns (rows, u): rows is a list of (b, a_start, a_end) with a_start < a_end, u the unit vector of the heading.
    Raises _NotMonotone when a row crosses the field more than once."""
    u = (math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg)))
    v = (-u[1], u[0])
    frame = [(x * u[0] + y * u[1], x * v[0] + y * v[1]) for x, y in ring]
    b_min = min(p[1] for p in frame)
    b_max = max(p[1] for p in frame)
    height = b_max - b_min
    if height <= 2 * _EDGE_NUDGE_M:
        raise FieldError("The field is too thin to plan passes across.")

    count = max(1, math.ceil(height / spacing_m)) + 1  # same pass count as the four-corner planner
    rows = []
    last_b = None
    for i in range(count):
        b = b_min + min(i * spacing_m, height)
        b = min(max(b, b_min + _EDGE_NUDGE_M), b_max - _EDGE_NUDGE_M)
        if last_b is not None and b - last_b < _SAME_PLACE_M:
            continue  # a height that is an exact multiple of the spacing would otherwise repeat the last row
        last_b = b
        crossings = []
        for (a1, b1), (a2, b2) in zip(frame, frame[1:] + frame[:1]):
            if b1 != b2 and min(b1, b2) <= b < max(b1, b2):
                crossings.append(a1 + (b - b1) * (a2 - a1) / (b2 - b1))
        crossings.sort()
        if len(crossings) > 2:
            raise _NotMonotone()
        if len(crossings) == 2 and crossings[1] - crossings[0] >= MIN_PASS_M:
            rows.append((b, crossings[0], crossings[1]))
    return rows, u


def _drive(rows):
    """Waypoints (a, b) for the rows in order: alternate directions, with a connecting point where needed."""
    waypoints = []
    position = None  # where the rover is: the end of the last pass
    for b, a_low, a_high in rows:
        if position is None:
            start, end = a_low, a_high
        else:
            pa, pb = position
            start, end = (a_low, a_high) if abs(pa - a_low) <= abs(pa - a_high) else (a_high, a_low)
            if abs(pa - start) > _SAME_PLACE_M:
                if a_low - _SAME_PLACE_M <= pa <= a_high + _SAME_PLACE_M:
                    waypoints.append((pa, b))  # step across at the end of the last pass, then along this one
                else:
                    waypoints.append((start, pb))  # go back along the last pass to under the start, then step
        waypoints.append((start, b))
        waypoints.append((end, b))
        position = (end, b)
    return waypoints


def plan_coverage(boundary, spacing_m, inset_m=0.0, heading_deg=None):
    """Waypoints (lat, lon) covering a polygon field.

    boundary: (lat, lon) points in order around the field (for exactly four points, in any order).
    inset_m: keep the passes this far inside the boundary (0 puts pass ends on it).
    heading_deg: compass direction of the passes (0 = north, 90 = east); None picks one automatically, starting with
    the direction of the longest side.

    Raises FieldError when the field cannot be planned, with a message that says what to change."""
    boundary = [tuple(p) for p in boundary]
    if len(boundary) == 4:
        boundary = _order_quad(boundary)
    ring, (ref_lat, ref_lon) = check_boundary(boundary, spacing_m)

    if inset_m > 0:
        try:
            ring = inset(ring, inset_m)
        except PolygonError as error:
            raise FieldError(f"Cannot keep {inset_m:g} m inside the boundary: {error}.") from None

    headings = [heading_deg % 180] if heading_deg is not None else _heading_candidates(ring)
    rows = u = None
    for heading in headings:
        try:
            rows, u = _rows(ring, heading, spacing_m)
            break
        except _NotMonotone:
            continue
    if rows is None:
        raise FieldError(
            "Passes at "
            + (f"a heading of {heading_deg:g} degrees" if heading_deg is not None else "any of the boundary's directions")
            + " cross this field in more than one piece, which the planner cannot connect yet. "
            + ("Try another heading_deg, or " if heading_deg is not None else "")
            + "simplify the boundary."
        )
    if not rows:
        raise FieldError("No passes fit inside the field. Check the boundary, the spacing and the inset.")

    v = (-u[1], u[0])
    waypoints = []
    for a, b in _drive(rows):
        x, y = a * u[0] + b * v[0], a * u[1] + b * v[1]
        waypoints.append(to_gps(x, y, ref_lat, ref_lon))
    if len(waypoints) > MAX_WAYPOINTS:
        raise FieldError(
            f"That plan would have {len(waypoints)} waypoints, more than the {MAX_WAYPOINTS} limit. "
            "Check the boundary and the pass spacing."
        )
    return waypoints
