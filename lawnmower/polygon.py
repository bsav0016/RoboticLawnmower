"""Small plain-polygon helpers in local metres: area, point-in-polygon, simplicity and inward offset.

A polygon is a list of (x, y) points in order around the ring, not repeating the first point at the end.
"""
import math

# An inward offset at a corner sharper than about 10 degrees would shoot far out of the field.
MIN_MITER_DENOMINATOR = 0.015
# A vertex this close to a non-neighbouring edge counts as touching it.
TOUCH_TOLERANCE_M = 0.01


class PolygonError(ValueError):
    """The polygon cannot be used for the requested operation; the message says why."""


def signed_area(points):
    """Positive for counter-clockwise rings, negative for clockwise (shoelace formula)."""
    total = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        total += x1 * y2 - x2 * y1
    return total / 2


def counter_clockwise(points):
    return list(points) if signed_area(points) > 0 else list(reversed(points))


def point_in_polygon(point, points):
    """Ray casting; points exactly on the boundary may count either way."""
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def distance_to_boundary(point, points):
    """Shortest distance from a point to the polygon's edges."""
    return min(_distance_to_segment(point, a, b) for a, b in zip(points, points[1:] + points[:1]))


def _distance_to_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def properly_cross(p1, p2, p3, p4):
    """True if segment p1-p2 crosses segment p3-p4 through both interiors.

    Ends that merely touch the other segment (or sit on it to within a micrometre of rounding error) do not count."""
    def side(a, c, d):
        # Signed distance from point d to the line through a and c.
        length = math.hypot(c[0] - a[0], c[1] - a[1])
        return ((c[0] - a[0]) * (d[1] - a[1]) - (c[1] - a[1]) * (d[0] - a[0])) / length if length else 0.0

    d1, d2 = side(p1, p2, p3), side(p1, p2, p4)
    d3, d4 = side(p3, p4, p1), side(p3, p4, p2)
    return d1 * d2 < 0 and d3 * d4 < 0 and min(abs(d1), abs(d2), abs(d3), abs(d4)) > 1e-6


def edges_cross(points):
    """True if two edges of the ring cross each other (a figure-eight or bow-tie)."""
    n = len(points)
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue  # these two edges share a vertex
            if properly_cross(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n]):
                return True
    return False


def is_simple(points):
    """True if no edge crosses another and no vertex touches an edge it is not part of."""
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue  # neighbouring edges share a vertex
            c, d = points[j], points[(j + 1) % n]
            if properly_cross(a, b, c, d):
                return False
        for k in range(n):
            if k in (i, (i + 1) % n):
                continue
            if _distance_to_segment(points[k], a, b) < TOUCH_TOLERANCE_M:
                return False
    return True


def _offset(points, distance, inwards):
    """The ring moved by `distance` metres, each corner pushed along the bisector (mitred)."""
    ring = counter_clockwise(points)
    n = len(ring)
    direction = 1 if inwards else -1
    normals = []
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        length = math.hypot(x2 - x1, y2 - y1)
        # Left of the edge is inside for a counter-clockwise ring.
        normals.append((-direction * (y2 - y1) / length, direction * (x2 - x1) / length))

    moved = []
    for i in range(n):
        n1, n2 = normals[i - 1], normals[i]
        denominator = 1 + n1[0] * n2[0] + n1[1] * n2[1]
        if denominator < MIN_MITER_DENOMINATOR:
            raise PolygonError("a corner is too sharp to move the boundary " + ("inwards" if inwards else "outwards"))
        k = distance / denominator
        moved.append((ring[i][0] + k * (n1[0] + n2[0]), ring[i][1] + k * (n1[1] + n2[1])))

    original_area = signed_area(ring)
    new_area = signed_area(moved)
    if inwards:
        ok = (0 < new_area < original_area and is_simple(moved)
              and all(point_in_polygon(p, ring) for p in moved))
    else:
        ok = (new_area > original_area and is_simple(moved)
              and all(point_in_polygon(p, moved) for p in ring))
    if not ok:
        raise PolygonError(
            f"moving the boundary {'inwards' if inwards else 'outwards'} by {distance:g} m "
            + ("does not leave a usable field" if inwards else "does not give a usable shape")
        )
    return moved


def inset(points, distance):
    """The polygon moved inwards by `distance` metres.

    Exact for convex fields and for concave ones whose features are wider than twice the distance. Raises
    PolygonError when the result is not a smaller, valid polygon inside the original (distance too large for the
    shape, or a corner too sharp)."""
    return _offset(points, distance, inwards=True)


def outset(points, distance):
    """The polygon grown outwards by `distance` metres (a keep-out margin around a no-go zone).

    Raises PolygonError when the result is not a larger, valid polygon around the original."""
    return _offset(points, distance, inwards=False)


def rings_clear(a, b):
    """True if the edges of two rings neither cross nor touch (one may still lie wholly inside the other)."""
    for p1, p2 in zip(a, a[1:] + a[:1]):
        for p3, p4 in zip(b, b[1:] + b[:1]):
            if properly_cross(p1, p2, p3, p4):
                return False
    return (all(_distance_to_segment(p, c, d) >= TOUCH_TOLERANCE_M for p in a for c, d in zip(b, b[1:] + b[:1]))
            and all(_distance_to_segment(p, c, d) >= TOUCH_TOLERANCE_M for p in b for c, d in zip(a, a[1:] + a[:1])))
