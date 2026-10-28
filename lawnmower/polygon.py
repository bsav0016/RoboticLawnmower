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


def _properly_cross(p1, p2, p3, p4):
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return side(p1, p2, p3) * side(p1, p2, p4) < 0 and side(p3, p4, p1) * side(p3, p4, p2) < 0


def is_simple(points):
    """True if no edge crosses another and no vertex touches an edge it is not part of."""
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue  # neighbouring edges share a vertex
            c, d = points[j], points[(j + 1) % n]
            if _properly_cross(a, b, c, d):
                return False
        for k in range(n):
            if k in (i, (i + 1) % n):
                continue
            if _distance_to_segment(points[k], a, b) < TOUCH_TOLERANCE_M:
                return False
    return True


def inset(points, distance):
    """The polygon moved inwards by `distance` metres, each corner pushed along the bisector (mitred).

    Exact for convex fields and for concave ones whose features are wider than twice the distance. Raises
    PolygonError when the result is not a smaller, valid polygon inside the original (distance too large for the
    shape, or a corner too sharp)."""
    ring = counter_clockwise(points)
    n = len(ring)
    normals = []
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        length = math.hypot(x2 - x1, y2 - y1)
        normals.append((-(y2 - y1) / length, (x2 - x1) / length))  # left of the edge = inside for a CCW ring

    moved = []
    for i in range(n):
        n1, n2 = normals[i - 1], normals[i]
        denominator = 1 + n1[0] * n2[0] + n1[1] * n2[1]
        if denominator < MIN_MITER_DENOMINATOR:
            raise PolygonError("a corner is too sharp to move the boundary inwards")
        k = distance / denominator
        moved.append((ring[i][0] + k * (n1[0] + n2[0]), ring[i][1] + k * (n1[1] + n2[1])))

    original_area = signed_area(ring)
    new_area = signed_area(moved)
    ok = (0 < new_area < original_area and is_simple(moved)
          and all(point_in_polygon(p, ring) for p in moved))
    if not ok:
        raise PolygonError(f"moving the boundary inwards by {distance:g} m does not leave a usable field")
    return moved
