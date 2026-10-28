"""Coverage planning for a polygon field with no-go zones: parallel passes clipped to the free space.

Pure maths, no hardware. The field is projected into local metres, optionally moved inwards (so the rover is kept
off the edge, where GPS error could take it outside), the no-go zones are grown by the same margin, and the free
space is cut by parallel lines `spacing_m` apart. Each line becomes one or more passes (a line is split wherever it
meets a no-go zone or a notch in the boundary).

Passes in neighbouring rows that continue each other form a "cell", mown in boustrophedon order with a connecting
waypoint at each bend so the rover never cuts across the outside of the field. The cells are then visited
nearest-first, and between cells the rover follows the shortest route through the free space (around no-go zones and
notches), never a straight line across them.
"""
import heapq
import math
from collections import namedtuple

from lawnmower.field import MAX_WAYPOINTS, FieldError, check_boundary, check_exclusion
from lawnmower.geometry import to_gps, to_meters
from lawnmower.polygon import (
    PolygonError, distance_to_boundary, inset, outset, point_in_polygon, properly_cross, rings_clear,
)

# A pass shorter than this is a sliver at a sharp tip, not worth driving.
MIN_PASS_M = 0.2
# Passes in neighbouring rows must overlap by at least this to count as continuing each other.
_MIN_OVERLAP_M = 0.2
# Rows are nudged this far inside the field so a row exactly on a vertex or edge still crosses it cleanly.
_EDGE_NUDGE_M = 1e-6
# Two points closer than this along a pass are the same place.
_SAME_PLACE_M = 0.01
# A point this close to a boundary counts as on it, for "is this route inside the field".
_ON_EDGE_M = 0.02
# How many field headings are tried when none is given.
_MAX_HEADINGS = 6

Prepared = namedtuple("Prepared", "outline driven exclusions keepouts ref")


def _order_quad(boundary):
    """For four points the order is not significant (as with the four-corner planner): sort around the centre."""
    ref_lat = sum(p[0] for p in boundary) / 4
    ref_lon = sum(p[1] for p in boundary) / 4

    def angle(p):
        x, y = to_meters(p[0], p[1], ref_lat, ref_lon)
        return math.atan2(y, x)
    return sorted(boundary, key=angle)


def prepare_field(boundary, spacing_m, inset_m=0.0, exclusions=()):
    """Check a field and project it to local metres.

    Returns a Prepared tuple: `outline` (the field boundary), `driven` (the outline moved inwards by inset_m, the
    same ring when inset_m is 0), `exclusions` (the no-go zones as given), `keepouts` (each zone grown by inset_m) and
    `ref`, the (lat, lon) origin of the frame. Every ring is counter-clockwise (x, y) metres. Raises FieldError."""
    boundary = [tuple(p) for p in boundary]
    if len(boundary) == 4:
        boundary = _order_quad(boundary)
    outline, ref = check_boundary(boundary, spacing_m)
    driven = outline
    if inset_m > 0:
        try:
            driven = inset(outline, inset_m)
        except PolygonError as error:
            raise FieldError(f"Cannot keep {inset_m:g} m inside the boundary: {error}.") from None

    zones, keepouts = [], []
    margin_note = f" once {inset_m:g} m of margin is added" if inset_m > 0 else ""
    for index, points in enumerate(exclusions, start=1):
        zone = check_exclusion(index, [tuple(p) for p in points], ref)
        keepout = zone
        if inset_m > 0:
            try:
                keepout = outset(zone, inset_m)
            except PolygonError as error:
                raise FieldError(f"Cannot keep {inset_m:g} m away from exclusion {index}: {error}.") from None
        if not (all(point_in_polygon(p, driven) for p in keepout) and rings_clear(driven, keepout)):
            raise FieldError(
                f"Exclusion {index} reaches the edge of the field{margin_note}. Move it inside the boundary"
                + (", or reduce inset_m." if inset_m > 0 else ".")
            )
        for other, earlier in enumerate(keepouts, start=1):
            if (not rings_clear(keepout, earlier) or point_in_polygon(keepout[0], earlier)
                    or point_in_polygon(earlier[0], keepout)):
                raise FieldError(
                    f"Exclusions {other} and {index} touch or overlap{margin_note}. "
                    "Merge them into one zone or move them apart."
                )
        zones.append(zone)
        keepouts.append(keepout)
    return Prepared(outline, driven, zones, keepouts, ref)


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
    return headings[:_MAX_HEADINGS]


def _to_frame(points, u):
    """Points in a frame where the passes run along +a (the unit vector u) and rows step along +b."""
    v = (-u[1], u[0])
    return [(x * u[0] + y * u[1], x * v[0] + y * v[1]) for x, y in points]


# ---------------------------------------------------------------------------------------------------------------------
# Free space: is a straight route inside the field and clear of the no-go zones, and the shortest route between points
# ---------------------------------------------------------------------------------------------------------------------

class _Space:
    """The free space (the driven ring minus the keep-out rings), for route checks and shortest routes."""

    def __init__(self, driven, keepouts):
        self.driven = driven
        self.keepouts = list(keepouts)
        self.rings = [driven] + self.keepouts
        self._boxes = [(min(p[0] for p in r), min(p[1] for p in r), max(p[0] for p in r), max(p[1] for p in r))
                       for r in self.rings]
        # Corners a shortest route can bend at: the sharp inward corners of the field and every corner of a no-go zone.
        self._vertices = _reflex_corners(driven) + [p for ring in self.keepouts for p in ring]
        self._links = None  # vertex-to-vertex visibility, built on first use

    def point_free(self, p, margin=_ON_EDGE_M):
        if not (point_in_polygon(p, self.driven) or distance_to_boundary(p, self.driven) < margin):
            return False
        for keepout, box in zip(self.keepouts, self._boxes[1:]):
            if box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3]:
                if point_in_polygon(p, keepout) and distance_to_boundary(p, keepout) >= margin:
                    return False
        return True

    def route_free(self, p, q):
        """True if the straight line from p to q stays inside the field and out of every no-go zone."""
        length = math.hypot(q[0] - p[0], q[1] - p[1])
        if length < 1e-9:
            return True
        low_x, high_x = min(p[0], q[0]), max(p[0], q[0])
        low_y, high_y = min(p[1], q[1]), max(p[1], q[1])
        for ring, box in zip(self.rings, self._boxes):
            if box[0] > high_x or box[2] < low_x or box[1] > high_y or box[3] < low_y:
                continue  # this ring is nowhere near the route
            for c, d in zip(ring, ring[1:] + ring[:1]):
                if properly_cross(p, q, c, d):
                    return False
        samples = min(12, max(4, int(length / 1.0)))
        for k in range(1, samples):
            t = k / samples
            if not self.point_free((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))):
                return False
        return True

    def _build_links(self):
        vertices = self._vertices
        links = [[] for _ in vertices]
        for i in range(len(vertices)):
            for j in range(i + 1, len(vertices)):
                if self.route_free(vertices[i], vertices[j]):
                    d = math.hypot(vertices[i][0] - vertices[j][0], vertices[i][1] - vertices[j][1])
                    links[i].append((j, d))
                    links[j].append((i, d))
        self._links = links

    def routes_from(self, start):
        """Shortest distances from `start` to every corner, for route_to: (start, distances, parents)."""
        if self._links is None:
            self._build_links()
        vertices = self._vertices
        distance = [math.inf] * len(vertices)
        parent = [None] * len(vertices)
        queue = []
        for i, v in enumerate(vertices):
            d = math.hypot(v[0] - start[0], v[1] - start[1])
            if self.route_free(start, v):
                distance[i] = d
                heapq.heappush(queue, (d, i))
        while queue:
            d, node = heapq.heappop(queue)
            if d > distance[node]:
                continue
            for nxt, step in self._links[node]:
                if d + step < distance[nxt]:
                    distance[nxt] = d + step
                    parent[nxt] = node
                    heapq.heappush(queue, (d + step, nxt))
        return start, distance, parent

    def route_to(self, tree, goal):
        """(length, points) of the shortest route from the start of the tree to goal, or None.

        Points include both ends and any corners passed on the way."""
        start, distance, parent = tree
        if self.route_free(start, goal):
            return math.hypot(goal[0] - start[0], goal[1] - start[1]), [start, goal]
        vertices = self._vertices
        # Try corners in order of the best the route through them could be; the first one that can see the goal wins.
        order = sorted((distance[i] + math.hypot(v[0] - goal[0], v[1] - goal[1]), i)
                       for i, v in enumerate(vertices) if distance[i] < math.inf)
        for cost, i in order:
            if self.route_free(vertices[i], goal):
                path, at = [goal], i
                while at is not None:
                    path.append(vertices[at])
                    at = parent[at]
                path.append(start)
                path.reverse()
                return cost, path
        return None


def _reflex_corners(ring):
    """The corners of a counter-clockwise ring that point into the ring (where a route can bend around)."""
    n = len(ring)
    corners = []
    for i in range(n):
        (x0, y0), (x1, y1), (x2, y2) = ring[i - 1], ring[i], ring[(i + 1) % n]
        if (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) < 0:
            corners.append(ring[i])
    return corners


# ---------------------------------------------------------------------------------------------------------------------
# Rows, cells and driving order
# ---------------------------------------------------------------------------------------------------------------------

def _rows(rings, spacing_m):
    """Passes cut from the free space: a list of (b, [(a_start, a_end), ...]) for rows that have any.

    `rings` are in the frame of _to_frame: the driven ring first, then the keep-out rings. A row is split wherever it
    crosses a ring (inside and outside alternate, so a no-go zone cuts a hole in it)."""
    b_min = min(p[1] for p in rings[0])
    b_max = max(p[1] for p in rings[0])
    height = b_max - b_min
    if height <= 2 * _EDGE_NUDGE_M:
        raise FieldError("The field is too thin to plan passes across.")
    edges = [(a1, b1, a2, b2) for ring in rings for (a1, b1), (a2, b2) in zip(ring, ring[1:] + ring[:1]) if b1 != b2]

    count = max(1, math.ceil(height / spacing_m)) + 1  # same pass count as the four-corner planner
    rows = []
    last_b = None
    for i in range(count):
        b = b_min + min(i * spacing_m, height)
        b = min(max(b, b_min + _EDGE_NUDGE_M), b_max - _EDGE_NUDGE_M)
        if last_b is not None and b - last_b < _SAME_PLACE_M:
            continue  # a height that is an exact multiple of the spacing would otherwise repeat the last row
        last_b = b
        crossings = sorted(a1 + (b - b1) * (a2 - a1) / (b2 - b1)
                           for a1, b1, a2, b2 in edges if min(b1, b2) <= b < max(b1, b2))
        segments = [(crossings[k], crossings[k + 1]) for k in range(0, len(crossings) - 1, 2)
                    if crossings[k + 1] - crossings[k] >= MIN_PASS_M]
        if segments:
            rows.append((b, segments))
    return rows


def _overlap(s, t):
    return min(s[1], t[1]) - max(s[0], t[0]) > _MIN_OVERLAP_M


def _chains(rows):
    """Group passes into chains: each pass continues into exactly one pass in the next row, and no other pass does.

    A chain is a list of (b, a_low, a_high) in row order."""
    successors = {}
    predecessors = {}
    for i in range(len(rows) - 1):
        for j, s in enumerate(rows[i][1]):
            successors[(i, j)] = [k for k, t in enumerate(rows[i + 1][1]) if _overlap(s, t)]
            for k in successors[(i, j)]:
                predecessors.setdefault((i + 1, k), []).append((i, j))

    def continues(node):
        before = predecessors.get(node, [])
        return len(before) == 1 and successors[before[0]] == [node[1]]

    chains = []
    for i, (_, segments) in enumerate(rows):
        for j in range(len(segments)):
            if i > 0 and continues((i, j)):
                continue  # this pass is the middle or end of a chain that started earlier
            node = (i, j)
            chain = [(rows[i][0], *segments[j])]
            while len(successors.get(node, [])) == 1 and len(predecessors[(node[0] + 1, successors[node][0])]) == 1:
                node = (node[0] + 1, successors[node][0])
                chain.append((rows[node[0]][0], *rows[node[0]][1][node[1]]))
            chains.append(chain)
    return chains


def _drive(chain, start_high=False):
    """Waypoints (point, index) for a chain of passes in order: alternate directions, with a connecting point where
    the next pass does not start under the end of the last one. `index` is the position of the pass in the chain."""
    waypoints = []
    position = None  # the end of the last pass
    for index, (b, a_low, a_high) in enumerate(chain):
        if position is None:
            start, end = (a_high, a_low) if start_high else (a_low, a_high)
        else:
            pa, pb = position
            start, end = (a_low, a_high) if abs(pa - a_low) <= abs(pa - a_high) else (a_high, a_low)
            if abs(pa - start) > _SAME_PLACE_M:
                if a_low - _SAME_PLACE_M <= pa <= a_high + _SAME_PLACE_M:
                    waypoints.append(((pa, b), index))  # step across at the end of the last pass, then along this one
                else:
                    waypoints.append(((start, pb), index))  # back along the last pass to under the start, then step
        waypoints.append(((start, b), index))
        waypoints.append(((end, b), index))
        position = (end, b)
    return waypoints


def _first_blocked_pass(waypoints, space):
    """Chain position of the first pass whose connection to the one before it is not free, or None."""
    for (p, i), (q, j) in zip(waypoints, waypoints[1:]):
        if abs(p[1] - q[1]) < 1e-9:
            continue  # along one row, which is free by construction
        if not space.route_free(p, q):
            return max(i, j)
    return None


def _cells(rows, space):
    """Chains, cut wherever the step between two passes would cross a no-go zone or leave the field."""
    cells = []
    for chain in _chains(rows):
        rest = chain
        while rest:
            cut = _first_blocked_pass(_drive(rest), space)
            if not cut:
                cells.append(rest)
                break
            cells.append(rest[:cut])
            rest = rest[cut:]
    return cells


def _sequence(cells, space):
    """Waypoints (a, b) visiting every cell: lowest first, then the nearest by route, joined by shortest routes."""
    variants = []
    for cell in cells:
        options = []
        for reverse in (False, True):
            for start_high in (False, True):
                points = _drive(cell[::-1] if reverse else cell, start_high)
                if _first_blocked_pass(points, space) is None:
                    options.append([p for p, _ in points])
        variants.append(options)

    first = min(range(len(cells)), key=lambda c: (cells[c][0][0], cells[c][0][1]))
    waypoints = list(variants[first][0])
    remaining = [c for c in range(len(cells)) if c != first]
    while remaining:
        position = waypoints[-1]
        tree = space.routes_from(position)
        # Nearest first by straight-line distance, which can never be longer than the route, so stop once the best
        # route found is shorter than the next candidate's straight line.
        candidates = sorted((math.hypot(option[0][0] - position[0], option[0][1] - position[1]), c, k)
                            for c in remaining for k, option in enumerate(variants[c]))
        best = None
        for straight, c, k in candidates:
            if best is not None and straight >= best[0] - 1e-9:
                break
            route = space.route_to(tree, variants[c][k][0])
            if route is not None and (best is None or route[0] < best[0] - 1e-9):
                best = (route[0], c, variants[c][k], route[1])
        if best is None:
            raise FieldError(
                "The rover cannot get from one part of the field to another: the no-go zones block every route."
            )
        _, c, option, path = best
        waypoints.extend(path[1:-1])  # corners passed on the way; the ends are where we are and where the cell starts
        waypoints.extend(option)
        remaining.remove(c)
    return waypoints


def plan_coverage(boundary, spacing_m, inset_m=0.0, heading_deg=None, exclusions=()):
    """Waypoints (lat, lon) covering a polygon field, avoiding no-go zones.

    boundary: (lat, lon) points in order around the field (for exactly four points, in any order).
    inset_m: keep the passes this far inside the boundary and away from the no-go zones (0 puts pass ends on them).
    heading_deg: compass direction of the passes (0 = north, 90 = east); None picks one automatically: the direction
    of the longest side if that gives one continuous run, otherwise the side direction with the fewest pieces.
    exclusions: no-go zones, each a ring of (lat, lon) points inside the boundary.

    Raises FieldError when the field cannot be planned, with a message that says what to change."""
    prepared = prepare_field(boundary, spacing_m, inset_m, exclusions)
    ref_lat, ref_lon = prepared.ref

    headings = [heading_deg % 180] if heading_deg is not None else _heading_candidates(prepared.driven)
    best = None
    for heading in headings:
        u = (math.sin(math.radians(heading)), math.cos(math.radians(heading)))
        driven = _to_frame(prepared.driven, u)
        keepouts = [_to_frame(k, u) for k in prepared.keepouts]
        space = _Space(driven, keepouts)
        rows = _rows([driven] + keepouts, spacing_m)
        if not rows:
            continue
        cells = _cells(rows, space)
        if best is None or len(cells) < len(best[0]):
            best = (cells, space, u)
        if len(cells) == 1:
            break
    if best is None:
        raise FieldError("No passes fit inside the field. Check the boundary, the spacing and the inset.")
    cells, space, u = best

    v = (-u[1], u[0])
    waypoints = []
    for a, b in _sequence(cells, space):
        x, y = a * u[0] + b * v[0], a * u[1] + b * v[1]
        waypoints.append(to_gps(x, y, ref_lat, ref_lon))
    if len(waypoints) > MAX_WAYPOINTS:
        raise FieldError(
            f"That plan would have {len(waypoints)} waypoints, more than the {MAX_WAYPOINTS} limit. "
            "Check the boundary and the pass spacing."
        )
    return waypoints
