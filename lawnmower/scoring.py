"""Score a driven track against a field: how much of the grass was cut and how far the rover strayed.

All points are (x, y) meters in one local frame, x east and y north. The field's polygons are in the
same frame. The track is where the rover really was, not where it thought it was."""
import numpy as np

from .polygon import distance_to_boundary


def inside_mask(xs, ys, polygon):
    """Boolean array: which of the points (arrays xs, ys) are inside the polygon (ray casting)."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    inside = np.zeros(xs.shape, dtype=bool)
    for (x1, y1), (x2, y2) in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        if y1 == y2:
            continue
        crosses = (y1 > ys) != (y2 > ys)
        x_at_y = x1 + (ys - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (xs < x_at_y)
    return inside


def mowable_mask(xs, ys, boundary, exclusions=()):
    """Points inside the boundary and outside every no-go zone."""
    mask = inside_mask(xs, ys, boundary)
    for zone in exclusions:
        mask &= ~inside_mask(xs, ys, zone)
    return mask


def coverage_pct(track, cut_width_m, boundary, exclusions=(), resolution_m=0.25):
    """Percent of the mowable area (inside the boundary, outside the zones) that a strip cut_width_m wide,
    centred on the track, passed over. Measured on a grid of resolution_m cells."""
    xs = np.arange(min(p[0] for p in boundary) + resolution_m / 2, max(p[0] for p in boundary), resolution_m)
    ys = np.arange(min(p[1] for p in boundary) + resolution_m / 2, max(p[1] for p in boundary), resolution_m)
    if len(xs) == 0 or len(ys) == 0:
        return 0.0
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    mowable = mowable_mask(gx.ravel(), gy.ravel(), boundary, exclusions).reshape(gx.shape)
    if not mowable.any():
        return 0.0

    cut = np.zeros(gx.shape, dtype=bool)
    radius = cut_width_m / 2.0
    reach = int(np.ceil(radius / resolution_m)) + 1
    last = None
    for x, y in track:
        # Stamping every point of a dense track is wasteful: skip points closer than a quarter cell.
        if last is not None and (x - last[0]) ** 2 + (y - last[1]) ** 2 < (resolution_m / 4) ** 2:
            continue
        last = (x, y)
        i = int(round((x - xs[0]) / resolution_m))
        j = int(round((y - ys[0]) / resolution_m))
        i0, i1 = max(i - reach, 0), min(i + reach + 1, len(xs))
        j0, j1 = max(j - reach, 0), min(j + reach + 1, len(ys))
        if i0 >= i1 or j0 >= j1:
            continue
        near = (gx[i0:i1, j0:j1] - x) ** 2 + (gy[i0:i1, j0:j1] - y) ** 2 <= radius ** 2
        cut[i0:i1, j0:j1] |= near
    return 100.0 * float((cut & mowable).sum()) / float(mowable.sum())


def excursions(track, boundary, exclusions=()):
    """(farthest outside the boundary, deepest into a no-go zone), in meters; 0 when the track stayed legal."""
    pts = np.asarray(track, dtype=float)
    if len(pts) == 0:
        return 0.0, 0.0
    outside = 0.0
    for x, y in pts[~inside_mask(pts[:, 0], pts[:, 1], boundary)]:
        outside = max(outside, distance_to_boundary((x, y), boundary))
    into_zone = 0.0
    for zone in exclusions:
        for x, y in pts[inside_mask(pts[:, 0], pts[:, 1], zone)]:
            into_zone = max(into_zone, distance_to_boundary((x, y), zone))
    return outside, into_zone
