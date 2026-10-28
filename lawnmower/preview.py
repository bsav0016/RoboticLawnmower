"""Draw a mowing plan on a laptop before the rover moves: GeoJSON for a map tool, and a stand-alone SVG.

Pure Python, standard library only, no hardware. Both outputs show the field boundary, the boundary moved inwards by
`inset_m` (when there is one), the no-go zones (and their margin), the path through the waypoints, the start and the end.
"""
import math
from xml.sax.saxutils import escape

from lawnmower.coverage import prepare_field
from lawnmower.geometry import distance_meters, to_gps, to_meters
from lawnmower.polygon import signed_area

# The SVG is drawn this many pixels across on its longest side.
SVG_SIZE_PX = 760
SVG_MARGIN_PX = 36
SVG_FOOTER_PX = 52
# Individual waypoint dots are drawn only for plans this small, so a large plan stays readable.
MAX_DOTS = 400
_SCALE_BAR_CHOICES_M = (1, 2, 5, 10, 20, 50, 100, 200, 500)


def path_length_m(waypoints):
    """Length of the path through the waypoints, in metres."""
    return sum(distance_meters(a[0], a[1], b[0], b[1]) for a, b in zip(waypoints, waypoints[1:]))


def _summary(field, waypoints, outline):
    return {
        "waypoints": len(waypoints),
        "path_length_m": round(path_length_m(waypoints), 1),
        "area_m2": round(signed_area(outline), 1),
        "pass_spacing_m": field.pass_spacing_m,
        "inset_m": field.inset_m,
    }


def _lonlat_ring(points_m, ref):
    ring = [to_gps(x, y, ref[0], ref[1]) for x, y in points_m]
    ring.append(ring[0])
    return [[lon, lat] for lat, lon in ring]  # GeoJSON order is longitude first


def plan_geojson(field, waypoints):
    """The plan as a GeoJSON FeatureCollection (a dict): boundary, inset boundary, path, start and end."""
    prepared = prepare_field(field.boundary, field.pass_spacing_m, field.inset_m, field.exclusions)
    outline, driven, ref = prepared.outline, prepared.driven, prepared.ref
    features = [{
        "type": "Feature",
        "properties": {"role": "boundary", **_summary(field, waypoints, outline)},
        "geometry": {"type": "Polygon", "coordinates": [_lonlat_ring(outline, ref)]},
    }]
    if field.inset_m > 0:
        features.append({
            "type": "Feature",
            "properties": {"role": "inset-boundary", "inset_m": field.inset_m},
            "geometry": {"type": "Polygon", "coordinates": [_lonlat_ring(driven, ref)]},
        })
    for index, (zone, keepout) in enumerate(zip(prepared.exclusions, prepared.keepouts), start=1):
        features.append({
            "type": "Feature",
            "properties": {"role": "exclusion", "index": index},
            "geometry": {"type": "Polygon", "coordinates": [_lonlat_ring(zone, ref)]},
        })
        if field.inset_m > 0:
            features.append({
                "type": "Feature",
                "properties": {"role": "keepout", "index": index, "margin_m": field.inset_m},
                "geometry": {"type": "Polygon", "coordinates": [_lonlat_ring(keepout, ref)]},
            })
    features.append({
        "type": "Feature",
        "properties": {"role": "path"},
        "geometry": {"type": "LineString", "coordinates": [[lon, lat] for lat, lon in waypoints]},
    })
    for role, point in (("start", waypoints[0]), ("end", waypoints[-1])):
        features.append({
            "type": "Feature",
            "properties": {"role": role},
            "geometry": {"type": "Point", "coordinates": [point[1], point[0]]},
        })
    return {"type": "FeatureCollection", "features": features}


def _nice_length(max_m):
    """The biggest round scale-bar length that is not more than max_m."""
    fitting = [m for m in _SCALE_BAR_CHOICES_M if m <= max_m]
    return fitting[-1] if fitting else _SCALE_BAR_CHOICES_M[0]


def plan_svg(field, waypoints):
    """The plan as a stand-alone SVG document (a string), north up, with a scale bar."""
    prepared = prepare_field(field.boundary, field.pass_spacing_m, field.inset_m, field.exclusions)
    outline, driven, ref = prepared.outline, prepared.driven, prepared.ref
    path = [to_meters(lat, lon, ref[0], ref[1]) for lat, lon in waypoints]

    xs = [p[0] for p in outline + path]
    ys = [p[1] for p in outline + path]
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    scale = SVG_SIZE_PX / span
    width = (max_x - min_x) * scale + 2 * SVG_MARGIN_PX
    height = (max_y - min_y) * scale + 2 * SVG_MARGIN_PX + SVG_FOOTER_PX

    def px(point):
        return (SVG_MARGIN_PX + (point[0] - min_x) * scale, SVG_MARGIN_PX + (max_y - point[1]) * scale)

    def points_attr(points):
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in map(px, points))

    summary = _summary(field, waypoints, outline)
    title = "Mowing plan"
    footer = (f"{summary['waypoints']} waypoints | path {summary['path_length_m']:g} m | "
              f"field {summary['area_m2']:g} m2 | spacing {field.pass_spacing_m:g} m | inset {field.inset_m:g} m")

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'viewBox="0 0 {width:.0f} {height:.0f}" font-family="sans-serif">',
        f"<title>{escape(title)}</title>",
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<polygon id="boundary" points="{points_attr(outline)}" fill="#e3f1e3" stroke="#2f6b2f" stroke-width="2"/>',
    ]
    if field.inset_m > 0:
        parts.append(f'<polygon id="inset-boundary" points="{points_attr(driven)}" fill="none" stroke="#2f6b2f" '
                     f'stroke-width="1" stroke-dasharray="5 4"/>')
    for index, (zone, keepout) in enumerate(zip(prepared.exclusions, prepared.keepouts), start=1):
        if field.inset_m > 0:
            parts.append(f'<polygon id="keepout-{index}" points="{points_attr(keepout)}" fill="none" stroke="#b5382c" '
                         f'stroke-width="1" stroke-dasharray="5 4"/>')
        parts.append(f'<polygon id="exclusion-{index}" points="{points_attr(zone)}" fill="#f3c9c4" stroke="#b5382c" '
                     f'stroke-width="2"/>')
    parts.append(f'<polyline id="path" points="{points_attr(path)}" fill="none" stroke="#1f5fbf" stroke-width="1.5" '
                 f'stroke-linejoin="round"/>')
    if len(path) <= MAX_DOTS:
        for x, y in map(px, path):
            parts.append(f'<circle class="waypoint" cx="{x:.1f}" cy="{y:.1f}" r="2" fill="#1f5fbf"/>')
    (sx, sy), (ex, ey) = px(path[0]), px(path[-1])
    parts.append(f'<circle id="start" cx="{sx:.1f}" cy="{sy:.1f}" r="6" fill="#1a9a3a" stroke="#ffffff" stroke-width="1.5"/>')
    parts.append(f'<text x="{sx + 9:.1f}" y="{sy - 8:.1f}" font-size="12" fill="#1a6e2c">start</text>')
    parts.append(f'<circle id="end" cx="{ex:.1f}" cy="{ey:.1f}" r="6" fill="#d33a2c" stroke="#ffffff" stroke-width="1.5"/>')
    parts.append(f'<text x="{ex + 9:.1f}" y="{ey + 16:.1f}" font-size="12" fill="#a02a20">end</text>')

    # North arrow, top right.
    nx = width - SVG_MARGIN_PX / 2 - 6
    parts.append(f'<line x1="{nx:.1f}" y1="34" x2="{nx:.1f}" y2="12" stroke="#444444" stroke-width="2"/>')
    parts.append(f'<polygon points="{nx:.1f},8 {nx - 4:.1f},16 {nx + 4:.1f},16" fill="#444444"/>')
    parts.append(f'<text x="{nx:.1f}" y="48" font-size="12" text-anchor="middle" fill="#444444">N</text>')

    # Scale bar and the summary line, in the footer.
    bar_m = _nice_length(span / 2)
    base_y = height - SVG_FOOTER_PX + 14
    parts.append(f'<line x1="{SVG_MARGIN_PX}" y1="{base_y:.1f}" x2="{SVG_MARGIN_PX + bar_m * scale:.1f}" y2="{base_y:.1f}" '
                 f'stroke="#222222" stroke-width="3"/>')
    parts.append(f'<text x="{SVG_MARGIN_PX}" y="{base_y + 16:.1f}" font-size="12" fill="#222222">{bar_m} m</text>')
    parts.append(f'<text x="{SVG_MARGIN_PX}" y="{base_y + 34:.1f}" font-size="12" fill="#555555">{escape(footer)}</text>')
    parts.append("</svg>")
    return "\n".join(parts) + "\n"
