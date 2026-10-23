"""GPS geometry: local flat-earth frame, bearings, distances and row projection."""
import math

METERS_PER_DEGREE_LAT = 111111.0
EARTH_RADIUS_M = 6371000


def to_meters(lat, lon, ref_lat, ref_lon):
    """Convert a GPS point to (x, y) meters relative to a reference point (flat earth, valid for small areas ~<1 km)."""
    y = (lat - ref_lat) * METERS_PER_DEGREE_LAT
    x = (lon - ref_lon) * METERS_PER_DEGREE_LAT * math.cos(math.radians(ref_lat))
    return x, y


def to_gps(x, y, ref_lat, ref_lon):
    """Convert (x, y) meters back to GPS given a reference point."""
    lat = ref_lat + y / METERS_PER_DEGREE_LAT
    lon = ref_lon + x / (METERS_PER_DEGREE_LAT * math.cos(math.radians(ref_lat)))
    return lat, lon


def calculate_bearing(cur_lat, cur_lon, tgt_lat, tgt_lon):
    """Initial bearing from the current point to the target, degrees clockwise from true north (0-360)."""
    lat1 = math.radians(cur_lat)
    lat2 = math.radians(tgt_lat)
    dlon = math.radians(tgt_lon - cur_lon)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def heading_error(current_heading, target_bearing):
    """Signed turn needed, normalized to -180..180. Positive means turn right."""
    error = target_bearing - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error


def distance_meters(cur_lat, cur_lon, tgt_lat, tgt_lon):
    """Great-circle (haversine) distance in meters."""
    lat1, lat2 = math.radians(cur_lat), math.radians(tgt_lat)
    dlat = math.radians(tgt_lat - cur_lat)
    dlon = math.radians(tgt_lon - cur_lon)
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.asin(math.sqrt(a))


def project_onto_row(cur_lat, cur_lon, row_start, row_end, ref):
    """Project the current position onto the row segment (row_start to row_end, both (lat, lon)).

    This removes lateral drift during straight runs: the robot navigates as if it is exactly on the
    line, so small sideways GPS errors do not cause steering corrections. `ref` is the (lat, lon)
    origin of the local frame. The projection is clamped to the segment."""
    ref_lat, ref_lon = ref
    px, py = to_meters(cur_lat, cur_lon, ref_lat, ref_lon)
    ax, ay = to_meters(*row_start, ref_lat, ref_lon)
    bx, by = to_meters(*row_end, ref_lat, ref_lon)
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-10:
        return cur_lat, cur_lon
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    return to_gps(ax + t * dx, ay + t * dy, ref_lat, ref_lon)
