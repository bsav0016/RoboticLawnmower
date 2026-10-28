import math

# --- Define the 4 corner GPS coordinates of the box ---
# Order does not matter; the script will sort them automatically.
# The field comes from the local field.json or .env file (see field.example.json and .env.example) when run standalone.
from field_config import load_field
from lawnmower.coverage import plan_coverage
from lawnmower.field import DEFAULT_SPACING_M, FieldError, check_corners
from lawnmower.geometry import to_gps, to_meters  # noqa: F401

# --- Lawnmower spacing (meters between parallel passes) ---
PASS_SPACING_M = DEFAULT_SPACING_M

# --- Output file ---
OUTPUT_FILE = "waypoints.py"


# ---------------------------------------------------------------------------
# Coordinate helpers (flat-earth approximation, valid for small areas ~<1 km)
# ---------------------------------------------------------------------------

# to_meters / to_gps live in lawnmower.geometry and are re-exported here.


# ---------------------------------------------------------------------------
# Corner ordering
# ---------------------------------------------------------------------------

def order_corners(corners):
    """Sort 4 corners into CCW polygon order around their centroid."""
    cx = sum(c[0] for c in corners) / 4
    cy = sum(c[1] for c in corners) / 4
    return sorted(corners, key=lambda c: math.atan2(c[0] - cx, c[1] - cy))


# ---------------------------------------------------------------------------
# Lawnmower pattern generator
# ---------------------------------------------------------------------------

def generate_lawnmower(corners, spacing_m):
    """
    Given 4 GPS corners (any order), generate a lawnmower waypoint list.
    Traverses along the long edge first, steps 'spacing_m' meters, traverses
    back, repeating until the full short dimension is covered.

    Returns a list of (lat, lon) tuples.
    """
    ref_lat = sum(c[0] for c in corners) / 4
    ref_lon = sum(c[1] for c in corners) / 4

    # Project all corners to local meter space
    pts = [to_meters(lat, lon, ref_lat, ref_lon) for lat, lon in corners]

    # Order corners as a convex polygon
    cx = sum(p[0] for p in pts) / 4
    cy = sum(p[1] for p in pts) / 4
    pts = sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

    # Identify which pair of opposite edges is longer (the "long" axis)
    # Polygon vertices in order: 0, 1, 2, 3
    # Edges: 0-1, 1-2, 2-3, 3-0
    def edge_len(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    len_01 = edge_len(pts[0], pts[1])
    len_12 = edge_len(pts[1], pts[2])

    if len_01 >= len_12:
        # Long edges: 0-1 and 3-2  →  sweep along 0→1 direction
        # Short edges: 1-2 and 0-3  →  step along 0→3 direction
        sweep_start_a = pts[0]
        sweep_end_a   = pts[1]
        sweep_start_b = pts[3]
        sweep_end_b   = pts[2]
    else:
        # Long edges: 1-2 and 0-3  →  sweep along 1→2 direction
        # Short edges: 0-1 and 3-2  →  step along 0→3 direction... wait, reorder
        sweep_start_a = pts[1]
        sweep_end_a   = pts[2]
        sweep_start_b = pts[0]
        sweep_end_b   = pts[3]

    # Vector along the short axis (from one long edge to the other)
    short_vec = (sweep_start_b[0] - sweep_start_a[0],
                 sweep_start_b[1] - sweep_start_a[1])
    short_len = math.hypot(*short_vec)
    short_unit = (short_vec[0] / short_len, short_vec[1] / short_len)

    # Number of passes needed to cover the short dimension
    num_passes = max(1, math.ceil(short_len / spacing_m) + 1)

    waypoints = []

    for i in range(num_passes):
        t = min(i * spacing_m / short_len, 1.0)

        # Interpolate start and end of this pass along the short axis
        sx = sweep_start_a[0] + t * (sweep_start_b[0] - sweep_start_a[0])
        sy = sweep_start_a[1] + t * (sweep_start_b[1] - sweep_start_a[1])
        ex = sweep_end_a[0]   + t * (sweep_end_b[0]   - sweep_end_a[0])
        ey = sweep_end_a[1]   + t * (sweep_end_b[1]   - sweep_end_a[1])

        # Alternate direction each pass (boustrophedon)
        if i % 2 == 0:
            waypoints.append(to_gps(sx, sy, ref_lat, ref_lon))
            waypoints.append(to_gps(ex, ey, ref_lat, ref_lon))
        else:
            waypoints.append(to_gps(ex, ey, ref_lat, ref_lon))
            waypoints.append(to_gps(sx, sy, ref_lat, ref_lon))

    return waypoints


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def create_waypoint_list(corner_a, corner_b, corner_c, corner_d, spacing_m=1.0):
    """Generate a lawnmower waypoint list from 4 corner GPS coordinates.

    Args:
        corner_a..corner_d: (lat, lon) tuples in any order.
        spacing_m: meters between parallel passes (default 1.0).

    Returns:
        List of (lat, lon) tuples.

    Raises:
        FieldError: the corners or spacing cannot make a sensible plan (see lawnmower/field.py).
    """
    corners = [corner_a, corner_b, corner_c, corner_d]
    check_corners(corners, spacing_m)
    ordered = order_corners(corners)
    return generate_lawnmower(ordered, spacing_m)


def plan_field(field):
    """Waypoints for a Field (see lawnmower/field.py).

    A plain four-sided field (no inset, no chosen heading, no no-go zones) goes through the four-corner planner above,
    which is the one that has driven the rover. Anything else (more or fewer sides, an inset, a chosen heading,
    no-go zones) goes through lawnmower.coverage, which handles any polygon field with no-go zones."""
    plain = field.inset_m == 0 and field.heading_deg is None and not field.exclusions
    if len(field.boundary) == 4 and plain:
        return create_waypoint_list(*field.boundary, spacing_m=field.pass_spacing_m)
    return plan_coverage(field.boundary, field.pass_spacing_m, field.inset_m, field.heading_deg, field.exclusions)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        field = load_field()
        waypoints = plan_field(field)
    except FieldError as error:
        raise SystemExit(f"Cannot plan this field: {error}")

    print(f"Generated {len(waypoints)} waypoints ({len(waypoints) // 2} passes):\n")
    for i, (lat, lon) in enumerate(waypoints):
        print(f"  [{i:3d}]  ({lat:.8f}, {lon:.8f})")

    # Write output file
    with open(OUTPUT_FILE, "w") as f:
        f.write("# Auto-generated lawnmower waypoint list.\n")
        f.write("# Created by create_waypoint_list.py\n\n")
        f.write("WAYPOINTS = [\n")
        for lat, lon in waypoints:
            f.write(f"    ({lat:.8f}, {lon:.8f}),\n")
        f.write("]\n")

    print(f"\nWaypoints saved to '{OUTPUT_FILE}'.")
    print()
    print("# --- Integration snippet for navigate_to_target.py ---")
    print("# Add at the top of navigate_to_target.py:")
    print("#   from waypoints import WAYPOINTS")
    print("#   waypoint_index = 0")
    print("#   TARGET_LAT, TARGET_LON = WAYPOINTS[waypoint_index]")
    print("#")
    print("# Inside the main loop, replace the 'Target reached!' block with:")
    print("#   waypoint_index += 1")
    print("#   if waypoint_index >= len(WAYPOINTS):")
    print("#       print('All waypoints reached!')")
    print("#       break")
    print("#   TARGET_LAT, TARGET_LON = WAYPOINTS[waypoint_index]")
    print("#   print(f'Moving to waypoint {waypoint_index}: ({TARGET_LAT}, {TARGET_LON})')")
