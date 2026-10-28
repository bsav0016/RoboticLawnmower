"""Draw the mowing plan for your field on this computer, without touching the robot.

Reads the field the same way navigate_path.py does (field.json, or the corners in .env), plans it, and writes
plan.svg (open it in any browser) and plan.geojson (open it in geojson.io, QGIS or similar to see it on a map).
Run it before a mowing run to check the path; nothing here needs the Pi.

    python preview_plan.py                      # field.json or .env, files written to the current folder
    python preview_plan.py --field other.json --out previews
"""
import argparse
import json
import os

from create_waypoint_list import plan_field
from field_config import load_field
from lawnmower.field import FieldError
from lawnmower.preview import path_length_m, plan_geojson, plan_svg


def main(argv=None):
    parser = argparse.ArgumentParser(description="Write plan.svg and plan.geojson for the field, without the robot.")
    parser.add_argument("--field", help="field file to use (default: field.json, or the corners in .env)")
    parser.add_argument("--out", default=".", help="folder for the files (default: the current folder)")
    args = parser.parse_args(argv)

    field = load_field(field_path=args.field)
    try:
        waypoints = plan_field(field)
        svg = plan_svg(field, waypoints)
        geojson = plan_geojson(field, waypoints)
    except FieldError as error:
        raise SystemExit(f"Cannot plan this field: {error}")

    os.makedirs(args.out, exist_ok=True)
    svg_path = os.path.join(args.out, "plan.svg")
    geojson_path = os.path.join(args.out, "plan.geojson")
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(svg)
    with open(geojson_path, "w", encoding="utf-8") as f:
        json.dump(geojson, f, indent=2)
        f.write("\n")

    print(f"{len(waypoints)} waypoints, path {path_length_m(waypoints):.0f} m")
    print(f"Wrote {svg_path} and {geojson_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
