"""Load the field from local files that are not committed.

Either a field.json (see field.example.json), or the four corners in a .env file: copy .env.example to .env and
fill in CORNER_A..CORNER_D as "lat,lon". Real environment variables take precedence over the .env file.
field.json wins when both exist; FIELD_FILE points at a different field file.
"""
import json
import os

from lawnmower.field import Field, FieldError, parse_field

_ENV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
_FIELD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "field.json")


def _load_env_file(path=_ENV_PATH):
    values = {}
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, _, value = line.partition("=")
                    values[key.strip()] = value.strip()
    return values


def _corner(name, file_values):
    raw = os.environ.get(name) or file_values.get(name)
    if not raw:
        raise SystemExit(f"{name} is not set. Copy .env.example to .env and fill in the field corners.")
    try:
        lat, lon = (float(part) for part in raw.split(","))
    except ValueError:
        raise SystemExit(f'{name} is "{raw}", which is not a "lat,lon" pair such as 37.000000,-122.000000.')
    return lat, lon


def load_corners():
    file_values = _load_env_file()
    return [_corner(name, file_values) for name in ("CORNER_A", "CORNER_B", "CORNER_C", "CORNER_D")]


def load_field(field_path=None, env_path=_ENV_PATH):
    """The field to mow, as a Field.

    Uses the field file when there is one (field_path, else $FIELD_FILE, else field.json next to this script),
    otherwise the four corners from the environment or the .env file."""
    path = field_path or os.environ.get("FIELD_FILE") or _FIELD_PATH
    if not os.path.exists(path):
        if field_path or os.environ.get("FIELD_FILE"):
            raise SystemExit(f"The field file {path} does not exist.")
        file_values = _load_env_file(env_path)
        corners = [_corner(name, file_values) for name in ("CORNER_A", "CORNER_B", "CORNER_C", "CORNER_D")]
        return Field(boundary=tuple(corners))
    try:
        with open(path, encoding="utf-8") as f:
            return parse_field(json.load(f))
    except json.JSONDecodeError as error:
        raise SystemExit(f"{path} is not valid JSON: {error}")
    except FieldError as error:
        raise SystemExit(f"{path}: {error}")
