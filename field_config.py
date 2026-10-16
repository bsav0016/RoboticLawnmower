"""Load the field corner coordinates from a local .env file (not committed).

Copy .env.example to .env and fill in CORNER_A..CORNER_D as "lat,lon".
Real environment variables take precedence over the file.
"""
import os

_ENV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")


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
    lat, lon = (float(part) for part in raw.split(","))
    return lat, lon


def load_corners():
    file_values = _load_env_file()
    return [_corner(name, file_values) for name in ("CORNER_A", "CORNER_B", "CORNER_C", "CORNER_D")]
