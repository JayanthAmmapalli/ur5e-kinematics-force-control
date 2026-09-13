#!/usr/bin/env python3
"""Link the mujoco_menagerie UR5e meshes into models/assets.

models/ur5e_arm.xml is a modified copy of menagerie's ur5e.xml, but the 30 MB of
meshes it references are NOT vendored into this repository. Instead this script
symlinks models/assets at the menagerie clone, which keeps the repo light and the
model file directly loadable:

    python -m mujoco.viewer --mjcf=models/scene.xml

Clone the menagerie first (anywhere; outside this repo is recommended so it stays
out of git and out of any directory-level backup):

    git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git \\
        ~/projects/mujoco_menagerie

Set MUJOCO_MENAGERIE if you put it somewhere other than ~/projects/mujoco_menagerie.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LINK = REPO / "models" / "assets"
DEFAULT_MENAGERIE = Path.home() / "projects" / "mujoco_menagerie"
# One mesh that must exist, as proof we are pointed at the real thing.
SENTINEL = "base_0.obj"


def menagerie_root() -> Path:
    env = os.environ.get("MUJOCO_MENAGERIE")
    return Path(env).expanduser().resolve() if env else DEFAULT_MENAGERIE


def main() -> int:
    root = menagerie_root()
    assets = root / "universal_robots_ur5e" / "assets"

    if not assets.is_dir():
        print("error: UR5e assets not found at %s" % assets, file=sys.stderr)
        print("\nClone the menagerie, or set MUJOCO_MENAGERIE to where it already is:",
              file=sys.stderr)
        print("  git clone --depth 1 "
              "https://github.com/google-deepmind/mujoco_menagerie.git %s" % root,
              file=sys.stderr)
        return 1

    if not (assets / SENTINEL).is_file():
        print("error: %s exists but has no %s -- is this really the UR5e asset dir?"
              % (assets, SENTINEL), file=sys.stderr)
        return 1

    if LINK.is_symlink() or LINK.exists():
        if LINK.is_symlink() and LINK.resolve() == assets:
            print("already linked: %s -> %s" % (LINK.relative_to(REPO), assets))
            return 0
        if not LINK.is_symlink():
            print("error: %s exists and is not a symlink; refusing to replace it"
                  % LINK, file=sys.stderr)
            return 1
        LINK.unlink()

    LINK.parent.mkdir(parents=True, exist_ok=True)
    LINK.symlink_to(assets, target_is_directory=True)
    print("linked: %s -> %s" % (LINK.relative_to(REPO), assets))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
