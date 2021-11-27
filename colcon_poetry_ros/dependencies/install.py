import argparse
from pathlib import Path
import sys
import os
from typing import List
import subprocess


def main():
    args = _parse_args()

    os.environ["POETRY_VIRTUALENVS_CREATE"] = "false"

    for package_dir in _discover_packages(args.from_paths):
        print(f"Installing dependencies for {package_dir.name}...", file=sys.stderr)
        subprocess.run(
            ["poetry", "install", "--no-dev"],
            check=True,
            cwd=package_dir,
        )

    print("\nDependencies installed!", file=sys.stderr)


def _discover_packages(from_paths: List[Path]) -> List[Path]:
    package_dirs: List[Path] = []

    potential_packages = []
    for path in from_paths:
        potential_packages += list(path.glob("*"))

    for path in potential_packages:
        if path.is_dir() and (path / "pyproject.toml").is_file():
            if not (path / "poetry.lock").is_file():
                print(
                    f"Package {path.name} is missing a poetry.lock file. Have you run "
                    f"'poetry.lock'?",
                    file=sys.stderr,
                )
                sys.exit(1)

            package_dirs.append(path)

    if len(package_dirs) == 0:
        from_paths_str = ", ".join([str(p) for p in from_paths])
        print(
            f"No packages were found in the following paths: {from_paths_str}",
            file=sys.stderr,
        )
        sys.exit(1)

    return package_dirs


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Installs Python dependencies via Poetry for a package of ROS "
        "nodes. Dependencies are not installed in a virtualenv by default, so this "
        "tool is best used in a container."
    )

    parser.add_argument(
        "--from-paths",
        nargs="+",
        type=Path,
        default=[Path.cwd()],
        help="The path to start looking for Poetry projects in. Defaults to the "
        "current directory."
    )

    return parser.parse_args()


if __name__ == "__main__":
    main()
