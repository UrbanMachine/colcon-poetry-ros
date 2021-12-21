import argparse
from pathlib import Path
import sys
from typing import List
import subprocess
import logging
from tempfile import NamedTemporaryFile

from colcon_poetry_ros.package_identification.poetry import (
    PoetryROSPackage,
    NotAPoetryROSPackage,
)


def main():
    args = _parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    for project in _discover_packages(args.base_paths):
        logging.info(f"Installing dependencies for {project.path.name}...")

        with NamedTemporaryFile("w") as requirements_file:
            requirements_data = project.get_requirements_txt([])
            requirements_file.write(requirements_data)
            requirements_file.flush()

            install_base = args.install_base
            if not args.merge_install:
                install_base /= project.name

            subprocess.run(
                [
                    "python3",
                    "-m",
                    "pip",
                    "install",
                    # I don't understand why, but providing this fixes:
                    # https://github.com/pypa/pip/issues/9644
                    # Despite what the name would imply, dependencies are still
                    # installed
                    "--no-deps",
                    # Forces installation even if the package is installed at the
                    # system level
                    "--ignore-installed",
                    # Turns off Pip's check to ensure installed binaries are in the
                    # PATH. ROS workspaces take care of setting the PATH, but Pip
                    # doesn't know that.
                    "--no-warn-script-location",
                    "--requirement",
                    requirements_file.name,
                    "--prefix",
                    install_base,
                ],
                check=True,
            )

    logging.info("\nDependencies installed!")


def _discover_packages(base_paths: List[Path]) -> List[PoetryROSPackage]:
    projects: List[PoetryROSPackage] = []

    potential_packages = []
    for path in base_paths:
        potential_packages += list(path.glob("*"))

    for path in potential_packages:
        if path.is_dir():
            try:
                project = PoetryROSPackage(path)
            except NotAPoetryROSPackage:
                continue
            else:
                projects.append(project)

    if len(projects) == 0:
        base_paths_str = ", ".join([str(p) for p in base_paths])
        logging.error(
            f"No packages were found in the following paths: {base_paths_str}"
        )
        sys.exit(1)

    return projects


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Searches for ROS Poetry packages and installs their dependencies "
        "to a configurable install base"
    )

    parser.add_argument(
        "--base-paths",
        nargs="+",
        type=Path,
        default=[Path.cwd()],
        help="The paths to start looking for Poetry projects in. Defaults to the "
        "current directory."
    )

    parser.add_argument(
        "--install-base",
        type=Path,
        default=Path("install"),
        help="The base path for all install prefixes (default: install)",
    )

    parser.add_argument(
        "--merge-install",
        action="store_true",
        help="Merge all install prefixes into a single location",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="If provided, debug logs will be printed",
    )

    return parser.parse_args()


if __name__ == "__main__":
    main()
