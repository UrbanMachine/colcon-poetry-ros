import argparse
from pathlib import Path
import sys
from typing import List
import subprocess
import logging

from colcon_poetry_ros.package_identification.poetry import (
    PoetryPackage,
    NotAPoetryPackageError,
)


def main():
    args = _parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    for project in _discover_packages(args.base_paths):
        logging.info(f"Installing dependencies for {project.path.name}...")
        _install_dependencies(
            project, args.install_base, args.merge_install
        )

    logging.info("\nDependencies installed!")


def _discover_packages(base_paths: List[Path]) -> List[PoetryPackage]:
    projects: List[PoetryPackage] = []

    potential_packages = []
    for path in base_paths:
        potential_packages += list(path.glob("*"))

    for path in potential_packages:
        if path.is_dir():
            try:
                project = PoetryPackage(path)
            except NotAPoetryPackageError:
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


def _install_dependencies(
    project: PoetryPackage, install_base: Path, merge_install: bool
) -> None:
    """Uses poetry-bundle-plugin to create a virtual environment with all the project's
    dependencies in Colcon's install directory. We need to use the bundle plugin because
    Poetry does not natively let us install projects to a custom location.
    """
    try:
        subprocess.run(
            ["poetry", "bundle", "venv", "--help"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        logging.error(
            "The Poetry bundle plugin does not appear to be installed! See the "
            "project page for installation instructions: "
            "https://github.com/python-poetry/poetry-plugin-bundle"
        )
        sys.exit(1)

    if not merge_install:
        install_base /= project.name

    subprocess.run(
        [
            "poetry",
            "bundle",
            "venv",
            "--no-interaction",
            str(install_base.absolute()),
        ],
        check=True,
        cwd=project.path
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Searches for Poetry packages and installs their dependencies "
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
