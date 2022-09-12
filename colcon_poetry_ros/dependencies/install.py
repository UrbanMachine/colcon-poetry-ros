import argparse
from pathlib import Path
import sys
from typing import List
import subprocess
import logging
from tempfile import NamedTemporaryFile

from colcon_poetry_ros.package_identification.poetry import (
    PoetryPackage,
    NotAPoetryPackage,
)


def main():
    args = _parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    for project in _discover_packages(args.base_paths):
        logging.info(f"Installing dependencies for {project.path.name}...")
        if args.method == "pip":
            _install_dependencies_via_pip(
                project, args.install_base, args.merge_install
            )
        elif args.method == "bundle":
            _install_dependencies_via_poetry_bundle(
                project, args.install_base, args.merge_install
            )
        else:
            logging.error(f"Invalid method argument: '{args.method}'")
            sys.exit(1)

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
            except NotAPoetryPackage:
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


def _install_dependencies_via_poetry_bundle(
    project: PoetryPackage, install_base: Path, merge_install: bool
) -> None:
    try:
        subprocess.run(
            ["poetry", "bundle", "venv", "--help"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError as ex:
        logging.error(
            "The Poetry bundle plugin does not appear to be installed! See the "
            "project page for installation instructions: "
            "https://github.com/python-poetry/poetry-plugin-bundle"
        )
        sys.exit(1)

    if not merge_install:
        install_base /= project.name

    subprocess.run(
        ["poetry", "bundle", "venv", str(install_base.absolute())],
        check=True,
        cwd=project.path
    )


def _install_dependencies_via_pip(
    project: PoetryPackage, install_base: Path, merge_install: bool
) -> None:
    with NamedTemporaryFile("w") as requirements_file:
        requirements_data = project.get_requirements_txt([])
        requirements_file.write(requirements_data)
        requirements_file.flush()

        if not merge_install:
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Searches for Poetry packages and installs their dependencies "
        "to a configurable install base"
    )

    parser.add_argument(
        "--method",
        type=str,
        default="pip",
        help="The method to use when installing dependencies. The 'pip' option exports "
        "Poetry dependencies to a requirements.txt and installs them using the `pip` "
        "command. This is the default option. The 'bundle' option uses the Poetry "
        "Bundle plugin to install dependencies. This is the preferred option, but "
        "using it requires Poetry 1.2.0 or higher and the Bundle plugin to be "
        "installed."
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
