import re
import subprocess
from pathlib import Path
import logging
from tempfile import NamedTemporaryFile
from typing import List, Set

import toml
from packaging.version import VERSION_PATTERN

PACKAGE_NAME_PATTERN = r"^([A-Z0-9]|[A-Z0-9][A-Z0-9._-]*[A-Z0-9])$"
"""Matches on valid package names when run with re.IGNORECASE.
Pulled from: https://peps.python.org/pep-0508/#names
"""


class NotAPoetryPackage(Exception):
    """The given directory does not point to a Poetry project"""


class PoetryPackage:
    """Contains information on a package defined with Poetry"""

    def __init__(self, path: Path, logger: logging.Logger = logging):
        """
        :param path: The root path of the Poetry project
        :param logger: A logger to log with!
        """
        self.path = path
        self.logger = logger

        self.pyproject_file = path / "pyproject.toml"
        if not self.pyproject_file.is_file():
            # Poetry requires a pyproject.toml to function
            raise NotAPoetryPackage()

        try:
            self.pyproject = toml.loads(self.pyproject_file.read_text())
        except toml.TomlDecodeError as ex:
            raise RuntimeError(
                f"Failed to parse {self.pyproject_file} as a TOML file: {ex}"
            )

        if "tool" not in self.pyproject or "poetry" not in self.pyproject["tool"]:
            logger.debug(
                f"The {self.pyproject_file} file does not have a [tool.poetry] "
                f"section. The file is likely there to instruct a tool other than "
                f"Poetry."
            )
            raise NotAPoetryPackage()

        logger.info(f"Project {path} appears to be a Poetry ROS project")

        poetry_config = self.pyproject["tool"]["poetry"]

        if "name" not in poetry_config:
            raise RuntimeError(
                f"Failed to determine Python package name in {self.path}: The "
                f"[tool.poetry] section must have a 'name' field"
            )

        self.name = poetry_config["name"]

    def check_lock_file_exists(self) -> Path:
        """Raises an exception if the lock file is not available.

        :return: The lock file location
        """
        lock_file = self.path / "poetry.lock"
        if not lock_file.is_file():
            raise RuntimeError(
                f"The lock file is missing, expected at '{lock_file}'. Have you run "
                f"'poetry lock'?"
            )
        return lock_file

    def get_requirements_txt(self, extras: List[str]) -> str:
        """Generates a list of the project's dependencies in requirements.txt format.

        :param extras: Names of extras whose dependencies should be included
        :return: The requirements.txt text
        """
        command = [
            "poetry",
            "export",
            "--format",
            "requirements.txt",
        ]

        for extra in extras:
            command += ["--extras", extra]

        # Create a temporary file for `poetry export` to write its output to. We can't
        # just capture stdout because Poetry 1.2 uses stdout for logging, too.
        with NamedTemporaryFile("r") as requirements_file:
            command += ["--output", requirements_file.name]

            try:
                subprocess.run(
                    command,
                    cwd=self.path,
                    check=True,
                    encoding="utf-8",
                )
            except subprocess.CalledProcessError as ex:
                raise RuntimeError(
                    f"Failed to export Poetry dependencies in the requirements.txt "
                    f"format: {ex}"
                )

            return requirements_file.read()

    def get_dependencies(self, extras: List[str]) -> Set[str]:
        """Gets dependencies for a Poetry project.

        :param extras: Names of extras whose dependencies should be included
        :return: A list of dependencies in PEP440 format
        """
        try:
            result = subprocess.run(
                ["poetry", "show", "--no-interaction"],
                cwd=self.path,
                check=True,
                stdout=subprocess.PIPE,
                encoding="utf-8",
            )
        except subprocess.CalledProcessError as ex:
            raise RuntimeError(f"Failed to read package dependencies: {ex}")

        dependencies = set()

        for line in result.stdout.splitlines():
            try:
                dependency = self._parse_dependency_line(line)
            except ValueError as ex:
                self.logger.warning(str(ex))
            else:
                dependencies.add(dependency)

        return dependencies

    def _parse_dependency_line(self, line: str) -> str:
        """Makes a best-effort attempt to parse lines from ``poetry show`` as
        dependencies. Poetry does not have a stable CLI interface, so this logic may
        not be sufficient now or in the future. A smarter approach is needed.

        :param line: A raw line from ``poetry show``
        :return: A dependency string in PEP440 format
        """

        components = line.split()
        if len(components) < 2:
            raise ValueError(f"Could not parse line '{line}' as a dependency")

        name = components[0]
        if re.match(PACKAGE_NAME_PATTERN, name, re.IGNORECASE) is None:
            raise ValueError(f"Invalid dependency name '{name}'")

        version = None

        # Search for an item that looks like a version. Poetry adds other data in front
        # of the version number under certain circumstances.
        for item in components[1:]:
            if re.match(VERSION_PATTERN, item, re.VERBOSE | re.IGNORECASE) is not None:
                version = item

        if version is None:
            raise ValueError(
                f"For dependency '{name}': Could not find version specification "
                f"in '{line}'"
            )

        return f"{name}=={version}"
