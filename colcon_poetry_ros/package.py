import subprocess
from pathlib import Path
import logging
from typing import List, Set

import toml


class NotAPoetryROSPackage(Exception):
    """The given directory does not point to a ROS Poetry project"""


class PoetryROSPackage:
    """Contains information on a ROS package defined with Poetry"""

    def __init__(self, path: Path, logger: logging.Logger = logging):
        """
        :param path: The root path of the Poetry project
        :param logger: A logger to log with!
        """
        self.path = path

        self.pyproject_file = path / "pyproject.toml"
        if not self.pyproject_file.is_file():
            # Poetry requires a pyproject.toml to function
            raise NotAPoetryROSPackage()

        if not (path / "package.xml").is_file():
            logger.info(
                f"Ignoring pyproject.toml in {path} because the directory does "
                f"not have a package.xml file. This suggests that it is not a ROS "
                f"package."
            )
            raise NotAPoetryROSPackage()

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
            raise NotAPoetryROSPackage()

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
        command = ["poetry", "export", "--format", "requirements.txt"]

        for extra in extras:
            command += ["--extras", extra]

        try:
            result = subprocess.run(
                command,
                cwd=self.path,
                stdout=subprocess.PIPE,
                check=True,
                encoding="utf-8",
            )
        except subprocess.CalledProcessError as ex:
            raise RuntimeError(
                f"Failed to export Poetry dependencies in the requirements.txt format: "
                f"{ex}"
            )

        return result.stdout

    def get_dependencies(self, extras: List[str]) -> Set[str]:
        """Gets dependencies for a Poetry project.

        :param extras: Names of extras whose dependencies should be included
        :return: A list of dependencies in PEP440 format
        """
        requirements_txt = self.get_requirements_txt(extras)
        return _parse_requirements_txt(requirements_txt)


def _parse_requirements_txt(input_: str) -> Set[str]:
    """Parses Python dependencies in the requirements.txt format.

    :param input_: The text in the requirements.txt
    :return: A list of dependencies in PEP440 format
    """
    specifications: List[str] = []
    dependency = ""

    for line in input_.split("\n"):
        # Whitespace around definitions is never semantically significant
        line = line.strip()

        if line.startswith("#"):
            # Just a comment, ignore it
            continue
        elif len(line) == 0:
            continue

        dependency += line
        if line.endswith("\\"):
            # Line continuation! The dependency definition continues
            dependency += line[:-1]
        else:
            # The dependency definition is complete. Remove any environment
            # markers, leaving only the specification.
            spec = dependency.split(";")[0]
            specifications.append(spec)

    return set(specifications)

