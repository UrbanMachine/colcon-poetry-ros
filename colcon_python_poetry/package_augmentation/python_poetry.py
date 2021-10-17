import subprocess
import shutil
from typing import List, Set

import toml
from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_identification.python import \
    create_dependency_descriptor


class PythonPackageAugmentation(PackageAugmentationExtensionPoint):
    """Augment Python packages that use Poetry by referencing the pyproject.toml file"""

    def __init__(self):
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION,
            "^1.0",
        )

    def augment_package(
        self, desc: PackageDescriptor, *, additional_argument_names=None
    ):
        if "uses_poetry" not in desc.metadata:
            # Only run the augmenter if the Poetry identifier was used
            return

        lock_file = desc.path / "poetry.lock"
        if not lock_file.is_file():
            raise RuntimeError(
                f"The lock file is missing, expected at '{lock_file}'. Have you run "
                f"'poetry lock'?"
            )

        if not shutil.which("poetry"):
            raise RuntimeError(
                "Could not find the poetry command. Is Poetry installed?"
            )

        pyproject_toml = desc.path / "pyproject.toml"
        pyproject = toml.loads(pyproject_toml.read_text())

        # See https://www.python.org/dev/peps/pep-0518/#build-system-table
        if "build-system" in pyproject and "requires" in pyproject["build-system"]:
            build_deps = pyproject["build-system"]["requires"]
        else:
            build_deps = set()

        all_deps = _get_dependencies(desc, include_dev=True)
        no_dev_deps = _get_dependencies(desc, include_dev=False)
        # Use dev dependencies as test dependencies
        test_deps = all_deps.difference(no_dev_deps)

        desc.dependencies["build_depends"] = set(
            create_dependency_descriptor(dep) for dep in build_deps
        )
        desc.dependencies["run_depends"] = set(
            create_dependency_descriptor(dep) for dep in no_dev_deps
        )
        desc.dependencies["test_depends"] = set(
            create_dependency_descriptor(dep) for dep in test_deps
        )


def _get_dependencies(
    desc: PackageDescriptor,
    include_dev: bool = False,
) -> Set[str]:
    command = ["poetry", "export", "--format", "requirements.txt"]
    if include_dev:
        command += ["--dev"]

    try:
        result = subprocess.run(
            command,
            cwd=desc.path,
            stdout=subprocess.PIPE,
            check=True,
            encoding="utf-8",
        )
    except subprocess.CalledProcessError as ex:
        raise RuntimeError(
            f"Failed to export Poetry dependencies in the requirements.txt format: "
            f"{ex}"
        )

    return _parse_requirements(result.stdout)


def _parse_requirements(input_: str) -> Set[str]:
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
