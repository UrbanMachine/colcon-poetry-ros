import shutil

import toml
from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_augmentation.python import \
    create_dependency_descriptor, logger

from colcon_poetry_ros import config
from colcon_poetry_ros.package_identification.poetry import PoetryROSPackage


class PoetryPackageAugmentation(PackageAugmentationExtensionPoint):
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
        if desc.type != "poetry":
            # Some other identifier claimed this package
            return

        project = PoetryROSPackage(desc.path, logger)
        project.check_lock_file_exists()

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

        run_deps = project.get_dependencies(config.run_depends_extras.get())
        test_deps = project.get_dependencies(config.test_depends_extras.get())

        desc.dependencies["build_depends"] = set(
            create_dependency_descriptor(dep) for dep in build_deps
        )
        desc.dependencies["run_depends"] = set(
            create_dependency_descriptor(dep) for dep in run_deps
        )
        desc.dependencies["test_depends"] = set(
            create_dependency_descriptor(dep) for dep in test_deps
        )
