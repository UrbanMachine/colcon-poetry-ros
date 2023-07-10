import shutil

import toml
from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_augmentation.python import \
    create_dependency_descriptor, logger

from colcon_poetry_ros import config
from colcon_poetry_ros.package_identification.poetry import PoetryPackage


class PoetryPackageAugmentation(PackageAugmentationExtensionPoint):
    """Augment Python packages that use Poetry by referencing the pyproject.toml file"""

    _PACKAGE_SECTION = "tool.colcon-poetry-ros.dependencies"
    _DEPEND_LIST = "depend"
    _BUILD_DEPEND_LIST = "build_depend"
    _EXEC_DEPEND_LIST = "exec_depend"
    _TEST_DEPEND_LIST = "test_depend"
    _PACKAGE_BUILD_CATEGORY = "build"
    _PACKAGE_EXEC_CATEGORY = "run"
    _PACKAGE_TEST_CATEGORY = "test"

    def __init__(self):
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION,
            "^1.0",
        )

    def augment_package(
        self, desc: PackageDescriptor, *, additional_argument_names=None
    ):
        if desc.type != "poetry.python":
            # Some other identifier claimed this package
            return

        project = PoetryPackage(desc.path, logger)
        project.check_lock_file_exists()

        if not shutil.which("poetry"):
            raise RuntimeError(
                "Could not find the poetry command. Is Poetry installed?"
            )

        pyproject_toml = desc.path / "pyproject.toml"
        pyproject = toml.loads(pyproject_toml.read_text())

        # Parses dependencies to other colcon packages indicated in the pyproject.toml file.
        if self._PACKAGE_SECTION in pyproject and self._BUILD_DEPEND_LIST in pyproject[self._PACKAGE_SECTION]:
            build_depend = set(pyproject[self._PACKAGE_SECTION][self._BUILD_DEPEND_LIST])
        else:
            build_depend = set()

        if self._PACKAGE_SECTION in pyproject and self._EXEC_DEPEND_LIST in pyproject[self._PACKAGE_SECTION]:
            exec_depend = set(pyproject[self._PACKAGE_SECTION][self._EXEC_DEPEND_LIST])
        else:
            exec_depend = set()

        if self._PACKAGE_SECTION in pyproject and self._TEST_DEPEND_LIST in pyproject[self._PACKAGE_SECTION]:
            test_depend = set(pyproject[self._PACKAGE_SECTION][self._TEST_DEPEND_LIST])
        else:
            test_depend = set()

        # Depend add the deps to the build and exec depends
        if self._PACKAGE_SECTION in pyproject and self._DEPEND_LIST in pyproject[self._PACKAGE_SECTION]:
            depends = pyproject[self._PACKAGE_SECTION][self._DEPEND_LIST]
            build_depend.update(depends)
            exec_depend.update(depends)

        desc.dependencies[self._PACKAGE_BUILD_CATEGORY] = set(
            create_dependency_descriptor(dep) for dep in build_depend
        )
        desc.dependencies[self._PACKAGE_EXEC_CATEGORY] = set(
            create_dependency_descriptor(dep) for dep in exec_depend
        )
        desc.dependencies[self._PACKAGE_TEST_CATEGORY] = set(
            create_dependency_descriptor(dep) for dep in test_depend
        )
