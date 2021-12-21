from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.package_identification import PackageIdentificationExtensionPoint, logger
from colcon_core.plugin_system import satisfies_version

from colcon_poetry_ros.package import PoetryROSPackage, NotAPoetryROSPackage


class PoetryPackageIdentification(PackageIdentificationExtensionPoint):
    """Identifies Python packages that use Poetry by referencing the pyproject.toml file
    """

    # The priority needs to be higher than RosPackageIdentification and the built-in
    # Python identification. This identifier supercedes both.
    PRIORITY = 200

    def __init__(self):
        super().__init__()
        satisfies_version(
            PackageIdentificationExtensionPoint.EXTENSION_POINT_VERSION,
            "^1.0",
        )

    def identify(self, desc: PackageDescriptor):
        if desc.type is not None and desc.type != "poetry":
            # Some other identifier claimed this package
            return

        try:
            project = PoetryROSPackage(desc.path, logger)
        except NotAPoetryROSPackage:
            return

        if desc.name is not None and desc.name != project.name:
            raise RuntimeError(
                f"The {project.pyproject_file} file indicates that the package name is "
                f"'{project.name}', but the package name was already set as "
                f"'{desc.name}'"
            )

        desc.type = "poetry.python"
        desc.name = project.name
