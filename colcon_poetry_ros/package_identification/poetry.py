import toml
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.package_identification import PackageIdentificationExtensionPoint, logger
from colcon_core.plugin_system import satisfies_version


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

        pyproject_toml = desc.path / "pyproject.toml"
        if not pyproject_toml.is_file():
            # Poetry requires a pyproject.toml to function
            return

        if not (desc.path / "package.xml").is_file():
            logger.info(
                f"Ignoring pyproject.toml in {desc.path} because the directory does "
                f"not have a package.xml file. This suggests that it is not a ROS "
                f"package."
            )
            return

        try:
            pyproject = toml.loads(pyproject_toml.read_text())
        except toml.TomlDecodeError as ex:
            raise RuntimeError(
                f"Failed to parse {pyproject_toml} as a TOML file: {ex}"
            )

        if "tool" not in pyproject or "poetry" not in pyproject["tool"]:
            logger.debug(
                f"The {pyproject_toml} file does not have a [tool.poetry] section. "
                f"The file is likely there to instruct a tool other than Poetry."
            )
            return

        logger.info(f"Project {desc.path} appears to be a Poetry ROS project")

        poetry_config = pyproject["tool"]["poetry"]

        if "name" not in poetry_config:
            raise RuntimeError(
                f"Failed to determine Python package name in {desc.path}: The "
                f"[tool.poetry] section must have a 'name' field"
            )

        name = poetry_config["name"]
        if desc.name is not None and desc.name != name:
            raise RuntimeError(
                f"The {pyproject_toml} file indicates that the package name is "
                f"'{name}', but the package name was already set as '{desc.name}'"
            )

        desc.type = "poetry.python"
        desc.name = poetry_config["name"]
