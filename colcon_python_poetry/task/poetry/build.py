import shutil

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import get_command_environment
from colcon_core.task import TaskExtensionPoint
from colcon_core.task import run


logger = colcon_logger.getChild(__name__)


class PoetryBuildTask(TaskExtensionPoint):
    """Builds Python packages using Poetry"""

    def __init__(self):
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def build(self, *, additional_hooks=None):
        args = self.context.args

        logger.info(f"Building Python package in '{args.path}'")

        try:
            env = await get_command_environment(
                "build", args.build_base, self.context.dependencies)
        except RuntimeError as e:
            logger.error(str(e))
            return 1

        # TODO: Remove this hack when Poetry can install to a target directory
        #       See https://github.com/python-poetry/poetry/issues/1937
        # Unfortunately, Poetry does not support installing to a target directory, so
        # instead we build the package as a wheel, then use Pip to install the wheel
        # to a target directory.
        completed = await run(
            self.context,
            ["poetry", "build", "--format", "wheel"],
            cwd=args.path,
            env=env,
        )
        if completed.returncode:
            logger.error(f"Poetry failed to build the package for '{args.path}'")
            return completed.returncode
        shutil.move(args.path / "dist", args.build_base / "poetry_dist")

        # Find the wheel file that Poetry generated
        wheel_dir = args.build_base / "poetry_dist"
        wheels = list(wheel_dir.glob("*.whl"))
        if len(wheels) == 0:
            logger.error(f"Poetry failed to produce a wheel file in '{wheel_dir}'")
        wheel_path = wheels[0]

        completed = await run(
            self.context,
            ["pip3", "install", str(wheel_path), "--target", args.install_base],
            cwd=args.path,
            env=env,
        )
        if completed.returncode:
            logger.error(f"Failed to install Poetry's wheel for '{args.path}'")
            return completed.returncode
