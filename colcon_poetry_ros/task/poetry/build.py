import shutil
from pathlib import Path

import toml
from colcon_core.environment import create_environment_hooks, \
    create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import get_command_environment, create_environment_hook
from colcon_core.task import TaskExtensionPoint
from colcon_core.task import run

from colcon_poetry_ros import config


logger = colcon_logger.getChild(__name__)


class PoetryBuildTask(TaskExtensionPoint):
    """Builds Python packages using Poetry"""

    def __init__(self):
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def build(self, *, additional_hooks=None):
        pkg = self.context.pkg
        args = self.context.args

        if pkg.type != "poetry.python":
            logger.error(
                f"The Poetry build was invoked on the wrong package type! Expected "
                f"'poetry.python' but got '{pkg.type}'."
            )

        logger.info(f"Building Poetry Python package in '{args.path}'")

        try:
            env = await get_command_environment(
                "build", args.build_base, self.context.dependencies
            )
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

        poetry_dist = Path(args.build_base) / "poetry_dist"
        if poetry_dist.exists():
            shutil.rmtree(str(poetry_dist))

        shutil.move(
            str(Path(args.path) / "dist"),
            str(poetry_dist)
        )

        # Find the wheel file that Poetry generated
        wheels = list(poetry_dist.glob("*.whl"))
        if len(wheels) == 0:
            logger.error(f"Poetry failed to produce a wheel file in '{poetry_dist}'")
        wheel_name = str(wheels[0])

        # Include any extras that are needed at runtime. Extras are included by adding
        # a bracket-surrounded comma-separated list to the end of the package name, like
        # "colcon-poetry-ros[cool_stuff,other_stuff]"
        extras = config.run_depends_extras.get()
        if len(extras) > 0:
            extras_str = ",".join(extras)
            extras_str = f"[{extras_str}]"
            wheel_name += extras_str

        # Install Poetry's generated wheel
        completed = await run(
            self.context,
            [
                "pip3",
                "install",
                wheel_name,
                # pip will skip installation if the package version is the same
                # but we want the installed version to always reflect the source
                # regardless of the package version
                "--force-reinstall",
                # Turns off Pip's check to ensure installed binaries are in the
                # PATH. ROS workspaces take care of setting the PATH, but Pip
                # doesn't know that.
                "--no-warn-script-location",
                "--no-deps",
                "--prefix",
                args.install_base,
            ],
            cwd=args.path,
            env=env,
        )
        if completed.returncode:
            logger.error(f"Failed to install Poetry's wheel for '{args.path}'")
            return completed.returncode

        # Poetry installs scripts to {prefix}/bin, but ROS wants them at
        # {prefix}/lib/{package_name}
        poetry_script_dir = Path(args.install_base) / "bin"
        if not poetry_script_dir.exists():
            logger.info(f"Attempting to use .../local/bin instead for poetry_script_dir")
            poetry_script_dir = Path(args.install_base) / "local" / "bin"
        ros_script_dir = Path(args.install_base) / "lib" / pkg.name
        if poetry_script_dir.is_dir():
            ros_script_dir.mkdir(parents=True, exist_ok=True)

            script_files = poetry_script_dir.glob("*")
            script_files = filter(Path.is_file, script_files)

            for script in script_files:
                shutil.copy2(str(script), str(ros_script_dir))
            shutil.rmtree(str(poetry_script_dir))
        else:
            logger.warning(
                "Poetry did not install any scripts. Are you missing a "
                "[tool.poetry.scripts] section?"
            )

        return_code = await self._add_data_files()
        if return_code != 0:
            return return_code

        # This hook is normally defined by AmentPythonBuildTask, but since this class
        # replaces that, we have to define it ourselves
        additional_hooks = create_environment_hook(
            "ament_prefix_path",
            Path(args.install_base),
            self.context.pkg.name,
            "AMENT_PREFIX_PATH",
            "",
            mode="prepend",
        )

        hooks = create_environment_hooks(args.install_base, pkg.name)
        create_environment_scripts(
            pkg, args, default_hooks=list(hooks), additional_hooks=additional_hooks
        )

    async def _add_data_files(self) -> int:
        """Installs data files based on the [tool.colcon-poetry-ros.data-files] table.
        Poetry's support for data files is fairly incomplete at the time of writing, so
        we need to do this ourselves.

        See: https://github.com/python-poetry/poetry/issues/2015
        """
        pkg = self.context.pkg
        args = self.context.args

        pyproject_toml = pkg.path / "pyproject.toml"
        pyproject = toml.loads(pyproject_toml.read_text())

        try:
            data_files = pyproject["tool"]["colcon-poetry-ros"]["data-files"]
        except KeyError:
            logger.warning(
                f"File {pyproject_toml} does not define any data files, so none will "
                f"be included in the installation"
            )
            return 0

        if not isinstance(data_files, dict):
            logger.error(f"{_DATA_FILES_TABLE} must be a table")
            return 1

        for destination, sources in data_files.items():
            if not isinstance(sources, list):
                logger.error(
                    f"Field '{destination}' in {_DATA_FILES_TABLE} must be an array"
                )

            dest_path = Path(args.install_base) / destination
            dest_path.mkdir(parents=True, exist_ok=True)

            for source in sources:
                source_path = pkg.path / Path(source)
                _copy_path(source_path, dest_path)

        return 0


def _copy_path(src: Path, dest: Path) -> None:
    """Copies a file or directory to a destination"""
    if src.is_dir():
        # The provided destination is interpreted as a parent directory when copying
        # directories
        dest_dir = dest / src.name
        # shutil.copytree can not completely overwrite a directory, so we need to delete
        # the existing one in advance
        if dest_dir.exists():
            _delete_path(dest_dir)

        shutil.copytree(str(src), str(dest_dir))
    else:
        shutil.copy2(str(src), str(dest))


def _delete_path(path: Path) -> None:
    """Deletes a file or directory"""
    if path.is_file():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(str(path))


_DATA_FILES_TABLE = "[tool.colcon-poetry-ros.data-files]"
