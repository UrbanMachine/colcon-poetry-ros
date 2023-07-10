# colcon-poetry-ros

An extension for [colcon-core][colcon-core] that adds support for Python
packages that use [Poetry][poetry] within ROS. This extension is a replacement
for Colcon's built-in `setup.cfg` based Python support and the Python-related
bits in [colcon-ros][colcon-ros].

We use this extension with Foxy and Humble, but other versions should work as
well. Please create an issue if you see problems!

[colcon-core]: https://github.com/colcon/colcon-core
[poetry]: https://python-poetry.org/
[colcon-ros]: https://github.com/colcon/colcon-ros

## Getting Started

Start by install this extension with Pip:

```bash
pip3 install colcon-poetry-ros
```

Then, add a `pyproject.toml` in the root of your package's directory. Each
package should have its own `pyproject.toml` file. It should look something
like this:

```toml
[tool.poetry]
name = "my_package"
version = "0.1.0"
description = "Does something cool"
authors = ["John Smith <johnny@urbanmachine.build>"]
license = "BSD-3-Clause"

[tool.poetry.dependencies]
python = "^3.8"

[tool.poetry.scripts]
node_a = "my_package.node_a:main"
node_b = "my_package.node_b:main"

[tool.colcon-poetry-ros.data-files]
"share/ament_index/resource_index/packages" = ["resource/my_package"]
"share/my_package" = ["package.xml"]

[build-system]
requires = ["poetry-core>=1.0.0"]
build-backend = "poetry.core.masonry.api"
```

Finally, run your build like normal:

```bash
colcon build
```

## Testing

This extension currently supports projects based on PyTest. Run the following
command to start tests:

```bash
colcon test
```

## Node Entrypoints

If you want to be able to run your nodes using `ros2 run`, add your node's
entrypoint to the `tool.poetry.scripts` table. See
[Poetry's documentation][poetry-scripts] for details.

```toml
[tool.poetry.scripts]
node_a = "my_package.node_a:main"
node_b = "my_package.node_b:main"
```

[poetry-scripts]: https://python-poetry.org/docs/pyproject/#scripts

## Data Files

Poetry has only limited support for including data files in an installation,
and the current implementation is not flexible enough to be used with ROS.
Instead, this extension consults a custom section in your `pyproject.toml`,
called `tool.colcon-poetry-ros.data-files`.

The format is intended to be mostly identical to the `data_files` field used
by [setuptools][setuptools-data-files]. The main differences are that copying
entire directories is supported, and globbing is not yet implemented.

All ROS packages must have, at minimum, these entries in the
`tool.colcon-poetry-ros.data-files` section (with `{package_name}` replaced
with the name of your package):

```toml
[tool.colcon-poetry-ros.data-files]
"share/ament_index/resource_index/packages" = ["resource/{package_name}"]
"share/{package_name}" = ["package.xml"]
```

These entries take care of adding the package index marker and `package.xml`
file to the installation.

[setuptools-data-files]: https://setuptools.pypa.io/en/latest/userguide/datafiles.html

## Installing Dependencies

Poetry dependencies are not installed as part of the build process, but they
can be installed using a separate tool that's included in this package.

```bash
python3 -m colcon_poetry_ros.dependencies.install --base-paths <path to your nodes>
```

This command installs each package's dependencies to Colcon's base install
directory. This means that your dependencies live alongside your package's
code after it's built, isolated from the rest of your system.

If you customize `colcon build` with the `--install-base` or `--merge-install`
flags, make sure to provide those to this tool as well.

## Communicating Dependencies to Colcon

Colcon can be given information on dependencies between packages, which
affects build order and can be displayed in tools like `colcon graph`. These
dependencies can be explicitly defined in the `pyproject.toml` under a custom
section called `tool.colcon-poetry-ros.dependencies`.

```toml
[tool.colcon-poetry-ros.dependencies]
depend = ["foo_package"]   # This will add to both `build_depend` and `exec_depend` following `package.xml` standards
build_depend = ["bar_package"]
exec_depend = ["baz_package"]
test_depend = ["qux_package"]
```
