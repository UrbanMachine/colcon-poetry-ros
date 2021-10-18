# colcon-poetry-ros

An extension for [colcon-core][colcon-core] that adds support for Python
packages that use [Poetry][poetry] within ROS. This extension is a replacement
for Colcon's built-in `setup.py` based Python support and the Python-related
bits in `colcon-ros`.

Testing is done with Foxy, but other versions may work.

## Getting Started

Start by install this extension with pip:

```bash
pip3 install git+ssh://git@github.com/UrbanMachine/colcon-poetry-ros
```

Then, add a `pyproject.toml` in the root of your package's directory. Each
package should have its own `pyproject.toml` file. It should look something
like this:

```toml
[tool.poetry]
name = "my_package"
version = "0.1.0"
description = "Does something cool"
authors = ["Urban Machine <info@urbanmachine.build>"]
license = "Proprietary"

[tool.poetry.dependencies]
python = "^3.8"
requests = "^2.26.0"

[tool.poetry.dev-dependencies]
pytest = "^6.2.5"

[tool.poetry.scripts]
node_a = "my_package.node_a:main"
node_b = "my_package.node_b:main"

[tool.colcon-poetry.data-files]
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

## Finding Dependencies

### Build Dependencies

This extension uses the `requires` field in the `build-system` table to source
build dependencies. See [the section in PEP 518][build-system-requires] for
details. This section may only be necessary if you're using a compiler like
Cython that runs during package installation. Since these packages are not
locked by Poetry, the version specifications in the `requires` field will be
passed to ROS as-is.

### Runtime Dependencies

Runtime dependencies are pulled from the `tool.poetry.dependencies` table. See
[Poetry's documentation][tool-poetry-dependencies] for details. Dependency
versions are defined by the `poetry.lock` file.

You can include dev dependencies by setting the
`POETRY_RUN_DEPENDS_INCLUDE_DEV` environment variable to "true". By default,
this value is false.

You can include extras by setting the
`POETRY_RUN_DEPENDS_EXTRAS` environment variable. Multiple extras can be
provided and are separated by commas. By default, no extras are included.

### Test Dependencies

Poetry currently has no way of defining test dependencies, so it's assumed
that dev dependencies are required for tests. They are pulled from the
`tool.poetry.dev-dependencies` table. See
[Poetry's documentation][tool-poetry-dependencies] for details. Dependency
versions are defined by the `poetry.lock` file.

You can exclude dev dependencies by setting the
`POETRY_TEST_DEPENDS_INCLUDE_DEV` environment variable to "false". As
mentioned above, this value is true by default.

You can include extras by setting the
`POETRY_TEST_DEPENDS_EXTRAS` environment variable. Multiple extras can be
provided and are separated by commas. By default, no extras are included.

[poetry]: https://python-poetry.org/
[colcon-core]: https://github.com/colcon/colcon-core
[build-system-requires]: https://www.python.org/dev/peps/pep-0518/#build-system-table
[tool-poetry-dependencies]: https://python-poetry.org/docs/pyproject/#dependencies-and-dev-dependencies

## Data Files

Poetry has only limited support for including data files in an installation,
and the current implementation is not flexible enough to be used with ROS.
Instead, this extension consults a custom section in your pyproject.toml,
called `tool.colcon-poetry.data-files`.

The format is intended to be identical to the `data_files` field used by
[setuptools][setuptools-data-files].

All ROS projects must have, at minimum, these entries in the
`tool.colcon-poetry.data-files` section:

```toml
[tool.colcon-poetry.data-files]
"share/ament_index/resource_index/packages" = ["resource/{package_name}"]
"share/{package_name}" = ["package.xml"]
```

These entries take care of adding the package index marker and package.xml
file to the installation.

[setuptools-data-files]: https://setuptools.pypa.io/en/latest/userguide/datafiles.html
