# colcon-python-poetry

An extension for [colcon-core][colcon-core] that adds support for Python
packages that use [Poetry][poetry].

## Build Dependencies

This extension uses the `requires` field in the `build-system` table to source
build dependencies. See [the section in PEP 518][build-system-requires] for
details. This section may only be necessary if you're using a compiler like
Cython that runs during package installation.

## Runtime Dependencies

Runtime dependencies are pulled from the `tool.poetry.dependencies` table. See
[Poetry's documentation][tool-poetry-dependencies] for details.

## Test Dependencies

Poetry currently has no way of defining test dependencies, so it's assumed
that dev dependencies are required for tests. They are pulled from the
`tool.poetry.dev-dependencies` table. See
[Poetry's documentation][tool-poetry-dependencies] for details.


[poetry]: https://python-poetry.org/
[colcon-core]: https://github.com/colcon/colcon-core
[build-system-requires]: https://www.python.org/dev/peps/pep-0518/#build-system-table
[tool-poetry-dependencies]: https://python-poetry.org/docs/pyproject/#dependencies-and-dev-dependencies
