import os
from distutils.util import strtobool
from typing import TypeVar, Generic

T = TypeVar('T')


class _EnvironmentVariable(Generic[T]):
    """Manages configuration from environment variables, handling any parsing necessary
    to convert the variable from a string to the desired type.
    """

    def __init__(self, name: str, default: T):
        """
        :param name: The name of the environment variable
        :param default: The default value if the variable is not set
        """
        self.name = name
        self.default = default

    def get(self) -> T:
        value = os.environ.get(self.name)
        if value is None:
            return self.default

        if isinstance(self.default, list):
            if value.strip() == "":
                return []
            else:
                value_list = value.strip().split(",")
                return [v.strip() for v in value_list]
        elif isinstance(self.default, str):
            return value
        elif isinstance(self.default, bool):
            return strtobool(value)
        else:
            raise NotImplementedError(
                f"Unsupported environment variable type {type(self.default)}"
            )


run_depends_extras = _EnvironmentVariable("POETRY_RUN_DEPENDS_EXTRAS", [])
test_depends_extras = _EnvironmentVariable("POETRY_TEST_DEPENDS_EXTRAS", ["test"])
