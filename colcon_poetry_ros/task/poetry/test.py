from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint
from colcon_core.task.python.test import PythonTestTask

logger = colcon_logger.getChild(__name__)


class PoetryTestTask(TaskExtensionPoint):
    """Adds support for testing via PyTest. Luckily we can reuse Colcon's built-in
    support for Python testing as long as we force it to always use PyTest.
    """

    # Use a higher priority than PythonTestTask, since this one replaces that one
    PRIORITY = 250

    def __init__(self):
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def test(self):
        logger.info("Using the Poetry wrapper for PyTest support")

        # Force PythonTestTask to use PyTest, since the alternative is setup.py-based
        # testing
        # TODO: Is this true? It seems like the so-called "setup.py" test task just
        #       invokes unittest, which should work file.
        self.context.args.python_testing = "pytest"

        extension = PythonTestTask()
        extension.set_context(context=self.context)
        return await extension.test()
