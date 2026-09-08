import logging
import unittest
from types import ModuleType
from unittest.mock import Mock, patch

import underactuated.meshcat_utils as dut


class TestMeshcatUtils(unittest.TestCase):
    def test_display_meshcat_colab_reuses_server(self):
        colab = ModuleType("google.colab")
        colab.output = Mock()
        meshcat = Mock()
        meshcat.port.return_value = 7002
        with patch.dict("sys.modules", {"google.colab": colab}):
            with patch("pydrake.geometry.StartMeshcat") as start:
                dut.DisplayMeshcat(meshcat, height=450)
                start.assert_not_called()
                colab.output.serve_kernel_port_as_iframe.assert_called_once_with(
                    7002, height=450
                )

    def test_start_meshcat_local(self):
        with patch.dict("sys.modules"):
            dut.sys.modules.pop("google.colab", None)
            with patch("pydrake.geometry.StartMeshcat") as start:
                self.assertIs(dut.StartMeshcat(), start.return_value)
                start.assert_called_once_with()

    def test_start_meshcat_colab(self):
        colab = ModuleType("google.colab")
        colab.output = Mock()
        with patch.dict("sys.modules", {"google.colab": colab}):
            with patch("pydrake.geometry.StartMeshcat") as start:
                start.return_value.port.return_value = 7001
                self.assertIs(dut.StartMeshcat(), start.return_value)
                start.assert_called_once_with()
                colab.output.serve_kernel_port_as_window.assert_called_once_with(
                    7001,
                    anchor_text="Open Meshcat in a separate window",
                    skip_warning=True,
                )
                colab.output.serve_kernel_port_as_iframe.assert_not_called()

    def test_start_meshcat_colab_filters_only_startup_url(self):
        colab = ModuleType("google.colab")
        colab.output = Mock()
        logger = logging.getLogger("drake")
        startup = "Meshcat listening for connections at http://localhost:7000"

        def start():
            logger.info(startup)
            logger.info("Other information")
            logger.warning(startup)
            raise RuntimeError("Startup failed")

        filters_before = list(logger.filters)
        with patch.dict("sys.modules", {"google.colab": colab}):
            with patch("pydrake.geometry.StartMeshcat", side_effect=start):
                with self.assertLogs("drake", level="INFO") as logs:
                    with self.assertRaisesRegex(RuntimeError, "Startup failed"):
                        dut.StartMeshcat()
                    logger.info(startup)
        self.assertEqual(
            [(r.levelno, r.getMessage()) for r in logs.records],
            [
                (logging.INFO, "Other information"),
                (logging.WARNING, startup),
                (logging.INFO, startup),
            ],
        )
        self.assertEqual(logger.filters, filters_before)
