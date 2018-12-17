"""
Test cases for the ExcaliburDetector class of the ODIN server EXCALIBUR plugin

Tim Nicholls, STFC Application Engineering Group
"""

from nose.tools import *
import logging
from mock import Mock

from excalibur.hl_detector import HLExcaliburDetector, ExcaliburParameter
from excalibur.fem import ExcaliburFem


class TestExcaliburDetector():

    @classmethod
    def setup_class(cls):
        ExcaliburFem.use_stub_api = True
        HLExcaliburDetector.test_mode = True
        cls.detector_fems = [
            ('192.168.0.1', 6969, '10.0.2.1'),
            ('192.168.0.2', 6969, '10.0.2.1'),
            ('192.168.0.3', 6969, '10.0.2.1')
        ]

        cls.detector = HLExcaliburDetector(cls.detector_fems)
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.DEBUG)

    def test_detector_simple_init(self):
        assert_equal(len(self.detector.fems), len(self.detector_fems))

    def test_set_calibration_status(self):
        self.detector.set_calibration_status(1, 1, 'dac')
        assert_equal(self.detector._status['calibration'][0], 1)
        self.detector.set_calibration_status(2, 1)
        assert_equal(self.detector._status['calibration'][1], 63)

    def test_execute_command(self):
        self.detector.hl_initialise = Mock()
        self.detector.execute_command({'path': 'command/initialise', 'data': None})
        self.detector.hl_initialise.assert_called()
        self.detector.update_calibration = Mock()
        self.detector.execute_command({'path': 'command/force_calibrate', 'data': None})
        self.detector.update_calibration.assert_called()
        self.detector.hl_manual_dac_calibration = Mock()
        self.detector.execute_command({'path': 'command/configure_dac', 'data': None})
        self.detector.hl_manual_dac_calibration.assert_called()
        self.detector.hl_test_mask_calibration = Mock()
        self.detector.execute_command({'path': 'command/configure_mask', 'data': None})
        self.detector.hl_test_mask_calibration.assert_called()


    def test_init_hardware(self):
        self.detector.wait_for_completion = Mock(return_value=(True, ''))
        self.detector.write_fe_param = Mock()
        self.detector.init_hardware_values()
        self.detector.write_fe_param.assert_called_with([ExcaliburParameter(param='mpx3_gainmode', value=[[0]], fem=0, chip=0)])

