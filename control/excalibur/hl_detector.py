"""
interface_wrapper.py - EXCALIBUR high level API for the ODIN server.

Alan Greer, DLS
"""
import sys
import traceback
import logging
import json
from datetime import datetime
import time
import threading
is_py2 = sys.version[0] == '2'
if is_py2:
    import Queue as queue
else:
    import queue as queue

from excalibur.detector import ExcaliburDetector, ExcaliburDetectorError
from excalibur.calibration_files import DetectorCalibration
from excalibur.definitions import ExcaliburDefinitions
from excalibur.efuse_id_parser import ExcaliburEfuseIDParser
from enum import Enum
from collections import OrderedDict



class ExcaliburParameter(OrderedDict):
    def __init__(self, param, value,
                 fem=ExcaliburDefinitions.ALL_FEMS, chip=ExcaliburDefinitions.ALL_CHIPS):
        super(ExcaliburParameter, self).__init__()
        self['param'] = param
        self['value'] = value
        self['fem'] = fem
        self['chip'] = chip

    def get(self):
        return self.param, self.value, self.fem, self.chip


class ExcaliburReadParameter(OrderedDict):
    def __init__(self, param, fem=ExcaliburDefinitions.ALL_FEMS, chip=ExcaliburDefinitions.ALL_CHIPS):
        super(ExcaliburReadParameter, self).__init__()
        self['param'] = param
        self['fem'] = fem
        self['chip'] = chip

    def get(self):
        return self.param, self.fem, self.chip


class ParameterType(Enum):
    """Enumeration of all available types
    """
    UNKNOWN = 0
    DICT = 1
    LIST = 2
    INT = 3
    DOUBLE = 4
    STRING = 5
    ENUM = 6


class Parameter(object):
    def __init__(self, name, data_type=ParameterType.UNKNOWN, value=None, callback=None, every_time=False):
        self._name = name
        self._datatype = data_type
        self._value = value
        self._callback = callback
        self._every_time = every_time

    @property
    def value(self):
        return self.get()['value']

    def get(self):
        # Create the dictionary of information
        return_value = {'value': self._value,
                        'type': self._datatype.value
                        }
        return return_value

    def set_value(self, value, callback=True):
        changed = False
        if self._value != value:
            self._value = value
            changed = True
        if self._callback is not None:
            if callback:
                if self._every_time:
                    self._callback(self._name, self._value)
                elif changed:
                    self._callback(self._name, self._value)


class EnumParameter(Parameter):
    def __init__(self, name, value=None, allowed_values=None, callback=None, every_time=False):
        super(EnumParameter, self).__init__(name, data_type=ParameterType.ENUM, value=value,
                                            callback=callback, every_time=every_time)
        self._allowed_values = allowed_values

    def get(self):
        # Create the dictionary of information
        return_value = super(EnumParameter, self).get()
        if self._allowed_values is not None:
            return_value['allowed_values'] = self._allowed_values
        return return_value

    @property
    def index(self):
        return self.get()['allowed_values'].index(self.value)


class IntegerParameter(Parameter):
    def __init__(self, name, value=None, limits=None, callback=None, every_time=False):
        super(IntegerParameter, self).__init__(name, data_type=ParameterType.INT, value=value,
                                               callback=callback, every_time=every_time)
        self._limits = limits

    def get(self):
        # Create the dictionary of information
        return_value = super(IntegerParameter, self).get()
        if self._limits is not None:
            return_value['limits'] = self._limits
        return return_value


class DoubleParameter(Parameter):
    def __init__(self, name, value=None, limits=None, callback=None, every_time=False):
        super(DoubleParameter, self).__init__(name, data_type=ParameterType.DOUBLE, value=value,
                                              callback=callback, every_time=every_time)
        self._limits = limits

    def get(self):
        # Create the dictionary of information
        return_value = super(DoubleParameter, self).get()
        if self._limits is not None:
            return_value['limits'] = self._limits
        return return_value


class StringParameter(Parameter):
    def __init__(self, name, value=None, callback=None, every_time=False):
        super(StringParameter, self).__init__(name, data_type=ParameterType.STRING, value=value,
                                              callback=callback, every_time=every_time)


class HLExcaliburDetector(ExcaliburDetector):
    """Wraps the detector class to provide a high level interface.

    """
    test_mode = False

    STATE_IDLE = 0
    STATE_ACQUIRE = 1
    STATE_CALIBRATING = 2

    def __init__(self, fem_connections):
        super(HLExcaliburDetector, self).__init__(fem_connections)

        self._fems = range(1, len(fem_connections)+1)
        logging.debug("Fem conection IDs: %s", self._fems)

        self._default_status = []
        for fem in self._fems:
            self._default_status.append(None)

        # Create the calibration object
        self._cb = DetectorCalibration()

        # Create the Excalibur parameters
        self._param = {
            'api': DoubleParameter('api', 0.1),
            'config/num_images': IntegerParameter('num_images', 1),
            'config/exposure_time': DoubleParameter('exposure_time', 1.0),
            'config/num_test_pulses': IntegerParameter('num_test_pulses', 0),
            'config/scan_dac_num': IntegerParameter('scan_dac_num', 0),
            'config/scan_dac_start': IntegerParameter('scan_dac_start', 0),
            'config/scan_dac_stop': IntegerParameter('scan_dac_stop', 0),
            'config/scan_dac_step': IntegerParameter('scan_dac_step', 0),
            'config/test_pulse_enable': EnumParameter('test_pulse_enable',
                                                      ExcaliburDefinitions.FEM_TEST_PULSE_NAMES[0],
                                                      ExcaliburDefinitions.FEM_TEST_PULSE_NAMES),
            'config/image_mode': EnumParameter('image_mode',
                                               ExcaliburDefinitions.FEM_IMAGEMODE_NAMES[0],
                                               ExcaliburDefinitions.FEM_IMAGEMODE_NAMES),
            'config/operation_mode': EnumParameter('operation_mode',
                                                   ExcaliburDefinitions.FEM_OPERATION_MODE_NAMES[0],
                                                   ExcaliburDefinitions.FEM_OPERATION_MODE_NAMES),
            'config/lfsr_bypass': EnumParameter('lfsr_bypass',
                                                ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_NAMES[0],
                                                ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_NAMES),
            'config/read_write_mode': EnumParameter('read_write_mode',
                                                    ExcaliburDefinitions.FEM_READOUT_MODE_NAMES[0],
                                                    ExcaliburDefinitions.FEM_READOUT_MODE_NAMES),
            'config/disc_csm_spm': EnumParameter('disc_csm_spm',
                                                 ExcaliburDefinitions.FEM_DISCCSMSPM_NAMES[0],
                                                 ExcaliburDefinitions.FEM_DISCCSMSPM_NAMES),
            'config/equalization_mode': EnumParameter('equalization_mode',
                                                      ExcaliburDefinitions.FEM_EQUALIZATION_MODE_NAMES[0],
                                                      ExcaliburDefinitions.FEM_EQUALIZATION_MODE_NAMES),
            'config/trigger_mode': EnumParameter('trigger_mode',
                                                 ExcaliburDefinitions.FEM_TRIGMODE_NAMES[0],
                                                 ExcaliburDefinitions.FEM_TRIGMODE_NAMES),
            'config/trigger_polarity': EnumParameter('trigger_polarity',
                                                     ExcaliburDefinitions.FEM_TRIGPOLARITY_NAMES[1],
                                                     ExcaliburDefinitions.FEM_TRIGPOLARITY_NAMES),
            'config/csm_spm_mode': EnumParameter('csm_spm_mode',
                                                 ExcaliburDefinitions.FEM_CSMSPM_MODE_NAMES[0],
                                                 ExcaliburDefinitions.FEM_CSMSPM_MODE_NAMES,
                                                 callback=self.update_calibration),
            'config/colour_mode': EnumParameter('colour_mode',
                                                ExcaliburDefinitions.FEM_COLOUR_MODE_NAMES[0],
                                                ExcaliburDefinitions.FEM_COLOUR_MODE_NAMES),
            'config/gain_mode': EnumParameter('gain_mode',
                                              ExcaliburDefinitions.FEM_GAIN_MODE_NAMES[0],
                                              ExcaliburDefinitions.FEM_GAIN_MODE_NAMES,
                                              callback=self.hl_set_gain_mode),
            'config/counter_select': IntegerParameter('counter_select', 0),
            'config/counter_depth': EnumParameter('counter_depth',
                                                  '12',
                                                  ['1', '6', '12', '24']),
            'config/cal_file_root': StringParameter('cal_file_root', '', callback=self.update_calibration),
            'config/energy_threshold': DoubleParameter('energy_threshold', 0.0, callback=self.update_calibration),
            'config/udp_file': StringParameter('udp_file', ''),
            'config/hv_bias': DoubleParameter('hv_bias', 0.0, callback=self.hl_hv_bias_set),
            'config/lv_enable': IntegerParameter('lv_enable', 0, callback=self.hl_lv_enable, every_time=True),
            'config/hv_enable': IntegerParameter('hv_enable', 0, callback=self.hl_hv_enable, every_time=True),
            'config/test_dac_file': StringParameter('test_dac_file', ''),
            'config/test_mask_file': StringParameter('test_mask_file', ''),

            #["Normal",
            #                                                                     "Burst",
            #                                                                     "Histogram",
            #                                                                     "DAC Scan",
            #                                                                     "Matrix Read"])
        }
        self._status = {
            'calibrating': 0,
            'calibration': [0] * len(self._fems),
            'lv_enabled': 0,
            'hv_enabled': 0,
            'sensor': {
                'width': ExcaliburDefinitions.X_PIXELS_PER_CHIP * ExcaliburDefinitions.X_CHIPS_PER_FEM,
                'height': ExcaliburDefinitions.Y_PIXELS_PER_CHIP *
                          ExcaliburDefinitions.Y_CHIPS_PER_FEM * len(self._fems),
                'bytes': 0
            },
            'manufacturer': 'DLS/STFC',
            'model': 'Odin [Excalibur2]',
            'error': '',
            'state': HLExcaliburDetector.STATE_IDLE,
            'fe_lv_enable': [None],
            'fe_hv_enable': [None],
            'pwr_p5va_vmon': [None],
            'pwr_p5vb_vmon': [None],
            'pwr_p5v_fem00_imon': [None],
            'pwr_p5v_fem01_imon': [None],
            'pwr_p5v_fem02_imon': [None],
            'pwr_p5v_fem03_imon': [None],
            'pwr_p5v_fem04_imon': [None],
            'pwr_p5v_fem05_imon': [None],
            'pwr_p48v_vmon': [None],
            'pwr_p48v_imon': [None],
            'pwr_p5vsup_vmon': [None],
            'pwr_p5vsup_imon': [None],
            'pwr_humidity_mon': [None],
            'pwr_air_temp_mon': [None],
            'pwr_coolant_temp_mon': [None],
            'pwr_coolant_flow_mon': [None],
            'pwr_p3v3_imon': [None],
            'pwr_p1v8_imonA': [None],
            'pwr_bias_imon': [None],
            'pwr_p3v3_vmon': [None],
            'pwr_p1v8_vmon': [None],
            'pwr_bias_vmon': [None],
            'pwr_p1v8_imonB': [None],
            'pwr_p1v8_vmonB': [None],
            'pwr_coolant_temp_status': [None],
            'pwr_humidity_status': [None],
            'pwr_coolant_flow_status': [None],
            'pwr_air_temp_status': [None],
            'pwr_fan_fault': [None],
            'efuseid_c0': [0] * len(self._fems),
            'efuseid_c1': [0] * len(self._fems),
            'efuseid_c2': [0] * len(self._fems),
            'efuseid_c3': [0] * len(self._fems),
            'efuseid_c4': [0] * len(self._fems),
            'efuseid_c5': [0] * len(self._fems),
            'efuseid_c6': [0] * len(self._fems),
            'efuseid_c7': [0] * len(self._fems),
            'efuse_match': [0] * len(self._fems)
        }
        logging.debug("Status: %s", self._status)
        self._calibration_status = {
            'dac': [0] * len(self._fems),
            'discl': [0] * len(self._fems),
            'disch': [0] * len(self._fems),
            'mask': [0] * len(self._fems),
            'thresh': [0] * len(self._fems)
        }

        self._executing_updates = True
        self._read_efuse_ids = False
        self._acquiring = False
        self._frames_acquired = 0
        self._hw_frames_acquired = 0
        self._acq_frame_count = 0
        self._acq_exposure = 0.0
        self._acq_start_time = datetime.now()
        self._acq_timeout = 0.0
        self._comms_lock = threading.RLock()
        self._param_lock = threading.RLock()
        self._fast_update_time = datetime.now()
        self._medium_update_time = datetime.now()
        self._slow_update_time = datetime.now()
        self._startup_time = datetime.now()
        self._frame_start_count = 0
        self._frame_count_time = None
        self._calibration_required = True
        self._moly_humidity_counter = 0

        # Temporary 24 bit mode setup
        # TODO: Remove this once 24 bit mode has been implemented within the firmware
        self._24bit_mode = False
        self._24bit_acquiring = False
        self._24bit_params = None
        self._counter_select = 0
        self._acquisition_loops = 0
        # End of 24 bit mode

        if self.test_mode is False:
            # Perform a slow read
            self.slow_read()
            self._lv_toggle_required = False
            with self._param_lock:
                if self._status['lv_enabled'] == 0:
                    # We have started up with the lv not enabled so toggle in case of detector power cycle
                    self._lv_toggle_required = True
            self._status_thread = threading.Thread(target=self.status_loop)
            self._status_thread.start()
            # Create the command handling thread
            self._command_lock = threading.Lock()
            self._command_queue = queue.Queue()
            self._command_thread = threading.Thread(target=self.command_loop)
            self._command_thread.start()
            self.init_hardware_values()

    def init_hardware_values(self):
        gain_mode = self._param['config/gain_mode']
        self.hl_set_gain_mode('config/gain_mode', gain_mode.value)

    def hl_set_gain_mode(self, name, value):
        with self._comms_lock:
            # Initialise the detector parameters
            write_params = []
            gain_mode = self._param['config/gain_mode']
            logging.debug('  Setting ASIC gain mode to {} '.format(gain_mode.value))
            write_params.append(ExcaliburParameter('mpx3_gainmode', [[gain_mode.index]]))
            self.hl_write_params(write_params)
            self.update_calibration(name, value)

    def hl_load_udp_config(self, name, filename):
        logging.debug("Loading UDP configuration [{}] from file {}".format(name, filename))

        try:
            with open(filename) as config_file:
                udp_config = json.load(config_file)
        except IOError as io_error:
            logging.error("Failed to open UDP configuration file: {}".format(io_error))
            self.set_error("Failed to open UDP configuration file: {}".format(io_error))
            return
        except ValueError as value_error:
            logging.error("Failed to parse UDP json config: {}".format(value_error))
            self.set_error("Failed to parse UDP json config: {}".format(value_error))
            return

        source_data_addr = []
        source_data_mac = []
        source_data_port = []
        dest_data_port_offset = []

        for idx, fem in enumerate(udp_config['fems']):
            source_data_addr.append(fem['ipaddr'])
            source_data_mac.append(fem['mac'])
            source_data_port.append(fem['port'])
            dest_data_port_offset.append(fem['dest_port_offset']
                                         )
            logging.debug(
                'FEM  {idx:d} | '
                'ip: {ipaddr:16s} mac: {mac:s} port: {port:5d} offset: {dest_port_offset:d}'.format(
                    idx=idx, **fem)
            )

        udp_params = []
        num_fems = len(self._fems)
        # Append per-FEM UDP source parameters, truncating to number of FEMs present in system
        udp_params.append(ExcaliburParameter(
            'source_data_addr', [[addr] for addr in source_data_addr[:num_fems]],
        ))
        udp_params.append(ExcaliburParameter(
            'source_data_mac', [[mac] for mac in source_data_mac[:num_fems]],
        ))
        udp_params.append(ExcaliburParameter(
            'source_data_port', [[port] for port in source_data_port[:num_fems]]
        ))
        udp_params.append(ExcaliburParameter(
            'dest_data_port_offset',
            [[offset] for offset in dest_data_port_offset[:num_fems]]
        ))

        # These configurations need to be nested once each each for [Detector[FEM[Chip]]]
        if 'all_fems' in udp_config['nodes'].keys():
            # We need to duplicate the same configuration to all FEMs
            dest_data_addr = [[[]]]
            dest_data_mac = [[[]]]
            dest_data_port = [[[]]]
            for dest_idx, dest in enumerate(udp_config['nodes']['all_fems']):
                dest_data_addr[0][0].append(dest['ipaddr'])
                dest_data_mac[0][0].append(dest['mac'])
                dest_data_port[0][0].append(int(dest['port']))

                logging.debug(
                    'Node {node:d} | '
                    'ip: {ipaddr:16s} mac: {mac:s} port: {port:5d}'.format(
                        node=dest_idx, **dest)
                )
        else:
            fems = [fem['name'] for fem in udp_config['fems']]
            if all(fem in udp_config['nodes'].keys() for fem in fems):
                # Each FEM needs a different configuration
                dest_data_addr = [[[]] for _ in self._fems]
                dest_data_mac = [[[]] for _ in self._fems]
                dest_data_port = [[[]] for _ in self._fems]
                for fem_idx, fem_key in enumerate(fems):
                    for dest_idx, dest in enumerate(udp_config['nodes'][fem_key]):
                        dest_data_addr[fem_idx][0].append(dest['ipaddr'])
                        dest_data_mac[fem_idx][0].append(dest['mac'])
                        dest_data_port[fem_idx][0].append(int(dest['port']))

                        logging.debug(
                            'FEM {fem:d} Node {node:d} | '
                            'ip: {ipaddr:16s} mac: {mac:s} port: {port:5d}'.format(
                                fem=fem_idx, node=dest_idx, **dest)
                        )
            else:
                message = "Failed to parse UDP json config." \
                          "Node config must contain a config for each entry in fems or " \
                          "one config with the key 'all_fems'.\n" \
                          "Fems: {}\n" \
                          "Node Config Keys: {}".format(fems, udp_config['nodes'].keys())
                logging.error(message)
                self.set_error(message)
                return

        # Append the UDP destination parameters, noting [[[ ]]] indexing as they are common for
        # all FEMs and chips - there must be a better way to do this
        udp_params.append(ExcaliburParameter(
            'dest_data_addr', dest_data_addr
        ))
        udp_params.append(ExcaliburParameter(
            'dest_data_mac', dest_data_mac
        ))
        udp_params.append(ExcaliburParameter(
            'dest_data_port', dest_data_port
        ))

        farm_mode_enable = udp_config['farm_mode']['enable']
        farm_mode_num_dests = udp_config['farm_mode']['num_dests']

        # Append the farm mode configuration parameters
        udp_params.append(ExcaliburParameter('farm_mode_enable', [[farm_mode_enable]]))
        udp_params.append(ExcaliburParameter('farm_mode_num_dests', [[farm_mode_num_dests]]))

        # Write all the parameters to system
        logging.debug('Writing UDP configuration parameters to system')
        self.hl_write_params(udp_params)
        logging.debug('UDP configuration complete')

    def shutdown(self):
        self._executing_updates = False
        self.queue_command(None)

    def set_calibration_status(self, fem, status, area=None):
        if area is not None:
            self._calibration_status[area][fem-1] = status
        else:
            for area in ['dac', 'discl', 'disch', 'mask', 'thresh']:
                self._calibration_status[area][fem - 1] = status

        logging.debug("Calibration: %s", self._calibration_status)
        bit = 0
        calibration_bitmask = 0
        for area in ['dac', 'discl', 'disch', 'mask', 'thresh']:
            calibration_bitmask += (self._calibration_status[area][fem - 1] << bit)
            bit += 1
        if calibration_bitmask == 0x1F:
            calibration_bitmask += (1 << bit)

        self._status['calibration'][fem-1] = calibration_bitmask

    def hl_manual_dac_calibration(self, filename):
        logging.debug("Manual DAC calibration requested: %s", filename)
        for fem in self._fems:
            self.set_calibration_status(fem, 0, 'dac')
        self._cb.manual_dac_calibration(self._fems, filename)
        self.download_dac_calibration()
        logging.debug("Status: %s", self._status)

    def hl_test_mask_calibration(self, filename):
        logging.debug("Test mask file requested: %s", filename)
        for fem in self._fems:
            self.set_calibration_status(fem, 0, 'mask')
        self._cb.manual_mask_calibration(self._fems, filename)
        self.download_test_masks()
        logging.debug("Status: %s", self._status)

    def update_calibration(self, name, value):
        logging.debug("Update calibration requested due to %s updated to %s", name, value)
        if (datetime.now() - self._startup_time).total_seconds() < 10.0:
            # update_calibration requested too early so flag for an update as soon as possible
            self._calibration_required = True
            logging.debug("Too early in initialisation to calibrate, queued...")
        else:
            lv_enabled = 0
            with self._param_lock:
                lv_enabled = self._status['lv_enabled']
            if lv_enabled == 1:
                try:
                    self._status['calibrating'] = 1
                    self._status['state'] = HLExcaliburDetector.STATE_CALIBRATING
                    logging.info("Calibrating now...")
                    # Reset all calibration status values prior to loading a new calibration
                    for fem in self._fems:
                        self.set_calibration_status(fem, 0)
                    if self._param['config/cal_file_root'].value != '':
                        self._cb.set_file_root(self._param['config/cal_file_root'].value)
                        self._cb.set_csm_spm_mode(self._param['config/csm_spm_mode'].index)
                        self._cb.set_gain_mode(self._param['config/gain_mode'].index)
                        self._cb.set_energy_threshold(self._param['config/energy_threshold'].value)
                        self._cb.load_calibration_files(self._fems)
                        self.download_dac_calibration()
                        self.download_pixel_calibration()
                        logging.debug("Status: %s", self._status)
                    else:
                        logging.debug("No calibration root supplied")
                    self._status['calibrating'] = 0
                    self._status['state'] = HLExcaliburDetector.STATE_IDLE
                except Exception as ex:
                    # If any exception occurs during calibration reset the status item
                    self._status['calibrating'] = 0
                    self._status['state'] = HLExcaliburDetector.STATE_IDLE
                    # Set the error message
                    self.set_error(str(ex))
            else:
                logging.debug("Not updating calibration as LV is not enabled")

    def get_chip_ids(self, fem_id):
        # Return either the default chip IDs or reversed chip IDs depending on the FEM
        # ID.  TODO: 
        chip_ids = ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS
        if fem_id & 1 != 1:
            chip_ids = reversed(chip_ids)
        return chip_ids

    def download_dac_calibration(self):
        dac_params = []

        for (dac_name, dac_param) in self._cb.get_dac(1).dac_api_params():
            logging.debug("%s  %s", dac_name, dac_param)
            dac_vals = []
            for fem in self._fems:
                logging.debug("Downloading FEM # {}".format(fem))
                #fem_vals = [self._cb.get_dac(fem).dacs(fem, chip_id)[dac_name] for chip_id in self.get_chip_ids(fem)]
                fem_vals = [self._cb.get_dac(fem).dacs(fem, chip_id)[dac_name] for chip_id in ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS]
                dac_vals.append(fem_vals)

            dac_params.append(ExcaliburParameter(dac_param, dac_vals,
                                                 fem=self._fems, chip=ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS))

        dac_params.append(ExcaliburParameter('mpx3_dacsense', [[0]],
                                             fem=self._fems, chip=ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS))

        # Write all the parameters to system
        logging.debug('Writing DAC configuration parameters to system {}'.format(str(dac_params)))
        with self._comms_lock:
            self.hl_write_params(dac_params)
            time.sleep(1.0)
            # Now send the command to load the DAC configuration
            self.hl_do_command('load_dacconfig')

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'dac')
            self.set_calibration_status(fem, 1, 'thresh')

    def download_pixel_masks(self):
        pixel_params = []
        mpx3_pixel_masks = []
        logging.debug("Generating mpx3_pixel_mask...")
        for fem in self._fems:
            chip_ids = self.get_chip_ids(fem)
            fem_vals = [self._cb.get_mask(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_masks.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_mask', mpx3_pixel_masks,
                                               fem=self._fems, chip=ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS))

        # Write all the parameters to system
        with self._comms_lock:
            self.hl_write_params(pixel_params)
            time.sleep(1.0)
            # Send the command to load the pixel configuration
            self.hl_do_command('load_pixelconfig')

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'mask')

    def download_test_masks(self):
        chip_ids = ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS
        pixel_params = []
        mpx3_pixel_masks = []
        mpx3_pixel_mask = [0] * ExcaliburDefinitions.FEM_PIXELS_PER_CHIP
        mpx3_pixel_discl = [0] * ExcaliburDefinitions.FEM_PIXELS_PER_CHIP
        mpx3_pixel_disch = [0] * ExcaliburDefinitions.FEM_PIXELS_PER_CHIP
        logging.debug("Generating mpx3_pixel_test...")
        for fem in self._fems:
            fem_vals = [self._cb.get_mask(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_masks.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_mask', [[mpx3_pixel_mask]],
                                               fem=self._fems, chip=chip_ids))
        pixel_params.append(ExcaliburParameter('mpx3_pixel_discl', [[mpx3_pixel_discl]],
                                               fem=self._fems, chip=chip_ids))
        pixel_params.append(ExcaliburParameter('mpx3_pixel_disch', [[mpx3_pixel_disch]],
                                               fem=self._fems, chip=chip_ids))
        pixel_params.append(ExcaliburParameter('mpx3_pixel_test', mpx3_pixel_masks,
                                               fem=self._fems, chip=chip_ids))

        # Write all the parameters to system
        with self._comms_lock:
            self.hl_write_params(pixel_params)
            time.sleep(1.0)

            # Send the command to load the pixel configuration
            self.hl_do_command('load_pixelconfig')

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'mask')

    def download_pixel_calibration(self):
        #chip_ids = ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS
        pixel_params = []
        mpx3_pixel_masks = []
        # Write all the parameters to system
        logging.debug("Writing pixel parameters to hardware...")

        logging.debug("Generating mpx3_pixel_mask...")
        for fem in self._fems:
            chip_ids = self.get_chip_ids(1)
            fem_vals = [self._cb.get_mask(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_masks.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_mask', mpx3_pixel_masks,
                                               fem=self._fems, chip=ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS))

        with self._comms_lock:
            self.hl_write_params(pixel_params)

            time.sleep(1.0)

            # Send the command to load the pixel configuration
            logging.debug("Sending the load_pixelconfig command...")
            self.hl_do_command('load_pixelconfig')

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'mask')

        pixel_params = []
        mpx3_pixel_discl = []
        logging.debug("Generating mpx3_pixel_discl...")
        for fem in self._fems:
            chip_ids = self.get_chip_ids(1)
            fem_vals = [self._cb.get_discL(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_discl.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_discl', mpx3_pixel_discl,
                                               fem=self._fems, chip=ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS))

        with self._comms_lock:
            self.hl_write_params(pixel_params)

            time.sleep(1.0)

            # Send the command to load the pixel configuration
            logging.debug("Sending the load_pixelconfig command...")
            self.hl_do_command('load_pixelconfig')

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'discl')

        pixel_params = []
        mpx3_pixel_disch = []
        logging.debug("Generating mpx3_pixel_disch...")
        for fem in self._fems:
            chip_ids = self.get_chip_ids(1)
            fem_vals = [self._cb.get_discH(fem)[chip - 1].pixels for chip in chip_ids]
            mpx3_pixel_disch.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_disch', mpx3_pixel_disch,
                                               fem=self._fems, chip=ExcaliburDefinitions.FEM_DEFAULT_CHIP_IDS))

        with self._comms_lock:
            self.hl_write_params(pixel_params)
        
            time.sleep(1.0)

            # Send the command to load the pixel configuration
            logging.debug("Sending the load_pixelconfig command...")
            self.hl_do_command('load_pixelconfig')

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'disch')

    def status_loop(self):
        # Status loop has two polling rates, fast and slow
        # Fast poll is currently set to 0.2 s
        # Slow poll is currently set to 5.0 s
        if self._lv_toggle_required:
            # Short pause to ensure the power card ID has been set from the low level detector
            time.sleep(1.0)
            # We only ever toggle the lv once if required
            self._lv_toggle_required = False
            # Perform the toggling of the command bit for lv
            self.hl_toggle_lv()

        while self._executing_updates:
            if (datetime.now() - self._startup_time).total_seconds() > 10.0:
                if self._calibration_required:
                    try:
                        self._calibration_required = False
                        self.update_calibration('lv_enabled', '1')
                    except:
                        pass
            if (datetime.now() - self._slow_update_time).seconds > 10.0:
                self._slow_update_time = datetime.now()
                self.slow_read()
            if (datetime.now() - self._medium_update_time).seconds > 10.0:
                self._medium_update_time = datetime.now()
                self.power_card_read()
            if (datetime.now() - self._fast_update_time).microseconds > 100000:
                self._fast_update_time = datetime.now()
                self.fast_read()
            time.sleep(0.1)

    def queue_command(self, command):
        #if self._command_lock.acquire(False):
        self._command_queue.put(command, block=False)
        #    self._command_lock.release()
        #else:
        #    self.set_error("Cannot submit command whilst another is active")

    def command_loop(self):
        running = True
        while running:
            try:
                command = self._command_queue.get()
                if command:
                    with self._command_lock:
                        self.execute_command(command)
                else:
                    running = False
            except Exception as e:
                type_, value_, traceback_ = sys.exc_info()
                ex = traceback.format_exception(type_, value_, traceback_)
                logging.error(e)
                self.set_error("Unhandled exception: {} => {}".format(str(e), str(ex)))

    def execute_command(self, command):
        path = command['path']
        data = command['data']
        try:
            if path in self._param:
                self._param[path].set_value(data)
            elif path == 'command/initialise':
                # Initialise the FEMs
                logging.debug('Initialise has been called')
                self.hl_initialise()
            elif path == 'command/force_calibrate':
                self.update_calibration('reload', 'manual')
            elif path == 'command/configure_dac':
                # Configure the DAC
                dac_file = self._param['config/test_dac_file'].value
                logging.debug('Manual DAC calibration has been called with file: %s', dac_file)
                self.hl_manual_dac_calibration(dac_file)
            elif path == 'command/configure_mask':
                # Apply a test maks
                mask_file = self._param['config/test_mask_file'].value
                logging.debug('Manual mask file download has been called with file: %s', mask_file)
                self.hl_test_mask_calibration(mask_file)
            elif path == 'command/24bit_acquire':
                # Perform a 24 bit acquisition loop
                self.hl_do_24bit_acquisition()
#            elif path == 'command/start_acquisition':
#                # Starting an acquisition!
#                logging.debug('Start acquisition has been called')
#                self.hl_arm_detector()
#                self.do_acquisition()
#            elif path == 'command/stop_acquisition':
#                # Starting an acquisition!
#                logging.debug('Abort acquisition has been called')
#                self.hl_stop_acquisition()
            else:
                super(HLExcaliburDetector, self).set(path, data)
        except Exception as ex:
            self.set_error(str(ex))
            raise ExcaliburDetectorError(str(ex))

    def get(self, path):
        with self._param_lock:
            if path == 'command/initialise':
                response = {'value': 1}
            elif path == 'command/force_calibrate':
                response = {'value': 1}
            elif path == 'command/configure_dac':
                response = {'value': 1}
            elif path == 'command/configure_mask':
                response = {'value': 1}
            elif path in self._param:
                response = self._param[path].get()
            elif self.search_status(path) is not None:
                response = {'value': self.search_status(path)}
                try:
                    response.update(super(HLExcaliburDetector, self).get(path))
                except:
                    # Valid to fail if the get request is for a high level item
                    pass
            else:
                response = super(HLExcaliburDetector, self).get(path)

            return response

    def set(self, path, data):
        self.clear_error()
        try:
            if path == 'command/start_acquisition':
                # Starting an acquisition!
                logging.debug('Start acquisition has been called')
                self.hl_arm_detector()
                self.do_acquisition()
            elif path == 'command/stop_acquisition':
                # Starting an acquisition!
                logging.debug('Abort acquisition has been called')
                self.hl_stop_acquisition()
            else:
                self.queue_command({'path': path, 'data': data})
        except Exception as ex:
            self.set_error(str(ex))
            raise ExcaliburDetectorError(str(ex))

    def set_error(self, err):
        # Record the error message into the status
        self._status['error'] = err

    def clear_error(self):
        # Record the error message into the status
        self._status['error'] = ""

    def search_status(self, path):
        items = path.split('/')
        item_dict = None
        if items[0] == 'status':
            try:
                item_dict = self._status
                for item in items[1:]:
                    item_dict = item_dict[item]
            except KeyError as ex:
                item_dict = None
        return item_dict

    def fast_read(self):
        status = {}
        with self._param_lock:
            bit_depth = self._param['config/counter_depth'].value
            bps = 1
            if bit_depth == '12':
                bps = 2
            elif bit_depth == '24':
                bps = 4
            self._status['sensor']['bytes'] = self._status['sensor']['width'] * self._status['sensor']['height'] * bps

        frame_rate = 0.0
        if not self._24bit_mode:
            with self._comms_lock:
                acq_completion_state_mask = 0x40000000
                # Connect to the hardware
                if not self.connected:
                    self.connect({'state': True})

                fem_params = ['frames_acquired', 'control_state']

                read_params = ExcaliburReadParameter(fem_params)
                self.read_fe_param(read_params)

                while True:
                    time.sleep(0.01)
                    if not self.command_pending():
                        if self._get('command_succeeded'):
                            logging.debug("Command has succeeded")
                        else:
                            logging.debug("Command has failed")
                        break
                vals = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read']['value']
                logging.debug("Raw fast read status: %s", vals)
                # Calculate the minimum number of frames from the fems, as this will be the actual complete frame count
                frames_acquired = min(vals['frames_acquired'])
                self._hw_frames_acquired = frames_acquired
                #acq_completed = all(
                #    [((state & acq_completion_state_mask) == acq_completion_state_mask) for state in vals['control_state']]
                #)
                if self._acquiring:
                    # Record the frames acquired
                    self._frames_acquired = frames_acquired
                    # We are acquiring so check to see if we have the correct number of frames
                    if frames_acquired == self._acq_frame_count:
                        self._acquiring = False
                        # Acquisition has finished so we must send the stop command
                        logging.debug("stop_acquisition called at end of a complete acquisition")
                        self.hl_stop_acquisition()
                    elif frames_acquired > self._acq_frame_count:
                        # There has been an error in the acquisition, we should never have too many frames
                        self._acquiring = False
                        # Acquisition has finished so we must send the stop command
                        logging.debug("stop_acquisition called at end of a complete acquisition")
                        self.hl_stop_acquisition()
                    else:
                        if frames_acquired > 0:
                            if self._frame_count_time is None:
                                self._frame_start_count = frames_acquired
                                self._frame_count_time = datetime.now()
                            # Check to see if we have timed out
                            delta_us = (datetime.now() - self._frame_count_time).microseconds
                            delta_s = (datetime.now() - self._frame_count_time).seconds
                            frame_rate = float(frames_acquired-self._frame_start_count) / (float(delta_s) + (float(delta_us) / 1000000.0))
                        else:
                            self._frame_start_count = 0
                            self._frame_count_time = None
                            frame_rate = 0.0

                        # We can only time out if we are not waiting for triggers
                        if self._param['config/trigger_mode'].index == ExcaliburDefinitions.FEM_TRIGMODE_INTERNAL:
                            delta_t = (datetime.now() - self._acq_start_time).seconds
                            # Work out the worst case for number of expected frames (assuming 25% plus 5 second startup)
                            delta_t -= 5.0
                            if delta_t > 0.0:
                                expected_frames = int(delta_t / (self._acq_exposure * 1.25))
                                logging.debug("We would have expected %d frames by now", expected_frames)
                                if expected_frames > frames_acquired:
                                    #self._acquiring = False
                                    # Acquisition has finished so we must send the stop command
                                    #self.set_error("stop_acquisition called due to a timeout")
                                    logging.debug("stop_acquisition called due to a timeout")
                                    #self.hl_stop_acquisition()

                init_state = []
                for fem_state in self.get('status/fem')['fem']:
                    init_state.append(fem_state['state'])

                status = {'fem_state': init_state,
                          'frames_acquired': self._frames_acquired,
                          'fem_frames': vals['frames_acquired'],
                          'frame_rate': frame_rate,
                          'acquisition_complete': (not self._acquiring)}
            with self._param_lock:
                self._status.update(status)
            logging.debug("Fast update status: %s", status)

    def power_card_read(self):
        with self._comms_lock:
            # Do not perform a slow read if an acquisition is taking place
            if not self._acquiring:
                # Connect to the hardware
                if not self.connected:
                    self.connect({'state': True})

                powercard_params = ['fe_lv_enable',
                                    'fe_hv_enable',
                                    'pwr_p5va_vmon',
                                    'pwr_p5vb_vmon',
                                    'pwr_p5v_fem00_imon',
                                    'pwr_p5v_fem01_imon',
                                    'pwr_p5v_fem02_imon',
                                    'pwr_p5v_fem03_imon',
                                    'pwr_p5v_fem04_imon',
                                    'pwr_p5v_fem05_imon',
                                    'pwr_p48v_vmon',
                                    'pwr_p48v_imon',
                                    'pwr_p5vsup_vmon',
                                    'pwr_p5vsup_imon',
                                    'pwr_humidity_mon',
                                    'pwr_air_temp_mon',
                                    'pwr_coolant_temp_mon',
                                    'pwr_coolant_flow_mon',
                                    'pwr_p3v3_imon',
                                    'pwr_p1v8_imonA',
                                    'pwr_bias_imon',
                                    'pwr_p3v3_vmon',
                                    'pwr_p1v8_vmon',
                                    'pwr_bias_vmon',
                                    'pwr_p1v8_imonB',
                                    'pwr_p1v8_vmonB',
                                    'pwr_coolant_temp_status',
                                    'pwr_humidity_status',
                                    'pwr_coolant_flow_status',
                                    'pwr_air_temp_status',
                                    'pwr_fan_fault']
                fe_params = powercard_params
                read_params = ExcaliburReadParameter(fe_params, fem=self.powercard_fem_idx+1)
                self.read_fe_param(read_params)

                while True:
                    time.sleep(0.1)
                    if not self.command_pending():
                        if self._get('command_succeeded'):
                            logging.debug("Command has succeeded")
                            status = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read'][
                                'value']
                            with self._param_lock:
                                # Check for the current HV enabled state
                                hv_enabled = 0
                                # Greater than hv_bias means the HV is enabled
                                if status['pwr_bias_vmon'][0] > self._param['config/hv_bias'].value - 5.0:
                                    hv_enabled = 1
                                self._status['hv_enabled'] = hv_enabled

                                for param in powercard_params:
                                    if param in status:
                                        val = status[param]
                                        if isinstance(val, list):
                                            self._status[param] = val[0]
                                        else:
                                            self._status[param] = val
                        else:
                            logging.error("Command has failed")
                            with self._param_lock:
                                for param in powercard_params:
                                    self._status[param] = None
                        break
                logging.debug("Power card update status: %s", self._status)

    def slow_read(self):
        status = {}
        with self._comms_lock:
            # Do not perform a slow read if an acquisition is taking place
            if not self._acquiring:
                # Connect to the hardware
                if not self.connected:
                    self.connect({'state': True})

                fem_params = ['fem_local_temp', 'fem_remote_temp', 'moly_temp', 'moly_humidity']
                supply_params = ['supply_p1v5_avdd1', 'supply_p1v5_avdd2', 'supply_p1v5_avdd3', 'supply_p1v5_avdd4',
                                'supply_p1v5_vdd1', 'supply_p2v5_dvdd1']

                fe_params = fem_params + supply_params + ['mpx3_dac_out']

                read_params = ExcaliburReadParameter(fe_params)
                self.read_fe_param(read_params)

                while True:
                    time.sleep(0.1)
                    if not self.command_pending():
                        if self._get('command_succeeded'):
                            logging.debug("Command has succeeded")
                            status = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read'][
                                'value']
                            with self._param_lock:
                                lv_enabled = 1
                                for param in fe_params:
                                    if param in status:
                                        val = []
                                        if param in supply_params:
                                            for item in status[param]:
                                                if item != 1:
                                                    val.append(0)
                                                else:
                                                    val.append(1)
                                        else:
                                            if param == 'moly_temp' or param == 'moly_humidity':
                                                for item in status[param]:
                                                    if item < 0.0:
                                                        val.append(None)
                                                        lv_enabled = 0
                                                    else:
                                                        val.append(item)
                                            else:
                                                val = status[param]
                                        self._status[param] = val
                                # Catch when the lv has been enabled and attempt to re-send calibration
                                # Also do not return the humidity right away as it has a settling time
                                if self._status['lv_enabled'] == 0 and lv_enabled == 1:
                                    self._calibration_required = True
                                    self._moly_humidity_counter = 3
                                if self._moly_humidity_counter > 0:
                                    self._status['moly_humidity'] = self._default_status
                                    self._moly_humidity_counter -= 1
                                self._status['lv_enabled'] = lv_enabled
                        else:
                            logging.debug("Command has failed")
                            with self._param_lock:
                                for param in fe_params:
                                    self._status[param] = self._default_status
                                    logging.error('Command read_fe_param failed on following FEMS:')
                                    fem_error_count = 0
                                    for (idx, fem_id, error_code, error_msg) in self.get_fem_error_state():
                                        if error_code != 0:
                                            logging.error(
                                                '  FEM idx {} id {} : {} : {}'.format(idx, fem_id, error_code, error_msg))
                                            fem_error_count += 1
                                    err_msg = 'Command read_fe_param failed on {} FEMs'.format(fem_error_count)
                                    self.set_error(err_msg)

                                    #if param in status:
                                    #    self._status[param] = status[param]
                        break

                if not self._read_efuse_ids:
                    # Only read the efuse IDs if the LV is enabled
                    if self._status['lv_enabled'] == 1:
                        response_status, efuse_dict = self.hl_efuseid_read()
                        self._status.update(efuse_dict)
                        logging.debug("EFUSE return status: %s", response_status)
                        if response_status == 0:
                            self._read_efuse_ids = True

                logging.debug("Slow update status: %s", self._status)

    def hl_arm_detector(self):
        # Perform all of the actions required to get the detector ready for an acquisition
        with self._comms_lock:
            self.clear_error()

            # Start by downloading the UDP configuration
            self.hl_load_udp_config('arming', self._param['config/udp_file'].value)


    def hl_do_dac_scan(self):

        logging.info("Executing DAC scan ...")

        # Build a list of parameters to be written toset up the DAC scan
        scan_params = []

        scan_dac = self._param['config/scan_dac_num'].value
        logging.info('  Setting scan DAC to {}'.format(scan_dac))
        scan_params.append(ExcaliburParameter('dac_scan_dac', [[scan_dac]]))

        scan_start = self._param['config/scan_dac_start'].value
        logging.info('  Setting scan start value to {}'.format(scan_start))
        scan_params.append(ExcaliburParameter('dac_scan_start', [[scan_start]]))

        scan_stop = self._param['config/scan_dac_stop'].value
        logging.info('  Setting scan stop value to {}'.format(scan_stop))
        scan_params.append(ExcaliburParameter('dac_scan_stop', [[scan_stop]]))

        scan_step = self._param['config/scan_dac_step'].value
        logging.info('  Setting scan step size to {}'.format(scan_step))
        scan_params.append(ExcaliburParameter('dac_scan_step', [[scan_step]]))

        # Record the acquisition exposure time
        self._acq_exposure = self._param['config/exposure_time'].value

        acquisition_time = int(self._param['config/exposure_time'].value * 1000.0)
        logging.info('  Setting acquisition time to {} ms'.format(acquisition_time))
        scan_params.append(ExcaliburParameter('acquisition_time', [[acquisition_time]]))


        readout_mode = ExcaliburDefinitions.FEM_READOUT_MODE_SEQUENTIAL
        logging.info('  Setting ASIC readout mode to {}'.format(
            ExcaliburDefinitions.readout_mode_name(readout_mode)
        ))
        scan_params.append(ExcaliburParameter('mpx3_readwritemode', [[readout_mode]]))

        colour_mode = self._param['config/colour_mode']
        logging.info('  Setting ASIC colour mode to {} '.format(colour_mode.value))
        scan_params.append(ExcaliburParameter('mpx3_colourmode', [[colour_mode.index]]))

        csmspm_mode = self._param['config/csm_spm_mode']
        logging.info('  Setting ASIC pixel mode to {} '.format(csmspm_mode.value))
        scan_params.append(ExcaliburParameter('mpx3_csmspmmode', [[csmspm_mode.index]]))

        disc_csm_spm = self._param['config/disc_csm_spm']
        logging.info('  Setting ASIC discriminator output mode to {} '.format(disc_csm_spm.value))
        scan_params.append(ExcaliburParameter('mpx3_disccsmspm', [[disc_csm_spm.index]]))

        equalization_mode = self._param['config/equalization_mode']
        logging.info('  Setting ASIC equalization mode to {} '.format(equalization_mode.value))
        scan_params.append(ExcaliburParameter('mpx3_equalizationmode', [[equalization_mode.index]]))

        gain_mode = self._param['config/gain_mode']
        logging.info('  Setting ASIC gain mode to {} '.format(gain_mode.value))
        scan_params.append(ExcaliburParameter('mpx3_gainmode', [[gain_mode.index]]))

        counter_select = self._param['config/counter_select'].value
        logging.info('  Setting ASIC counter select to {} '.format(counter_select))
        scan_params.append(ExcaliburParameter('mpx3_counterselect', [[counter_select]]))

        counter_depth = self._param['config/counter_depth'].value
        logging.info('  Setting ASIC counter depth to {} bits'.format(counter_depth))
        scan_params.append(ExcaliburParameter('mpx3_counterdepth',
                                               [[ExcaliburDefinitions.FEM_COUNTER_DEPTH_MAP[counter_depth]]]))

        operation_mode = ExcaliburDefinitions.FEM_OPERATION_MODE_DACSCAN
        logging.info('  Setting operation mode to {}'.format(
            ExcaliburDefinitions.operation_mode_name(operation_mode)
        ))
        scan_params.append(ExcaliburParameter('mpx3_operationmode', [[operation_mode]]))

        lfsr_bypass_mode = ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_DISABLED
        logging.info('  Setting LFSR bypass mode to {}'.format(
            ExcaliburDefinitions.lfsr_bypass_mode_name(lfsr_bypass_mode)
        ))
        scan_params.append(ExcaliburParameter('mpx3_lfsrbypass', [[lfsr_bypass_mode]]))

        logging.info('  Disabling local data receiver thread')
        scan_params.append(ExcaliburParameter('datareceiver_enable', [[0]]))

        # Write all the parameters to system
        logging.debug('Writing configuration parameters to system {}'.format(str(scan_params)))
        self.hl_write_params(scan_params)

        self._frame_start_count = 0
        self._frame_count_time = None

        # Send start acquisition command
        logging.debug('Sending start acquisition command')
        self.hl_start_acquisition()
        logging.debug('Start acquisition completed')

    def do_acquisition(self):
        with self._comms_lock:
            self.clear_error()
            if self._hw_frames_acquired > 0:
                # Counters have not cleared yet, send a stop acquisition before restarting
                self.hl_stop_acquisition()

            # Set the acquiring flag
            self._acquiring = True
            self._acq_start_time = datetime.now()
            status = {'acquisition_complete': (not self._acquiring)}
            self._status.update(status)
            # Resolve the acquisition operating mode appropriately, handling burst and matrix read if necessary
            operation_mode = self._param['config/operation_mode']

            # Check if the operational mode is DAC scan.
            if operation_mode.index == ExcaliburDefinitions.FEM_OPERATION_MODE_DACSCAN:
                logging.debug('DAC scan requested so entering DAC scan mode')
                self.hl_do_dac_scan()
                return

            # if self.args.burst_mode:
            #     operation_mode = ExcaliburDefinitions.FEM_OPERATION_MODE_BURST
            #
            # if self.args.matrixread:
            #     if self.args.burst_mode:
            #         logging.warning('Cannot select burst mode and matrix read simultaneously, ignoring burst option')
            #     operation_mode = ExcaliburDefinitions.FEM_OPERATION_MODE_MAXTRIXREAD
            #

            num_frames = self._param['config/num_images'].value
            image_mode = self._param['config/image_mode'].value
            logging.info('  Image mode set to {}'.format(image_mode))
            # Check for single image mode
            if image_mode == ExcaliburDefinitions.FEM_IMAGEMODE_NAMES[0]:
                # Single image mode requested, set num frames to 1
                logging.info('  Single image mode, setting number of frames to 1')
                num_frames = 1
            logging.info('  Setting number of frames to {}'.format(num_frames))



            # Temporary 24 bit mode setup
            # TODO: Remove this once 24 bit mode has been implemented within the firmware
            # 24-bit reads are a special case, so set things up appropriately in this mode
            logging.info("config/counter_depth value: {}".format(self._param['config/counter_depth'].value))
            if int(self._param['config/counter_depth'].value) == 24:
                self._24bit_mode = True

                # Force counter select to C1, C0 is read manually afterwards
                self._param['config/counter_select'].set_value(1, callback=False)

                # For acquisitions with > 1 frame, run multiple acquisition loops instea
                self._acquisition_loops = num_frames
                num_frames = 1
                logging.info("Configuring 24-bit acquisition with {} 1-frame loops".format(self._acquisition_loops))

                # In 24-bit mode, force a reset of the UDP frame counter before first acquisition loop
                logging.info('Resetting UDP frame counter for 24 bit mode')
                cmd_ok, err_msg = self.hl_do_command('reset_udp_counter')
                logging.info("{} => {}".format(cmd_ok, err_msg))
                if not cmd_ok:
                    logging.error("UDP counter reset failed: {}".format(err_msg))
                    return
            else:
                self._24bit_mode = False
            # End of 24 bit mode

            # Build a list of parameters to be written to the system to set up acquisition
            write_params = []

            tp_count = self._param['config/num_test_pulses'].value
            logging.info('  Setting test pulse count to {}'.format(tp_count))
            write_params.append(ExcaliburParameter('mpx3_numtestpulses', [[tp_count]]))
            tp_enable = self._param['config/test_pulse_enable']
            logging.info('  Setting test pulse enable to {}'.format(tp_enable.value))
            write_params.append(ExcaliburParameter('testpulse_enable', [[tp_enable.index]]))

            write_params.append(ExcaliburParameter('num_frames_to_acquire', [[num_frames]]))

            # Record the number of frames for this acquisition
            self._acq_frame_count = num_frames

            # Record the acquisition exposure time
            self._acq_exposure = self._param['config/exposure_time'].value

            acquisition_time = int(self._param['config/exposure_time'].value * 1000.0)
            logging.info('  Setting acquisition time to {} ms'.format(acquisition_time))
            write_params.append(ExcaliburParameter('acquisition_time', [[acquisition_time]]))

            trigger_mode = self._param['config/trigger_mode']
            logging.info('  Setting trigger mode to {}'.format(trigger_mode.value))
            write_params.append(ExcaliburParameter('mpx3_externaltrigger', [[trigger_mode.index]]))

            trigger_polarity = self._param['config/trigger_polarity']
            logging.info('  Setting trigger polarity to {}'.format(trigger_polarity.value))
            write_params.append(ExcaliburParameter('mpx3_triggerpolarity', [[trigger_polarity.index]]))

            read_write_mode = self._param['config/read_write_mode']
            logging.info('  Setting ASIC readout mode to {}'.format(read_write_mode.value))
            write_params.append(ExcaliburParameter('mpx3_readwritemode', [[read_write_mode.index]]))

            colour_mode = self._param['config/colour_mode']
            logging.info('  Setting ASIC colour mode to {} '.format(colour_mode.value))
            write_params.append(ExcaliburParameter('mpx3_colourmode', [[colour_mode.index]]))

            csmspm_mode = self._param['config/csm_spm_mode']
            logging.info('  Setting ASIC pixel mode to {} '.format(csmspm_mode.value))
            write_params.append(ExcaliburParameter('mpx3_csmspmmode', [[csmspm_mode.index]]))

            equalization_mode = self._param['config/equalization_mode']
            logging.info('  Setting ASIC equalization mode to {} '.format(equalization_mode.value))
            write_params.append(ExcaliburParameter('mpx3_equalizationmode', [[equalization_mode.index]]))

            gain_mode = self._param['config/gain_mode']
            logging.info('  Setting ASIC gain mode to {} '.format(gain_mode.value))
            write_params.append(ExcaliburParameter('mpx3_gainmode', [[gain_mode.index]]))

            counter_select = self._param['config/counter_select'].value
            logging.info('  Setting ASIC counter select to {} '.format(counter_select))
            write_params.append(ExcaliburParameter('mpx3_counterselect', [[counter_select]]))

            counter_depth = self._param['config/counter_depth'].value
            logging.info('  Setting ASIC counter depth to {} bits'.format(counter_depth))
            write_params.append(ExcaliburParameter('mpx3_counterdepth',
                                                   [[ExcaliburDefinitions.FEM_COUNTER_DEPTH_MAP[counter_depth]]]))

            disc_csm_spm = self._param['config/disc_csm_spm']
            int_counter_depth = ExcaliburDefinitions.FEM_COUNTER_DEPTH_MAP[counter_depth]
            csm_spm_value = ExcaliburDefinitions.DISC_SPM_CSM_TABLE[int_counter_depth][csmspm_mode.index][disc_csm_spm.index][read_write_mode.index][counter_select]
            logging.info('  Setting ASIC discriminator output mode to {} '.format(csm_spm_value))
            write_params.append(ExcaliburParameter('mpx3_disccsmspm', [[csm_spm_value]]))

            logging.info('  Setting operation mode to {}'.format(operation_mode.value))
            write_params.append(ExcaliburParameter('mpx3_operationmode', [[operation_mode.index]]))

            lfsr_bypass = self._param['config/lfsr_bypass']
            logging.info('  Setting LFSR bypass mode to {}'.format(lfsr_bypass.value))
            write_params.append(ExcaliburParameter('mpx3_lfsrbypass', [[lfsr_bypass.index]]))

            #
            # if self.args.matrixread:
            #     lfsr_bypass_mode = ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_ENABLED
            # else:
            #     lfsr_bypass_mode = ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_DISABLED
            #
            #logging.info('  Setting data interface address and port parameters')
            #write_params.append(ExcaliburParameter('source_data_addr', [[addr] for addr in self.source_data_addr]))
            #write_params.append(ExcaliburParameter('source_data_mac', [[mac] for mac in self.source_data_mac]))
            #write_params.append(ExcaliburParameter('source_data_port', [[port] for port in self.source_data_port]))
            #write_params.append(ExcaliburParameter('dest_data_addr', [[addr] for addr in self.dest_data_addr]))
            #write_params.append(ExcaliburParameter('dest_data_mac', [[mac] for mac in self.dest_data_mac]))
            #write_params.append(ExcaliburParameter('dest_data_port', [[port] for port in self.dest_data_port]))

            logging.info('  Disabling local data receiver thread')
            write_params.append(ExcaliburParameter('datareceiver_enable', [[0]]))

            # Connect to the hardware
            # self.connect({'state': True})

            if self._24bit_mode:
                self._24bit_params = write_params
                # Create and queue the command object
                cmd = {
                    'path': 'command/24bit_acquire',
                    'data': {}
                }
                self.queue_command(cmd)

            else:
                # Write all the parameters to system
                logging.info('Writing configuration parameters to system {}'.format(str(write_params)))
                self.hl_write_params(write_params)

                self._frame_start_count = 0
                self._frame_count_time = None

                # Send start acquisition command
                logging.info('Sending start acquisition command')
                self.hl_start_acquisition()
                logging.info('Start acquisition completed')

    def hl_do_24bit_acquisition(self):
        logging.info('24 bit mode acquisition loop entered...')
        for acq_loop in range(self._acquisition_loops):

            self._frame_start_count = 0
            self._frame_count_time = None

            logging.info(
                'Executing acquisition loop {} of {}...'.format(acq_loop + 1, self._acquisition_loops)
            )

            # Write all the parameters to system
            logging.info('Writing configuration parameters to system')
            self.hl_write_params(self._24bit_params)

            # Send start acquisition command
            logging.info('Sending part 1 start acquisition command')
            self.hl_start_acquisition()

            logging.info("Waiting for part 1 acquisition to complete")
            self.wait_for_24bit_acquisition_completion(0x40000000)
            logging.info("Part 1 acquisition has completed")

            self.do_c0_matrix_read()
            logging.info('Acquisition of 24 bit frame completed')

        # Holding the standard acquiring flag true until all loops have completed
        self._acquiring = False
        # Reset 24bit mode flag so that fast read can read
        self._24bit_mode = False
        logging.info("Completed {} acquisition loops".format(self._acquisition_loops))

    def do_c0_matrix_read(self):
        logging.info('Performing a C0 matrix read for 24 bit mode')

        c0_read_params = []
        c0_read_params.append(ExcaliburParameter(
            'mpx3_operationmode', [[ExcaliburDefinitions.FEM_OPERATION_MODE_MAXTRIXREAD]]
        ))
        # Reset counter select back to C0
        self._param['config/counter_select'].set_value(0, callback=False)

        c0_read_params.append(ExcaliburParameter('mpx3_counterselect', [[0]]))
        c0_read_params.append(ExcaliburParameter('num_frames_to_acquire', [[1]]))
        c0_read_params.append(ExcaliburParameter('mpx3_lfsrbypass', [[0]]))

        logging.info("Sending configuration parameters for C0 matrix read")
        self.hl_write_params(c0_read_params)

        logging.info("Sending part 2 start acquisition command")
        self.hl_start_acquisition()

        logging.info("Waiting for part 2 acquisition to complete")
        self.wait_for_24bit_acquisition_completion(0x1f)
        logging.info("Part 2 acquisition has completed")


    def wait_for_24bit_acquisition_completion(self, acq_completion_state_mask):
        fem_params = ['frames_acquired', 'control_state']
        while True:
            read_params = ExcaliburReadParameter(fem_params)
            cmd_ok, err_msg, vals = self.hl_read_params(read_params)

            acq_completed = all(
                [((state & acq_completion_state_mask) == acq_completion_state_mask) for state in vals['control_state']]
            )
            if acq_completed:
                break

        self.hl_stop_acquisition()

    def hl_initialise(self):
        logging.info("Initialising front end...")
        for fem in self._fems:
            self.set_calibration_status(fem, 0)
        logging.info("Sending a fe_vdd_enable param set to 1")
        params = []
        params.append(ExcaliburParameter('fe_vdd_enable', [[1]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)
        logging.info("Sending the fe_init command")
        self.hl_do_command('fe_init')
        logging.info("Sending a stop acquisition")
        return self.hl_stop_acquisition()

    def hl_toggle_lv(self):
        logging.info("Toggling lv_enable 1,0")
        for fem in self._fems:
            self.set_calibration_status(fem, 0)
        if self.powercard_fem_idx < 0:
            self.set_error("Unable to toggle LV enable as server reports no power card")
            return
        params = [ExcaliburParameter('fe_lv_enable', [[1]], fem=self.powercard_fem_idx+1)]
        self.hl_write_params(params)
        params = [ExcaliburParameter('fe_lv_enable', [[0]], fem=self.powercard_fem_idx+1)]
        self.hl_write_params(params)

    def hl_lv_enable(self, name, lv_enable):
        logging.info("Setting lv_enable to %d", lv_enable)
        for fem in self._fems:
            self.set_calibration_status(fem, 0)
        if self.powercard_fem_idx < 0:
            self.set_error("Unable to set LV enable [] as server reports no power card".format(name))
            return
        params = []
        params.append(ExcaliburParameter('fe_lv_enable', [[lv_enable]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)
        if lv_enable == 1:
            self.hl_initialise()

    def hl_hv_enable(self, name, hv_enable):
        logging.info("Setting hv_enable to %d", hv_enable)
        if self.powercard_fem_idx < 0:
            self.set_error("Unable to set HV enable [] as server reports no power card".format(name))
            return
        params = []
        params.append(ExcaliburParameter('fe_hv_enable', [[hv_enable]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)

    def hl_hv_bias_set(self, name, value):
        if self.powercard_fem_idx < 0:
            self.set_error("Unable to set HV bias [] as server reports no power card".format(name))
            return
        params = []
        params.append(ExcaliburParameter('fe_hv_bias', [[float(value)]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)

    def hl_start_acquisition(self):
        with self._comms_lock:
            self.do_command('start_acquisition', None)
            return self.wait_for_completion()

    def hl_stop_acquisition(self):
        with self._comms_lock:
            self._acquiring = False
            self.do_command('stop_acquisition', None)
            return self.wait_for_completion()

    def hl_do_command(self, command):
        logging.debug("Do command: {}".format(command))
        with self._comms_lock:
            self.do_command(command, None)
            return self.wait_for_completion()

    def hl_write_params(self, params):
        logging.debug("Writing params: {}".format(params))
        with self._comms_lock:
            self.write_fe_param(params)
            return self.wait_for_completion()

    def hl_read_params(self, params):
        values = None
        with self._comms_lock:
            self.read_fe_param(params)
            cmd_ok, err_msg = self.wait_for_read_completion()
            if cmd_ok:
                values = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read']['value']
            return (cmd_ok, err_msg, values)

    def hl_efuseid_read(self):
        response_status = 0
        efuse_dict = {'efuseid_c0':  [],
                      'efuseid_c1':  [],
                      'efuseid_c2':  [],
                      'efuseid_c3':  [],
                      'efuseid_c4':  [],
                      'efuseid_c5':  [],
                      'efuseid_c6':  [],
                      'efuseid_c7':  [],
                      'efuse_match': []}
        if self._param['config/cal_file_root'].value != '':
            try:
                # First read out the efuse values from the files
                recorded_efuses = {}
                for fem in self._fems:
                    efid_parser = ExcaliburEfuseIDParser()
                    filename = self._param['config/cal_file_root'].value + "/fem" + str(fem) + '/efuseIDs'
                    efid_parser.parse_file(filename)
                    recorded_efuses[fem] = efid_parser.efuse_ids
                logging.debug("EfuseIDs read from file: %s", recorded_efuses)
                fe_params = ['efuseid']
                read_params = ExcaliburReadParameter(fe_params)
                self.read_fe_param(read_params)

                while True:
                    time.sleep(0.1)
                    if not self.command_pending():
                        if self._get('command_succeeded'):
                            logging.debug("Command has succeeded")
                            status = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read']['value']
                            fem = 1
                            for efuse in status['efuseid']:
                                id_match = 1
                                efuse_dict['efuseid_c0'].append(efuse[0])
                                if recorded_efuses[fem][1] != efuse[0]:
                                    id_match = 0
                                efuse_dict['efuseid_c1'].append(efuse[1])
                                if recorded_efuses[fem][2] != efuse[1]:
                                    id_match = 0
                                efuse_dict['efuseid_c2'].append(efuse[2])
                                if recorded_efuses[fem][3] != efuse[2]:
                                    id_match = 0
                                efuse_dict['efuseid_c3'].append(efuse[3])
                                if recorded_efuses[fem][4] != efuse[3]:
                                    id_match = 0
                                efuse_dict['efuseid_c4'].append(efuse[4])
                                if recorded_efuses[fem][5] != efuse[4]:
                                    id_match = 0
                                efuse_dict['efuseid_c5'].append(efuse[5])
                                if recorded_efuses[fem][6] != efuse[5]:
                                    id_match = 0
                                efuse_dict['efuseid_c6'].append(efuse[6])
                                if recorded_efuses[fem][7] != efuse[6]:
                                    id_match = 0
                                efuse_dict['efuseid_c7'].append(efuse[7])
                                if recorded_efuses[fem][8] != efuse[7]:
                                    id_match = 0
                                efuse_dict['efuse_match'].append(id_match)
                                fem += 1
                        break
            except:
                # Unable to get the efuse IDs so set the dict up with None vales
                response_status = -1
                for efuse_name in efuse_dict:
                    efuse_dict[efuse_name].append(None)
        else:
            response_status = -1
            logging.debug("No EFUSE ID root directory supplied")
        
        logging.debug("EFUSE: %s", efuse_dict)
        return response_status, efuse_dict

    def get_fem_error_state(self):
        fem_state = self.get('status/fem')['fem']
        logging.debug("%s", fem_state)
        for (idx, state) in enumerate(fem_state):
            yield (idx, state['id'], state['error_code'], state['error_msg'])

    def wait_for_completion(self):
        succeeded = False
        err_msg = ''
        try:
            while True:
                time.sleep(0.1)
                if not self.get('status/command_pending')['command_pending']:
                    succeeded = self.get('status/command_succeeded')['command_succeeded']
                    if succeeded:
                        pass
                    else:
                        logging.error('Command write_fe_param failed on following FEMS:')
                        fem_error_count = 0
                        for (idx, fem_id, error_code, error_msg) in self.get_fem_error_state():
                            if error_code != 0:
                                logging.error(
                                    '  FEM idx {} id {} : {} : {}'.format(idx, fem_id, error_code, error_msg))
                                fem_error_count += 1
                        err_msg = 'Command write_fe_param failed on {} FEMs'.format(fem_error_count)
                    break

        except ExcaliburDetectorError as e:
            err_msg = str(e)

        if not succeeded:
            self.set_error(err_msg)

        return succeeded, err_msg

    def wait_for_read_completion(self):
        succeeded = False
        err_msg = ''
        try:
            while True:
                time.sleep(0.1)
                if not self.get('status/command_pending')['command_pending']:
                    succeeded = self.get('status/command_succeeded')['command_succeeded']
                    if succeeded:
                        pass
                    else:
                        logging.error('Command read_fe_param failed on following FEMS:')
                        fem_error_count = 0
                        for (idx, fem_id, error_code, error_msg) in self.get_fem_error_state():
                            if error_code != 0:
                                logging.error(
                                    '  FEM idx {} id {} : {} : {}'.format(idx, fem_id, error_code, error_msg))
                                fem_error_count += 1
                        err_msg = 'Command read_fe_param failed on {} FEMs'.format(fem_error_count)
                    break

        except ExcaliburDetectorError as e:
            err_msg = str(e)

        if not succeeded:
            self.set_error(err_msg)

        return succeeded, err_msg
