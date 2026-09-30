"""
test_plugin.py - end-to-end testing of the EXCALIBUR plugin in an ODIN server instance
Tim Nicholls, STFC Application Engineering Group
"""


import requests
import json

from excalibur_detector.control.fem import ExcaliburFem
from excalibur_detector.control.adapter import ExcaliburAdapter
from excalibur_detector.control.testing.test_adapter import ExcaliburAdapterFixture 

class TestExcaliburPlugin(ExcaliburAdapterFixture):
    @classmethod
    def setup_class(cls):
        cls.json_request_headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        adapter_config = {
             'module': 'excalibur.adapter.ExcaliburAdapter',
             'detector_fems': '192.168.0.1:6969, 192.168.0.2:6969, 192.168.0.3:6969',
        }
        super().setup_class(**adapter_config)


    def test_simple_get(self):
        result = self.adapter.get('status/fem', self.request)
        assert result.status_code == 200    


    def test_adapter_connect(self):
        connect_params = {'connect': {'state': True}}
        self.request.body = json.dumps(connect_params)

        result = self.adapter.put('command', self.request)
        assert result.status_code == 200
