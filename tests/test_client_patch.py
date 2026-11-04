"""Synthetic payloads only; exercises the pinned client patch without a robot."""
import gzip
import importlib.util
import unittest
from types import SimpleNamespace
from unittest.mock import patch
CLIENT_AVAILABLE = importlib.util.find_spec('roborock') is not None
if CLIENT_AVAILABLE:
    from roborock.protocols.v1_protocol import create_blob_response_decoder
    from roborock.roborock_message import RoborockMessage
    from roborock.protocol import create_mqtt_params
    from roborock.data.v1.v1_clean_modes import water_level_to_code, water_code_to_level
    from roborock.data.v1.v1_containers import StatusV2 as Status


@unittest.skipUnless(CLIENT_AVAILABLE, 'Install the patched client to run these checks')
class ClientPatch(unittest.TestCase):
    def test_water_slider_round_trip(self):
        for level in range(1,31):
            self.assertEqual(water_code_to_level(water_level_to_code(level)),level)
        self.assertIsNone(water_code_to_level(200))
        self.assertEqual(water_level_to_code(80),250)

    def test_robot_bin_status_is_not_dock_bag_status(self):
        self.assertEqual(Status(rss=1).dust_bin_status.name,'not_installed')
        self.assertEqual(Status(rss=2).dust_bin_status.name,'installed')
        self.assertIsNone(Status().dust_bin_status)

    def test_mqtt_override_and_upstream_default(self):
        auth=SimpleNamespace(u='demo-user',k='demo-key',s='demo-secret',r=SimpleNamespace(m='ssl://broker.example:8883'))
        with patch.dict('os.environ',{'RR_MQTT_URL':''}):
            self.assertEqual(create_mqtt_params(auth).host,'broker.example')
        with patch.dict('os.environ',{'RR_MQTT_URL':'mqtt://@broker.internal:1883'}):
            got=create_mqtt_params(auth)
            self.assertEqual((got.host,got.port,got.tls,got.username,got.password),('broker.internal',1883,False,None,None))

    def blob(self,request_id,data):
        packed=gzip.compress(data)
        header=bytearray(24);header[:8]=b'ROBOROCK'
        header[8:12]=request_id.to_bytes(4,'little')
        header[16:18]=(24).to_bytes(2,'little')
        header[20:24]=len(packed).to_bytes(4,'little')
        return bytes(header),packed

    def test_single_blob(self):
        header,packed=self.blob(7,b'artificial obstacle image')
        got=create_blob_response_decoder()(RoborockMessage(protocol=301,payload=header+packed,random=42))
        self.assertEqual((got.request_id,got.data),(7,b'artificial obstacle image'))

    def test_interleaved_fragments_with_missing_gzip_trailer(self):
        decode=create_blob_response_decoder();streams=[]
        for rid in (7,8):
            data=(b'synthetic-image'+bytes([rid]))*1000
            header,packed=self.blob(rid,data);cut=len(packed)//2
            self.assertIsNone(decode(RoborockMessage(protocol=300,payload=header+packed[:cut],random=rid,seq=1)))
            streams.append((rid,data,packed[cut:-8]))
        for rid,data,tail in reversed(streams):
            got=decode(RoborockMessage(protocol=301,payload=tail,random=rid,seq=2))
            self.assertEqual((got.request_id,got.data),(rid,data))
        self.assertIsNone(decode(RoborockMessage(protocol=301,payload=b'orphan fragment',random=100)))


if __name__=='__main__': unittest.main()
