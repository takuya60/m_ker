import unittest

from openflex_teleop_ker.ker_stream import CMD_PING, KERStream


class TestKerStream(unittest.TestCase):
    def test_wifi_command_frame_pads_payload_and_adds_checksum(self):
        self.assertEqual(
            KERStream.encode_wifi_command(CMD_PING),
            bytes([0xA5, 0x43, 0x00, 0x00, 0x00, 0x00]),
        )

    def test_wifi_command_frame_preserves_zero_mask_arguments(self):
        self.assertEqual(
            KERStream.encode_wifi_command(bytes([0x04, 0x12, 0x34])),
            bytes([0xA5, 0x43, 0x04, 0x12, 0x34, 0x22]),
        )

    def test_wifi_command_rejects_invalid_payload_length(self):
        for payload in (b'', b'\x00\x01\x02\x03'):
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    KERStream.encode_wifi_command(payload)

    def test_endpoint_descriptions(self):
        self.assertEqual(
            KERStream(transport='usb').endpoint,
            'USB 0x303a:0x4002',
        )
        self.assertEqual(
            KERStream(transport='serial', port='/dev/ttyUSB1', baud=115200).endpoint,
            '/dev/ttyUSB1 @ 115200',
        )
        self.assertEqual(
            KERStream(transport='wifi', wifi_host='192.168.10.20', wifi_port=19090).endpoint,
            '192.168.10.20:19090',
        )


if __name__ == '__main__':
    unittest.main()
