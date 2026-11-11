from flybrain_edge.bittle import BleTransport, MockTransport, SerialTransport, open_transport
from flybrain_edge.bittle.transport import LINUX_PORT_PATTERNS, rank_ports


def test_rank_ports_prefers_usbmodem_and_drops_noise():
    cands = [
        "/dev/cu.Bluetooth-Incoming-Port",
        "/dev/cu.usbserial-0001",
        "/dev/cu.wchusbserial5A7B0012341",
        "/dev/cu.usbmodem5A7B0012341",
        "/dev/cu.debug-console",
        "/dev/cu.SLAB_USBtoUART",
        "/dev/tty.usbmodem5A7B0012341",
    ]
    # the wch twin of the same board is dropped (ardSerial.deleteDuplicatedUsbSerial rule)
    assert rank_ports(cands) == ["/dev/cu.usbmodem5A7B0012341", "/dev/cu.usbserial-0001"]


def test_rank_ports_keeps_wch_without_twin():
    assert rank_ports(["/dev/cu.wchusbserial110", "/dev/cu.usbserial-2"]) == [
        "/dev/cu.wchusbserial110",
        "/dev/cu.usbserial-2",
    ]


def test_rank_ports_linux():
    assert rank_ports(["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyS0"], LINUX_PORT_PATTERNS) == [
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
    ]


def test_ble_name_matching():
    assert BleTransport.name_matches("BittleA3_BLE", None)
    assert not BleTransport.name_matches("BittleA3_SSP", None)
    assert not BleTransport.name_matches(None, None)
    assert BleTransport.name_matches("MyDog_BLE", "mydog")


def test_open_transport_selection():
    assert isinstance(open_transport(mock=True), MockTransport)
    assert isinstance(open_transport(port="/dev/null"), SerialTransport)
    assert isinstance(open_transport(ble=""), BleTransport)
    t = open_transport(ble="Bittle7F")
    assert isinstance(t, BleTransport) and t.want_name == "Bittle7F"
