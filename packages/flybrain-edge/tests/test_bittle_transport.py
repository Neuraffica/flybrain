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
    # a real Bittle X+Arm (B10_250330) advertised its classic name "BittleC2_SSP" over LE
    assert BleTransport.name_matches("BittleC2_SSP", None)
    assert not BleTransport.name_matches("Bittle", None)
    assert not BleTransport.name_matches("Headphones_BLE", None)
    assert not BleTransport.name_matches(None, None)
    assert BleTransport.name_matches("MyDog_BLE", "mydog")


def test_open_transport_selection():
    assert isinstance(open_transport(mock=True), MockTransport)
    assert isinstance(open_transport(port="/dev/null"), SerialTransport)
    assert isinstance(open_transport(ble=""), BleTransport)
    t = open_transport(ble="Bittle7F")
    assert isinstance(t, BleTransport) and t.want_name == "Bittle7F"


def test_ble_candidates_prefer_uart_service_then_name():
    uart = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
    adverts = [
        ("dev-weak-name", "BittleC2_SSP", [], -80),
        ("dev-uart", "Petoi?", [uart], -90),
        ("dev-strong-name", "BittleA3_BLE", [], -40),
        ("dev-other", "Speaker", ["0000180f-0000-1000-8000-00805f9b34fb"], -30),
        ("dev-noname", None, [], -20),
    ]
    ranked = BleTransport.rank_candidates(adverts, None)
    assert [d for _, d in ranked] == ["dev-uart", "dev-strong-name", "dev-weak-name"]
    only = BleTransport.rank_candidates(adverts, "c2")
    assert [d for _, d in only] == ["dev-weak-name"]


def test_ble_full_name_matches_the_other_suffix_of_the_same_robot():
    # the same robot advertises BittleA7_BLE one day and BittleA7_SSP the next (macOS caching)
    assert BleTransport.name_matches("BittleA7_SSP", "BittleA7_BLE")
    assert BleTransport.name_matches("bittlea7_ble", "BittleA7_SSP")
    assert BleTransport.name_matches("BittleA7_BLE", "BittleA7_BLE")
    # never a different robot, even one whose name contains the stem
    assert not BleTransport.name_matches("BittleC2_SSP", "BittleA7_BLE")
    assert not BleTransport.name_matches("BittleA70_BLE", "BittleA7_BLE")
    assert not BleTransport.name_matches("XBittleA7_SSP", "BittleA7_BLE")
    # a partial name still matches by substring
    assert BleTransport.name_matches("BittleA7_SSP", "A7")
    adverts = [
        ("dev-ssp", "BittleA7_SSP", [], -40),
        ("dev-ble", "BittleA7_BLE", [], -80),
        ("dev-other", "BittleC2_BLE", [], -30),
    ]
    ranked = BleTransport.rank_candidates(adverts, "BittleA7_BLE")
    assert [d for _, d in ranked] == ["dev-ble", "dev-ssp"]  # the exact name first
    assert [d for _, d in BleTransport.rank_candidates(adverts[:1], "BittleA7_BLE")] == ["dev-ssp"]
