from rudra_base_bridge.dcdc_usb_monitor import parse_dcdc_usb_output


def test_parse_dcdc_usb_status_output() -> None:
    output = """
mode: 0 (dumb)
time config: 0
voltage config: 0
state: 1
input voltage: 15.42
ignition voltage: 0.00
output voltage: 12.06
power switch: On
output enable: On
aux vin enable Off
status flags 1: 10
version: 1.8
output voltage: 17.76
"""

    status = parse_dcdc_usb_output(output)

    assert status.mode_number == 0
    assert status.mode_name == 'dumb'
    assert status.state == 1
    assert status.input_voltage == 15.42
    assert status.ignition_voltage == 0.00
    assert status.output_voltage == 12.06
    assert status.programmed_output_voltage == 17.76
    assert status.power_switch_on is True
    assert status.output_enabled is True
    assert status.aux_vin_enabled is False
    assert status.version == '1.8'
