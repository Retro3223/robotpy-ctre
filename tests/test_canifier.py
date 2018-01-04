import pytest
from unittest.mock import MagicMock


@pytest.fixture(scope='function')
def canifier(ctre):
    return ctre.CANifier(1)


def test_canifier_init(ctre):
    ctre.CANifier(1)


def test_canifier_setLEDOutput(canifier):
    canifier.setLEDOutput(0.1, 3)


def test_canifier_setGeneralOutput(canifier):
    canifier.setGeneralOutput(2, True, True)


def test_canifier_setGeneralOutputs(canifier):
    canifier.setGeneralOutputs(2, 1)


def test_canifier_getGeneralInputs(ctre, canifier):
    pin_values = ctre.canifier.PinValues()
    canifier.getGeneralInputs(pin_values)


def test_canifier_getGeneralInput(canifier):
    canifier.getGeneralInput(2)


def test_canifier_getLastError(canifier):
    canifier.getLastError()


def test_canifier_setPWMOutput(canifier):
    canifier.setPWMOutput(1, 2)


def test_canifier_enablePWMOutput(canifier):
    canifier.enablePWMOutput(2, True)


def test_canifier_getPWMInput(canifier):
    canifier.getPWMInput(2, [])


def test_canifier_configSetCustomParam(canifier):
    canifier.configSetCustomParam(1, 2, 3)


def test_canifier_configGetCustomParam(canifier):
    canifier.configGetCustomParam(1, 2)


def test_canifier_configSetParameter(canifier):
    canifier.configSetParameter(1, 2, 3, 4, 5)


def test_canifier_configGetParameter(canifier):
    canifier.configGetParameter(1, 2, 3)


def test_canifier_setStatusFramePeriod(canifier):
    canifier.setStatusFramePeriod(1, 2, 3)


def test_canifier_getStatusFramePeriod(canifier):
    canifier.getStatusFramePeriod(1, 2)


def test_canifier_setControlFramePeriod(canifier):
    canifier.setControlFramePeriod(1, 2)


def test_canifier_getFirmwareVersion(canifier):
    canifier.getFirmwareVersion()


def test_canifier_hasResetOccurred(canifier):
    canifier.hasResetOccurred()


def test_canifier_getFaults(canifier):
    toFill = MagicMock()
    canifier.getFaults(toFill)


def test_canifier_getStickyFaults(canifier):
    toFill = MagicMock()
    canifier.getStickyFaults(toFill)


def test_canifier_clearStickyFaults(canifier):
    canifier.clearStickyFaults(1)


def test_canifier_getBusVoltage(canifier):
    canifier.getBusVoltage()


