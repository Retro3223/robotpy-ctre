import pytest
from unittest.mock import MagicMock

@pytest.fixture(scope='function')
def talon(ctre, MotController):
    return ctre.WPI_TalonSRX(1)


def test_talon_init(ctre):
    ctre.WPI_TalonSRX(1)


def test_talon_get(talon, ControlMode):
    assert talon.get() == 0


def test_talon_set1(talon, ControlMode):
    talon.set(1)
    assert talon.get() == 1
    talon.impl.setDemand.assert_called_with(talon.handle, ControlMode.PercentOutput, 1023, 0)


def test_talon_set2(talon, MotController, ControlMode):
    talon.set(ControlMode.Velocity, 1)
    assert talon.get() != 1
    talon.impl.setDemand.assert_called_with(talon.handle, ControlMode.Velocity, 1, 0)


def test_talon_set3(talon, MotController, ControlMode):
    talon.set(ControlMode.Position, 1, 55)
    assert talon.get() != 1
    talon.impl.setDemand.assert_called_with(talon.handle, ControlMode.Position, 1, 0)


def test_talon_set4(talon, ControlMode):
    talon.set(ControlMode.Current, 1.1)
    talon.impl.setDemand.assert_called_with(talon.handle, ControlMode.Current, 1100, 0)


def test_talon_disable(talon, ControlMode):
    talon.disable()
    talon.impl.setDemand.assert_called_with(talon.handle, ControlMode.Disabled, 0, 0)


def test_talon_stopMotor(talon, ControlMode):
    talon.stopMotor()
    talon.impl.setDemand.assert_called_with(talon.handle, ControlMode.Disabled, 0, 0)


def test_talon_initSendable(talon, ControlMode, sendablebuilder):
    talon.set(4)

    talon.initSendable(sendablebuilder)

    sendablebuilder.updateTable()

    assert sendablebuilder.getTable().getNumber("Value", 0.0) == 4

    sendablebuilder.properties[0].setter(3)

    assert talon.get() == 3


def test_talon_configForwardLimitSwitchSource(talon):
    talon.configForwardLimitSwitchSource(1, 2, 3)
    talon.impl.configForwardLimitSwitchSource.assert_called_with(talon.handle, 1, 2, 3, 0)


def test_talon_configReverseLimitSwitchSource(talon):
    talon.configReverseLimitSwitchSource(1, 2, 3)
    talon.impl.configReverseLimitSwitchSource.assert_called_with(talon.handle, 1, 2, 0, 3)


def test_talon_configPeakCurrentLimit(talon):
    talon.configPeakCurrentLimit(1, 2)
    talon.impl.configPeakCurrentLimit.assert_called_with(talon.handle, 1, 2)


def test_talon_configPeakCurrentDuration(talon):
    talon.configPeakCurrentDuration(1, 2)
    talon.impl.configPeakCurrentDuration.assert_called_with(talon.handle, 1, 2)


def test_talon_configContinuousCurrentLimit(talon):
    talon.configContinuousCurrentLimit(1, 2)
    talon.impl.configContinuousCurrentLimit.assert_called_with(talon.handle, 1, 2)

def test_talon_enableCurrentLimit(talon):
    talon.enableCurrentLimit(True)
    talon.impl.enableCurrentLimit.assert_called_with(talon.handle, True);

def test_basemotorcontroller_changeMotionControlFramePeriod(talon):
    talon.changeMotionControlFramePeriod(1)


def test_basemotorcontroller_clearMotionProfileHasUnderrun(talon):
    talon.clearMotionProfileHasUnderrun(1)


def test_basemotorcontroller_clearMotionProfileTrajectories(talon):
    talon.clearMotionProfileTrajectories()


def test_basemotorcontroller_clearStickyFaults(talon):
    talon.clearStickyFaults(1)


def test_basemotorcontroller_configAllowableClosedloopError(talon):
    talon.configAllowableClosedloopError(1, 2, 3)


def test_basemotorcontroller_configClosedloopRamp(talon):
    talon.configClosedloopRamp(1,2)
    

def test_basemotorcontroller_configForwardSoftLimitEnable(talon):
    talon.configForwardSoftLimitEnable(True, 2)


def test_basemotorcontroller_configForwardSoftLimitThreshold(talon):
    talon.configForwardSoftLimitThreshold(1, 2)


def test_basemotorcontroller_configGetCustomParam(talon):
    talon.configGetCustomParam(1, 2)


def test_basemotorcontroller_configGetParameter(talon):
    talon.configGetParameter(1,2,3)


def test_basemotorcontroller_configMaxIntegralAccumulator(talon):
    talon.configMaxIntegralAccumulator(1,2,3)


def test_basemotorcontroller_configMotionAcceleration(talon):
    talon.configMotionAcceleration(1, 2)


def test_basemotorcontroller_configMotionCruiseVelocity(talon):
    talon.configMotionCruiseVelocity(1,2)


def test_basemotorcontroller_configNeutralDeadband(talon):
    talon.configNeutralDeadband(1,2)


def test_basemotorcontroller_configNominalOutputForward(talon):
    talon.configNominalOutputForward(1, 2)


def test_basemotorcontroller_configNominalOutputReverse(talon):
    talon.configNominalOutputReverse(1,2)


def test_basemotorcontroller_configOpenloopRamp(talon):
    talon.configOpenloopRamp(1,2)


def test_basemotorcontroller_configPeakOutputForward(talon):
    talon.configPeakOutputForward(1,2)


def test_basemotorcontroller_configPeakOutputReverse(talon):
    talon.configPeakOutputReverse(1,2)


def test_basemotorcontroller_configRemoteFeedbackFilter(talon):
    talon.configRemoteFeedbackFilter(1,2,3,4)


def test_basemotorcontroller_configReverseSoftLimitEnable(talon):
    talon.configReverseSoftLimitEnable(True,2)


def test_basemotorcontroller_configReverseSoftLimitThreshold(talon):
    talon.configReverseSoftLimitThreshold(1,2)


def test_basemotorcontroller_configSelectedFeedbackSensor(talon):
    talon.configSelectedFeedbackSensor(1, 2, 3)


def test_basemotorcontroller_configSensorTerm(talon):
    talon.configSensorTerm(1,2,3)


def test_basemotorcontroller_configSetCustomParam(talon):
    talon.configSetCustomParam(1,2,3)


def test_basemotorcontroller_configSetParameter(talon):
    talon.configSetParameter(1,2,3,4,5)


def test_basemotorcontroller_configVelocityMeasurementPeriod(talon):
    talon.configVelocityMeasurementPeriod(1,2)


def test_basemotorcontroller_configVelocityMeasurementWindow(talon):
    talon.configVelocityMeasurementWindow(1,2)


def test_basemotorcontroller_configVoltageCompSaturation(talon):
    talon.configVoltageCompSaturation(1,2)


def test_basemotorcontroller_configVoltageMeasurementFilter(talon):
    talon.configVoltageMeasurementFilter(1,2)


def test_basemotorcontroller_config_IntegralZone(talon):
    talon.config_IntegralZone(1,2,3)


def test_basemotorcontroller_config_kD(talon):
    talon.config_kD(1, 2, 3)


def test_basemotorcontroller_config_kF(talon):
    talon.config_kF(1, 2, 3)


def test_basemotorcontroller_config_kI(talon):
    talon.config_kI(1, 2, 3)


def test_basemotorcontroller_config_kP(talon):
    talon.config_kP(1, 2, 3)


def test_basemotorcontroller_enableHeadingHold(talon):
    talon.enableHeadingHold(True)


def test_basemotorcontroller_enableVoltageCompensation(talon):
    talon.enableVoltageCompensation(True)


def test_basemotorcontroller_follow(talon, ctre):
    master = ctre.WPI_TalonSRX(3)
    talon.follow(master)


def test_basemotorcontroller_getActiveTrajectoryHeading(talon):
    talon.getActiveTrajectoryHeading()


def test_basemotorcontroller_getActiveTrajectoryPosition(talon):
    talon.getActiveTrajectoryPosition()


def test_basemotorcontroller_getActiveTrajectoryVelocity(talon):
    talon.getActiveTrajectoryVelocity()


def test_basemotorcontroller_getBaseID(talon):
    talon.getBaseID()


def test_basemotorcontroller_getBusVoltage(talon):
    talon.getBusVoltage()


def test_basemotorcontroller_getClosedLoopError(talon):
    talon.getClosedLoopError(1)


def test_basemotorcontroller_getControlMode(talon):
    talon.getControlMode()


def test_basemotorcontroller_getDeviceID(talon):
    talon.getDeviceID()

def test_basemotorcontroller_getErrorDerivative(talon):
    talon.getErrorDerivative(1)


def test_basemotorcontroller_getFaults(talon, ctre):
    toFill = ctre.basemotorcontroller.Faults()
    talon.getFaults(toFill)


def test_basemotorcontroller_getFirmwareVersion(talon):
    talon.getFirmwareVersion()


def test_basemotorcontroller_getHandle(talon):
    talon.getHandle()


def test_basemotorcontroller_getIntegralAccumulator(talon):
    talon.getIntegralAccumulator(1)


def test_basemotorcontroller_getLastError(talon):
    talon.getLastError()


def test_basemotorcontroller_getMotionProfileStatus(talon, ctre):
    toFill = ctre.basemotorcontroller.MotionProfileStatus()
    talon.getMotionProfileStatus(toFill)


def test_basemotorcontroller_getMotionProfileTopLevelBufferCount(talon):
    talon.getMotionProfileTopLevelBufferCount()


def test_basemotorcontroller_getMotorOutputPercent(talon):
    talon.getMotorOutputPercent()


def test_basemotorcontroller_getMotorOutputVoltage(talon):
    talon.getMotorOutputVoltage()


def test_basemotorcontroller_getOutputCurrent(talon):
    talon.getOutputCurrent()


def test_basemotorcontroller_getSelectedSensorPosition(talon):
    talon.getSelectedSensorPosition(1)


def test_basemotorcontroller_getSelectedSensorVelocity(talon):
    talon.getSelectedSensorVelocity(1)


def test_basemotorcontroller_getSensorCollection(talon):
    talon.getSensorCollection()


def test_basemotorcontroller_getStatusFramePeriod(talon):
    talon.getStatusFramePeriod(1, 2)


def test_basemotorcontroller_getStickyFaults(talon, ctre):
    toFill = ctre.basemotorcontroller.StickyFaults()
    talon.getStickyFaults(toFill)


def test_basemotorcontroller_getTemperature(talon):
    talon.getTemperature()

def test_basemotorcontroller_hasResetOccurred(talon):
    talon.hasResetOccurred()


def test_basemotorcontroller_isMotionProfileTopLevelBufferFull(talon):
    talon.isMotionProfileTopLevelBufferFull()


def test_basemotorcontroller_neutralOutput(talon):
    talon.neutralOutput()


def test_basemotorcontroller_overrideLimitSwitchesEnable(talon):
    talon.overrideLimitSwitchesEnable(True)


def test_basemotorcontroller_overrideSoftLimitsEnable(talon):
    talon.overrideSoftLimitsEnable(True)


def test_basemotorcontroller_processMotionProfileBuffer(talon):
    talon.processMotionProfileBuffer()


def test_basemotorcontroller_pushMotionProfileTrajectory(talon, ctre):
    point = ctre.basemotorcontroller.TrajectoryPoint()
    talon.pushMotionProfileTrajectory(point)


def test_basemotorcontroller_selectDemandType(talon):
    talon.selectDemandType(True)


def test_basemotorcontroller_selectProfileSlot(talon):
    talon.selectProfileSlot(1, 2)


def test_basemotorcontroller_setControlFramePeriod(talon):
    talon.setControlFramePeriod(1, 2)


def test_basemotorcontroller_setIntegralAccumulator(talon):
    talon.setIntegralAccumulator(1, 2, 3)


def test_basemotorcontroller_setInverted(talon):
    talon.setInverted(True)

    assert talon.getInverted() == True


def test_basemotorcontroller_setNeutralMode(talon):
    talon.setNeutralMode(1)


def test_basemotorcontroller_setSelectedSensorPosition(talon):
    talon.setSelectedSensorPosition(1, 2, 3)


def test_basemotorcontroller_setSensorPhase(talon):
    talon.setSensorPhase(True)


def test_basemotorcontroller_setStatusFramePeriod(talon):
    talon.setStatusFramePeriod(1, 2, 3)


def test_basemotorcontroller_valueUpdated(talon):
    talon.valueUpdated()
