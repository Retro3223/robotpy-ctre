# validated: 2018-01-06 EN d090f9cdc6a1 java/src/com/ctre/phoenix/motorcontrol/can/BaseMotorController.java
#----------------------------------------------------------------------------
#  Software License Agreement
#
# Copyright (C) Cross The Road Electronics.  All rights
# reserved.
# 
# Cross The Road Electronics (CTRE) licenses to you the right to 
# use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
# API Libraries ONLY when in use with Cross The Road Electronics hardware products.
# 
# THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
# WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
# LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
# CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
# INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
# PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
# BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
# THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
# SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
# (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
#----------------------------------------------------------------------------
from .sensorcollection import SensorCollection
from ._impl import MotController, ControlMode, NeutralMode, RemoteSensorSource, SensorTerm, FeedbackDevice, ControlFrame, StatusFrame, StatusFrameEnhanced, VelocityMeasPeriod, RemoteLimitSwitchSource, LimitSwitchNormal, LimitSwitchSource, TrajectoryPoint, MotionProfileStatus, Faults, StickyFaults, ParamEnum


__all__ = ["BaseMotorController"]


class BaseMotorController:
    """Base motor controller features for all CTRE CAN motor controllers."""

    def __init__(self, arbId: int):
        """Constructor for motor controllers.
        
        :param arbId:
        """
        self.impl = MotController()
        self.handle = self.impl.create1(arbId)
        self.arbId = arbId
        self.sensorColl = SensorCollection(self)
        self.motionProfStats = [0] * 9
        self.controlMode = ControlMode.PercentOutput
        self.sendMode = ControlMode.PercentOutput

    def getHandle(self):
        """:returns: CCI handle for child classes."""
        return self.handle

    def getDeviceID(self):
        """Returns the Device ID
        
        :returns: Device number.
        """
        return self.impl.getDeviceNumber(self.handle)

    def set(self, mode: ControlMode, demand0: float, demand1: float = 0.0):
        """Sets the appropriate output on the talon, depending on the mode.

        :param mode:
            The output mode to apply.
        :param demand0: 
            The output value to apply. such as advanced feed forward and/or cascaded close-looping in firmware.

            In :attr:`.ControlMode.PercentOutput`, the output is between -1.0 and 1.0, with 0.0 as
            stopped. 
            
            In :attr:`.ControlMode.Voltage` mode, output value is in volts. 
            
            In :attr:`.ControlMode.Current` mode, output value is in amperes. 
            
            In :attr:`.ControlMode.Speed` mode, output value is in position change / 100ms. 
            
            In :attr:`.ControlMode.Position` mode, output value is in encoder ticks or an analog value, depending on the sensor. 
            
            In :attr:`.ControlMode.Follower` mode, the output value is the integer device ID of the talon to duplicate.
        :type demand0: float
        :param demand1:
            Supplemental value.  This will also be control mode specific for future features.
        :type demand1: float

        see :meth:`.selectProfileSlot` to choose between the two sets of gains.
        """
        self.controlMode = mode
        self.sendMode = mode

        if self.controlMode == ControlMode.PercentOutput:
            self.impl.setDemand(self.handle, self.sendMode, int(1023 * demand0), 0)
        elif self.controlMode == ControlMode.Follower:
            # did caller specify device ID
            if 0 <= demand0 <= 62:
                work = self.getBaseID()
                work >>= 16
                work <<= 8
                work |= int(demand0) & 0xFF
            else:
                work = int(demand0)
            self.impl.setDemand(self.handle, self.sendMode, work, 0)
        elif self.controlMode in [ControlMode.Velocity, ControlMode.Position, ControlMode.MotionMagic, ControlMode.MotionMagicArc, ControlMode.MotionProfile]:
            self.impl.setDemand(self.handle, self.sendMode, int(demand0), 0)
        elif self.controlMode == ControlMode.Current:
            self.impl.setDemand(self.handle, self.sendMode, int(1000. * demand0), 0) # milliamps 
        else:
            self.impl.setDemand(self.handle, self.sendMode, 0, 0)

    def neutralOutput(self):
        """Neutral the motor output by setting control mode to disabled."""
        self.set(ControlMode.Disabled, 0, 0)

    def setNeutralMode(self, neutralMode: NeutralMode):
        """Sets the mode of operation during neutral throttle output.
        
        :param neutralMode:
            The desired mode of operation when the Controller output
            throttle is neutral (ie brake/coast)
        """
        self.impl.setNeutralMode(self.handle, neutralMode)

    def enableHeadingHold(self, enable: bool):
        """Enables a future feature called "Heading Hold".
        For now this simply updates the CAN signal to the motor controller.
        Future firmware updates will use this.
        
        :param enable: true/false enable
        """
        self.impl.enableHeadingHold(self.handle, int(enable))

    def selectDemandType(self, value: bool):
        """For now this simply updates the CAN signal to the motor controller.
        Future firmware updates will use this to control advanced cascaded loop behavior.
        
        :param value:
        """
        self.impl.selectDemandType(self.handle, int(value))

    def setSensorPhase(self, phaseSensor: bool):
        """Sets the phase of the sensor. Use when controller forward/reverse output
        doesn't correlate to appropriate forward/reverse reading of sensor.

        Pick a value so that positive PercentOutput yields a positive change in sensor.
        After setting this, user can freely call SetInvert() with any value.
        
        
        :param PhaseSensor:
            Indicates whether to invert the phase of the sensor.
        """
        self.impl.setSensorPhase(self.handle, phaseSensor)

    def setInverted(self, invert: bool):
        """
        Inverts the hbridge output of the motor controller.

        This does not impact sensor phase and should not be used to correct sensor polarity.

        This will invert the hbridge output but NOT the LEDs.
        This ensures....

        - Green LEDs always represents positive request from robot-controller/closed-looping mode.
        - Green LEDs correlates to forward limit switch.
        - Green LEDs correlates to forward soft limit.
        
        :param invert:
            Invert state to set.
        """
        self.invert = invert
        self.impl.setInverted(self.handle, invert)

    def getInverted(self):
        """:returns: invert setting of motor output"""
        return self.invert

    def configOpenloopRamp(self, secondsFromNeutralToFull: float, timeoutMs: int):
        """Configures the open-loop ramp rate of throttle output.
        
        :param secondsFromNeutralToFull:
            Minimum desired time to go from neutral to full throttle. A
            value of '0' will disable the ramp.
        :param timeoutMs:
            Timeout value in ms. Function will generate error if config is
            not successful within timeout.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configOpenLoopRamp(self.handle, secondsFromNeutralToFull, timeoutMs)

    def configClosedloopRamp(self, secondsFromNeutralToFull: float, timeoutMs: int):
        """Configures the closed-loop ramp rate of throttle output.
        
        :param secondsFromNeutralToFull:
            Minimum desired time to go from neutral to full throttle. A
            value of '0' will disable the ramp.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configClosedLoopRamp(self.handle, secondsFromNeutralToFull, timeoutMs)

    def configPeakOutputForward(self, percentOut: float, timeoutMs: int):
        """Configures the forward peak output percentage.
        
        :param percentOut:
            Desired peak output percentage [0,1].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configPeakOutputForward(self.handle, percentOut, timeoutMs)

    def configPeakOutputReverse(self, percentOut: float, timeoutMs: int):
        """Configures the reverse peak output percentage.
        
        :param percentOut:
            Desired peak output percentage.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configPeakOutputReverse(self.handle, percentOut, timeoutMs)

    def configNominalOutputForward(self, percentOut: float, timeoutMs: int):
        """Configures the forward nominal output percentage.
        
        :param percentOut:
            Nominal (minimum) percent output [0,+1].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configNominalOutputForward(self.handle, percentOut, timeoutMs)

    def configNominalOutputReverse(self, percentOut: float, timeoutMs: int):
        """Configures the reverse nominal output percentage.
        
        :param percentOut:
            Nominal (minimum) percent output [-1,0].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """ 
        return self.impl.configNominalOutputReverse(self.handle, percentOut, timeoutMs)

    def configNeutralDeadband(self, percentDeadband: float, timeoutMs: int):
        """Configures the output deadband percentage.
        
        :param percentDeadband: 
            Desired deadband percentage. Minimum is 0.1%, Maximum is 25%.
            Pass 0.04 for 4% (factory default).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configNeutralDeadband(self.handle, percentDeadband, timeoutMs)

    def configVoltageCompSaturation(self, voltage: float, timeoutMs: int):
        """Configures the Voltage Compensation saturation voltage.
        
        :param voltage:
            This is the max voltage to apply to the hbridge when voltage
            compensation is enabled.  For example, if 10 (volts) is specified
            and a TalonSRX is commanded to 0.5 (PercentOutput, closed-loop, etc)
            then the TalonSRX will attempt to apply a duty-cycle to produce 5V.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configVoltageCompSaturation(self.handle, voltage, timeoutMs)


    def configVoltageMeasurementFilter(self, filterWindowSamples: int, timeoutMs: int):
        """Configures the voltage measurement filter.
        
        :param filterWindowSamples:
            Number of samples in the rolling average of voltage
            measurement.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configVoltageMeasurementFilter(self.handle, filterWindowSamples, timeoutMs)

    def enableVoltageCompensation(self, enable: bool):
        """Enables voltage compensation. If enabled, voltage compensation works in
        all control modes.
        
        :param enable:
            Enable state of voltage compensation.
        """ 
        self.impl.enableVoltageCompensation(self.handle, enable)

    def getBusVoltage(self):
        """Gets the bus voltage seen by the device.
        
        :returns: The bus voltage value (in volts).
        """
        return self.impl.getBusVoltage(self.handle)

    def getMotorOutputPercent(self):
        """Gets the output percentage of the motor controller.
        
        :returns: Output of the motor controller (in percent).
        """
        return self.impl.getMotorOutputPercent(self.handle)

    def getMotorOutputVoltage(self):
        """:returns: applied voltage to motor in volts"""
        return self.getBusVoltage() * self.getMotorOutputPercent()

    def getOutputCurrent(self):
        """Gets the output current of the motor controller.
        
        :returns: The output current (in amps).
        """
        return self.impl.getOutputCurrent(self.handle)

    def getTemperature(self):
        """Gets the temperature of the motor controller.
        
        :returns: Temperature of the motor controller (in 'C)
        """
        return self.impl.getTemperature(self.handle)

    def configSelectedFeedbackSensor(self, feedbackDevice, pidIdx: int, timeoutMs: int):
        """Select the feedback device for the motor controller.
        Most CTRE CAN motor controllers will support remote sensors over CAN.

        :param feedbackDevice:
            Feedback Device to select.
        :type: feedbackDevice: :class:`.FeedbackDevice` or :class:`.RemoteFeedbackDevice`
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
            See Phoenix-Documentation for how to interpret.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configSelectedFeedbackSensor(self.handle, feedbackDevice, pidIdx, timeoutMs)

    def configRemoteFeedbackFilter(self, deviceID: int, remoteSensorSource: RemoteSensorSource, remoteOrdinal: int,
                        timeoutMs: int):
        """Select what remote device and signal to assign to Remote Sensor 0 or Remote Sensor 1.
        After binding a remote device and signal to Remote Sensor X, you may select Remote Sensor X
        as a PID source for closed-loop features.

        :param deviceID:
            The CAN ID of the remote sensor device.
        :param remoteSensorSource:
            The remote sensor device and signal type to bind.
        :param remoteOrdinal:
            0 for configuring Remote Sensor 0
            1 for configuring Remote Sensor 1
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configRemoteFeedbackFilter(self.handle, deviceID, remoteSensorSource, remoteOrdinal, timeoutMs)

    def configSensorTerm(self, sensorTerm: SensorTerm, feedbackDevice: FeedbackDevice, timeoutMs: int):
        """Select what sensor term should be bound to switch feedback device.

        Sensor Sum = Sensor Sum Term 0 - Sensor Sum Term 1

        Sensor Difference = Sensor Diff Term 0 - Sensor Diff Term 1

        The four terms are specified with this routine.  Then Sensor Sum/Difference
        can be selected for closed-looping.
        
        :param sensorTerm: Which sensor term to bind to a feedback source.
        :param feedbackDevice: The sensor signal to attach to sensorTerm.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configSensorTerm(self.handle, sensorTerm, feedbackDevice, timeoutMs)

    def getSelectedSensorPosition(self, pidIdx: int):
        """Get the selected sensor position.
        
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
            See Phoenix-Documentation for how to interpret.
        :returns: Position of selected sensor (in Raw Sensor Units).
        """
        return self.impl.getSelectedSensorPosition(self.handle, pidIdx)

    def getSelectedSensorVelocity(self, pidIdx: int):
        """Get the selected sensor velocity.
        
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
            See Phoenix-Documentation for how to interpret.
        :returns: selected sensor (in raw sensor units) per 100ms.
        """
        return self.impl.getSelectedSensorVelocity(self.handle, pidIdx)

    def setSelectedSensorPosition(self, sensorPos: int, pidIdx: int, timeoutMs: int):
        """Sets the sensor position to the given value.

        :param sensorPos:
            Position to set for the selected sensor (in raw sensor units).
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setSelectedSensorPosition(self.handle, sensorPos, pidIdx, timeoutMs)

    def setControlFramePeriod(self, frame: ControlFrame, periodMs: int):
        """Sets the period of the given control frame.
        
        :param frame:
            Frame whose period is to be changed.
        :param periodMs:
            Period in ms for the given frame.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setControlFramePeriod(self.handle, frame, periodMs)
        
    def setStatusFramePeriod(self, frame: StatusFrame, periodMs: int, timeoutMs: int):
        """Sets the period of the given status frame.

	User ensure CAN Bus utilization is not high.
	 
	This setting is not persistent and is lost when device is reset. If this
	is a concern, calling application can use HasReset() to determine if the
	status frame needs to be reconfigured.

        :param frame:
            Frame whose period is to be changed.
        :param periodMs:
            Period in ms for the given frame.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setStatusFramePeriod(self.handle, frame, periodMs, timeoutMs)

    def getStatusFramePeriod(self, frame, timeoutMs: int):
        """Gets the period of the given status frame.
        
        :param frame:
            Frame to get the period of.
        :type frame: :class:`.StatusFrame` or :class:`.StatusFrameEnhanced`
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Period of the given status frame.
        """
        return self.impl.getStatusFramePeriod(self.handle, frame, timeoutMs)

    def configVelocityMeasurementPeriod(self, period: VelocityMeasPeriod, timeoutMs: int):
        """Sets the period over which velocity measurements are taken.

        :param period:
            Desired period for the velocity measurement. see :class:`.VelocityMeasPeriod`
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configVelocityMeasurementPeriod(self.handle, period, timeoutMs)

    def configVelocityMeasurementWindow(self, windowSize: int, timeoutMs: int):
        """Sets the number of velocity samples used in the rolling average velocity
        measurement.

        :param windowSize:
            Number of samples in the rolling average of velocity
            measurement. Valid values are 1,2,4,8,16,32. If another value
	    is specified, it will truncate to nearest support value.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configVelocityMeasurementWindow(self.handle, windowSize, timeoutMs)

    def configForwardLimitSwitchSource(self, type: RemoteLimitSwitchSource, normalOpenOrClose: LimitSwitchNormal,
                        deviceID: int, timeoutMs: int):
        """Configures the forward limit switch for a remote source. For example, a
        CAN motor controller may need to monitor the Limit-F pin of another Talon
        or CANifier.

        :param type:
            Remote limit switch source. User can choose between a remote
            Talon SRX, CANifier, or deactivate the feature.
        :param normalOpenOrClose:
            Setting for normally open, normally closed, or disabled. This
            setting matches the web-based configuration drop down.
        :param deviceID:
            Device ID of remote source (Talon SRX or CANifier device ID).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs)

    def configReverseLimitSwitchSource(self, type: RemoteLimitSwitchSource, normalOpenOrClose: LimitSwitchNormal,
                        deviceID: int, timeoutMs: int):
        """Configures the reverse limit switch for a remote source. For example, a
        CAN motor controller may need to monitor the Limit-R pin of another Talon
        or CANifier.

        :param type:
            Remote limit switch source. User can choose between a remote
            Talon SRX, CANifier, or deactivate the feature.
        :param normalOpenOrClose:
            Setting for normally open, normally closed, or disabled. This
            setting matches the web-based configuration drop down.
        :param deviceID:
            Device ID of remote source (Talon SRX or CANifier device ID).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configReverseLimitSwitchSource(self.handle, type, normalOpenOrClose,
                                deviceID, timeoutMs)

    def configForwardLimitSwitchSource(self, type: LimitSwitchSource, normalOpenOrClose: LimitSwitchNormal,
                        timeoutMs: int, deviceID = 0):
        """Configures the forward limit switch for a remote source. For example, a
        CAN motor controller may need to monitor the Limit-F pin of another Talon
        or CANifier.

        :param type:
            Remote limit switch source. User can choose between a remote
            Talon SRX, CANifier, or deactivate the feature.
        :param normalOpenOrClose:
            Setting for normally open, normally closed, or disabled. This:
            setting matches the web-based configuration drop down.
        :param deviceID:
            Device ID of remote source (Talon SRX or CANifier device ID).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configForwardLimitSwitchSource(self.handle, type, normalOpenOrClose,
                        deviceID, timeoutMs)

    def overrideLimitSwitchesEnable(self, enable: bool):
        """Sets the enable state for limit switches.

        :param enable:
            Enable state for limit switches.
        """
        self.impl.overrideLimitSwitchesEnable(self.handle, enable)

    def configForwardSoftLimitThreshold(self, forwardSensorLimit: int, timeoutMs: int):
        """Configures the forward soft limit threhold.

        :param forwardSensorLimit:
            Forward Sensor Position Limit (in raw Sensor Units).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configForwardSoftLimitThreshold(self.handle, forwardSensorLimit, timeoutMs)

    def configReverseSoftLimitThreshold(self, reverseSensorLimit: int, timeoutMs: int):
        """Configures the reverse soft limit threshold.

        :param reverseSensorLimit:
            Reverse Sensor Position Limit (in Raw Sensor Units).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configReverseSoftLimitThreshold(self.handle, reverseSensorLimit, timeoutMs)
        
    def configForwardSoftLimitEnable(self, enable: bool, timeoutMs: int):
        """Configures the forward soft limit enable.

        :param enable:
            Forward Sensor Position Limit Enable.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configForwardSoftLimitEnable(self.handle, enable, timeoutMs)

    def configReverseSoftLimitEnable(self, enable: bool, timeoutMs: int):
        """Configures the reverse soft limit enable.

        :param enable:
            Reverse Sensor Position Limit Enable.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configReverseSoftLimitEnable(self.handle, enable, timeoutMs)

    def overrideSoftLimitsEnable(self, enable: bool):
        """Can be used to override-disable the soft limits.
        This function can be used to quickly disable soft limits without
        having to modify the persistent configuration.
        
        :param enable:
            Enable state for soft limit switches.
        """
        self.impl.overrideSoftLimitsEnable(self.handle, enable)

    def config_kP(self, slotIdx: int, value: float, timeoutMs: int):
        """Sets the 'P' constant in the given parameter slot.

        :param slotIdx:
            Parameter slot for the constant.
        :param value:
            Value of the P constant.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.config_kP(self.handle, slotIdx, value, timeoutMs)

    def config_kI(self, slotIdx: int, value: float, timeoutMs: int):
        """Sets the 'I' constant in the given parameter slot.

        :param slotIdx:
            Parameter slot for the constant.
        :param value:
            Value of the I constant.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.config_kI(self.handle, slotIdx, value, timeoutMs)

    def config_kD(self, slotIdx: int, value: float, timeoutMs: int):
        """Sets the 'D' constant in the given parameter slot.

        :param slotIdx:
            Parameter slot for the constant.
        :param value:
            Value of the D constant.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.config_kD(self.handle, slotIdx, value, timeoutMs)

    def config_kF(self, slotIdx: int, value: float, timeoutMs: int):
        """Sets the 'F' constant in the given parameter slot.

        :param slotIdx:
            Parameter slot for the constant.
        :param value:
            Value of the F constant.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.config_kF(self.handle, slotIdx, value, timeoutMs)

    def config_IntegralZone(self, slotIdx: int, izone: int, timeoutMs: int):
        """Sets the Integral Zone constant in the given parameter slot.
        Sets the Integral Zone constant in the given parameter slot. If the
        (absolute) closed-loop error is outside of this zone, integral
        accumulator is automatically cleared. This ensures than integral wind up
        events will stop after the sensor gets far enough from its target.

        :param slotIdx:
            Parameter slot for the constant.
        :param izone:
            Value of the Integral Zone constant. (closed loop error units X 1ms).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.config_IntegralZone(self.handle, slotIdx, float(izone), timeoutMs)

    def configAllowableClosedloopError(self, slotIdx: int, allowableClosedLoopError: int, timeoutMs: int):
        """Sets the allowable closed-loop error in the given parameter slot.

        :param slotIdx:
            Parameter slot for the constant.
        :param allowableClosedLoopError:
            Value of the allowable closed-loop error.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configAllowableClosedloopError(self.handle, slotIdx, allowableClosedLoopError,
                                timeoutMs)

    def configMaxIntegralAccumulator(self, slotIdx: int, iaccum: float, timeoutMs: int):
        """Sets the maximum integral accumulator in the given parameter slot.

        :param slotIdx:
            Parameter slot for the constant.
        :param iaccum:
            Value to set for the integral accumulator (closed loop error
            units X 1ms).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configMaxIntegralAccumulator(self.handle, slotIdx, iaccum, timeoutMs)

    def setIntegralAccumulator(self, iaccum: float, pidIdx: int, timeoutMs: int):
        """Sets the integral accumulator. Typically this is used to clear/zero the
        integral accumulator, however some use cases may require seeding the
        accumulator for a faster response.
        
        :param iaccum:
            Value to set for the integral accumulator (closed loop error
            units X 1ms).
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setIntegralAccumulator(self.handle, iaccum, pidIdx, timeoutMs)

    def getClosedLoopError(self, pidIdx: int) -> int:
        """Gets the closed-loop error.  The units depend on which control mode is in
        use. See Phoenix-Documentation information on units.

        :param slotIdx:
            Parameter slot of the constant.
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
        :returns: Closed-loop error value.
        """
        return self.impl.getClosedLoopError(self.handle, pidIdx)

    def getIntegralAccumulator(self, pidIdx: int) -> float:
        """Gets the iaccum value.

        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
        :returns: Integral accumulator value.
        """
        return self.impl.getIntegralAccumulator(self.handle, pidIdx)

    def getErrorDerivative(self, pidIdx: int):
        """Gets the derivative of the closed-loop error.
        
        :param slotIdx:
            Parameter slot of the constant.
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
        :returns: The error derivative value.
        """ 
        return self.impl.getErrorDerivative(self.handle, pidIdx)

    def selectProfileSlot(self, slotIdx: int, pidIdx: int):
        """Selects which profile slot to use for closed-loop control.

        :param slotIdx:
            Profile slot to select.
        :param pidIdx:
            0 for Primary closed-loop. 1 for cascaded closed-loop.
        """
        self.impl.selectProfileSlot(self.handle, slotIdx, pidIdx)

    def getActiveTrajectoryPosition(self) -> int:
        """Gets the active trajectory target position using
        MotionMagic/MotionProfile control modes.
        
        :returns: The Active Trajectory Position in sensor units.
        """
        return self.impl.getActiveTrajectoryPosition(self.handle)

    def getActiveTrajectoryVelocity(self) -> int:
        """Gets the active trajectory target velocity using
        MotionMagic/MotionProfile control modes.
        
        :returns: The Active Trajectory Velocity in sensor units per 100ms.
        """
        return self.impl.getActiveTrajectoryVelocity(self.handle)

    def getActiveTrajectoryHeading(self) -> float:
        """Gets the active trajectory target heading using
        MotionMagicArc/MotionProfileArc control modes.
        
        :returns: The Active Trajectory Heading in degreees.
        """
        return self.impl.getActiveTrajectoryHeading(self.handle)

    def configMotionCruiseVelocity(self, sensorUnitsPer100ms: int, timeoutMs: int):
        """Sets the Motion Magic Cruise Velocity.  This is the peak target velocity
        that the motion magic curve generator can use.
        
        :param sensorUnitsPer100ms:
            Motion Magic Cruise Velocity (in raw Sensor Units per 100 ms).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configMotionCruiseVelocity(self.handle, sensorUnitsPer100ms, timeoutMs)

    def configMotionAcceleration(self, sensorUnitsPer100msPerSec: int, timeoutMs: int):
        """Sets the Motion Magic Acceleration. This is the target acceleration that
        the motion magic curve generator can use.
        
        :param sensorUnitsPer100msPerSec:
            Motion Magic Acceleration (in raw sensor units per 100 ms per
            second).
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configMotionAcceleration(self.handle, sensorUnitsPer100msPerSec, timeoutMs)

    def clearMotionProfileTrajectories(self):
        """Clear the buffered motion profile in both controller's RAM (bottom), and in the
        API (top).
        """
        return self.impl.clearMotionProfileTrajectories(self.handle)

    def getMotionProfileTopLevelBufferCount(self) -> int:
        """Retrieve just the buffer count for the api-level (top) buffer. This
        routine performs no CAN or data structure lookups, so its fast and ideal
        if caller needs to quickly poll the progress of trajectory points being
        emptied into controller's RAM. Otherwise just use GetMotionProfileStatus.
        
        :returns: number of trajectory points in the top buffer.
        """
        return self.impl.getMotionProfileTopLevelBufferCount(self.handle)

    def pushMotionProfileTrajectory(self, trajPt: TrajectoryPoint):
        """Push another trajectory point into the top level buffer (which is emptied
        into the motor controller's bottom buffer as room allows).

        :param trajPt: 
            to push into buffer.

            The members should be filled in with these values...
            
            targPos:  servo position in sensor units.
            targVel:  velocity to feed-forward in sensor units
            per 100ms.
            profileSlotSelect  which slot to pull PIDF gains from.  Currently
            supports 0,1,2,3.
            isLastPoint  set to nonzero to signal motor controller to keep processing this
            trajectory point, instead of jumping to the next one
            when timeDurMs expires.  Otherwise MP executer will
            eventually see an empty buffer after the last point
            expires, causing it to assert the IsUnderRun flag.
            However this may be desired if calling application
            never wants to terminate the MP.
            zeroPos  set to nonzero to signal motor controller to "zero" the selected
            position sensor before executing this trajectory point.
            Typically the first point should have this set only thus
            allowing the remainder of the MP positions to be relative to
            zero.
        :returns: 
            CTR_OKAY if trajectory point push ok. ErrorCode if buffer is
            full due to kMotionProfileTopBufferCapacity.
        """
        return self.impl.pushMotionProfileTrajectory(self.handle,
                                trajPt.position, trajPt.velocity, trajPt.headingDeg,
                                trajPt.profileSlotSelect, trajPt.isLastPoint, trajPt.zeroPos)

    def isMotionProfileTopLevelBufferFull(self) -> bool:
        """Retrieve just the buffer full for the api-level (top) buffer. This
        routine performs no CAN or data structure lookups, so its fast and ideal
        if caller needs to quickly poll. Otherwise just use
        GetMotionProfileStatus.
        
        :returns: number of trajectory points in the top buffer.
        """
        return self.impl.isMotionProfileTopLevelBufferFull(self.handle)
        
    def processMotionProfileBuffer(self):
        """This must be called periodically to funnel the trajectory points from the
        API's top level buffer to the controller's bottom level buffer. Recommendation
        is to call this twice as fast as the execution rate of the motion
        profile. So if MP is running with 20ms trajectory points, try calling
        this routine every 10ms. All motion profile functions are thread-safe
        through the use of a mutex, so there is no harm in having the caller
        utilize threading.
        """
        self.impl.processMotionProfileBuffer(self.handle)
        
    def getMotionProfileStatus(self, statusToFill: MotionProfileStatus):
        """Retrieve all status information.
        For best performance, Caller can snapshot all status information regarding the
        motion profile executer.
        
        :param statusToFill: 
            Caller supplied object to fill.
        
            The members are filled, as follows...
            
            topBufferRem:   The available empty slots in the trajectory buffer.
            The robot API holds a "top buffer" of trajectory points, so your applicaion
            can dump several points at once.  The API will then stream them into the
            low-level buffer, allowing the motor controller to act on them.
            
            topBufferRem: The number of points in the top trajectory buffer.
            
            btmBufferCnt: The number of points in the low level controller buffer.
            
            hasUnderrun:    Set if isUnderrun ever gets set.
            Only is cleared by clearMotionProfileHasUnderrun() to ensure
            
            isUnderrun:     This is set if controller needs to shift a point from its buffer into
            the active trajectory point however
            the buffer is empty.
            This gets cleared automatically when is resolved.
            
            activePointValid:   True if the active trajectory point has not empty, false otherwise. The members in activePoint are only valid if this signal is set.
            
            isLast: is set/cleared based on the MP executer's current
            trajectory point's IsLast value.  This assumes
            IsLast was set when PushMotionProfileTrajectory
            was used to insert the currently processed trajectory
            point.
            
            profileSlotSelect: The currently processed trajectory point's
            selected slot.  This can differ in the currently selected slot used
            for Position and Velocity servo modes
            
            outputEnable:       The current output mode of the motion profile
            executer (disabled, enabled, or hold).  When changing the set()
            value in MP mode, it's important to check this signal to
            confirm the change takes effect before interacting with the top buffer.
        """
        retval = self.impl.getMotionProfileStatus(self.handle, self.motionProfStats)
        statusToFill.topBufferRem = self.motionProfStats[0]
        statusToFill.topBufferCnt = self.motionProfStats[1]
        statusToFill.btmBufferCnt = self.motionProfStats[2]
        statusToFill.hasUnderrun = self.motionProfStats[3] != 0
        statusToFill.isUnderrun = self.motionProfStats[4] != 0
        statusToFill.activePointValid = self.motionProfStats[5] != 0
        statusToFill.isLast = self.motionProfStats[6] != 0
        statusToFill.profileSlotSelect = self.motionProfStats[7]
        statusToFill.outputEnable = self.motionProfStats[8]
        return retval
        
    def clearMotionProfileHasUnderrun(self, timeoutMs: int):
        """Clear the "Has Underrun" flag. Typically this is called after application
        has confirmed an underrun had occured.
        
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.clearMotionProfileHasUnderrun(self.handle, timeoutMs)

    def changeMotionControlFramePeriod(self, periodMs: int):
        """Calling application can opt to speed up the handshaking between the robot
        API and the controller to increase the download rate of the controller's Motion
        Profile. Ideally the period should be no more than half the period of a
        trajectory point.
        
        :param periodMs:
            The transmit period in ms.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.changeMotionControlFramePeriod(self.handle, periodMs)
        
    def getLastError(self):
        """Gets the last error generated by this object.

        :returns: Last Error Code generated by a function.
        """
        return self.impl.getLastError(self.handle)

    def getFaults(self, toFill: Faults):
        """Gets the last error generated by this object. Not all functions return an
        error code but can potentially report errors. This function can be used
        to retrieve those error codes.
        
        :returns: Last Error Code generated by a function.
        """
        bits = self.impl.getFaults(self.handle)
        toFill.update(bits)
        return self.getLastError()
        
    def getStickyFaults(self, toFill: StickyFaults):
        """Polls the various sticky fault flags.
        
        :param toFill:
            Caller's object to fill with latest sticky fault flags.
        :returns: Last Error Code generated by a function.
        """
        bits = self.impl.getStickyFaults(self.handle)
        toFill.update(bits)
        return self.getLastError()

    def clearStickyFaults(self, timeoutMs: int):
        """Clears all sticky faults.
        
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Last Error Code generated by a function.
        """
        return self.impl.clearStickyFaults(self.handle, timeoutMs)

    def getFirmwareVersion(self):
        """Gets the firmware version of the device.
        
        :returns: Firmware version of device. For example: version 1-dot-2 
            is 0x0102.
        """
        return self.impl.getFirmwareVersion(self.handle)

    def hasResetOccurred(self) -> bool:
        """:returns: Returns true if the device has reset since last call"""
        return self.impl.hasResetOccurred(self.handle)

    def configSetCustomParam(self, newValue: int, paramIndex: int, timeoutMs: int):
        """Sets the value of a custom parameter. This is for arbitrary use.

        Sometimes it is necessary to save calibration/limit/target information 
        in the device. Particularly if the device is part of a subsystem that 
        can be replaced.

        :param newValue:
            Value for custom parameter.
        :param paramIndex:
            Index of custom parameter [0,1].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configSetCustomParam(self.handle, newValue, paramIndex, timeoutMs)

    def configGetCustomParam(self, paramIndex: int, timeoutMs: int):
        """Gets the value of a custom parameter.

        :param paramIndex:
            Index of custom parameter [0,1].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Value of the custom param.
        """
        return self.impl.configGetCustomParam(self.handle, paramIndex, timeoutMs)

    def configSetParameter(self, param: ParamEnum, value: float, subValue: int, ordinal: int, timeoutMs: int):
        """Sets a parameter.

        :param param:
            Parameter enumeration.
        :param value:
            Value of parameter.
        :param subValue:
            Subvalue for parameter. Maximum value of 255.
        :param ordinal:
            Ordinal of parameter.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return configSetParameter(param.value, value, subValue, ordinal, timeoutMs)

    def configSetParameter(self, param: int, value: float, subValue: int, ordinal: int, timeoutMs: int):
        """Sets a parameter. Generally this is not used. This can be utilized in
        
        - Using new features without updating API installation. 
        - Errata workarounds to circumvent API implementation. 
        - Allows for rapid testing / unit testing of firmware.

        :param param:
            Parameter enumeration.
        :param value:
            Value of parameter.
        :param subValue:
            Subvalue for parameter. Maximum value of 255.
        :param ordinal:
            Ordinal of parameter.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configSetParameter(self.handle, param, value, subValue, ordinal, timeoutMs)

    def configGetParameter(self, param: ParamEnum, ordinal: int, timeoutMs: int) -> float:
        """Gets a parameter.

        :param param:
            Parameter enumeration.
        :param ordinal:
            Ordinal of parameter.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.

        :returns: Value of parameter.
        """
        return self.impl.configGetParameter(self.handle, param, ordinal, timeoutMs)
        
    def getBaseID(self) -> int:
        return self.arbId

    def follow(self, masterToFollow: 'BaseMotorController'):
        """Set the control mode and output value so that this motor controller will
        follow another motor controller. Currently supports following Victor SPX
        and Talon SRX.
        """
        id32 = masterToFollow.getBaseID()
        id24 = id32
        id24 >>= 16
        id24 = id24 & 0xFFFF
        id24 <<= 8
        id24 |= (id32 & 0xFF)
        self.set(ControlMode.Follower, id24)

    def valueUpdated(self):
        """When master makes a device, this routine is called to signal the update."""
        pass
        
    def getSensorCollection(self) -> SensorCollection:
        """:returns: object that can get/set individual raw sensor values."""
        return self.sensorColl
        
    def getControlMode(self) -> ControlMode:
        """:returns: control mode motor controller is in"""
        return self.controlMode
