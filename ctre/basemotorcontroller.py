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


class BaseMotorController(MotController):
    """Base motor controller features for all CTRE CAN motor controllers."""

    def __init__(self, arbId: int):
        """
        Constructor for motor controllers.
        
        :param arbId:
        """
        self.create1(arbId)
        self.arbId = arbId
        self.sensorColl = SensorCollection(self)
        self.motionProfStats = [0] * 9
        self.controlMode = ControlMode.PercentOutput
        self.sendMode = ControlMode.PercentOutput

    def getHandle(self):
        """
        :returns: CCI handle for child classes."""
        return self.handle

    def getDeviceID(self):
        """
        Returns the Device ID
        
        :returns: Device number.
        """
        return self.getDeviceNumber()[1]

    def set(self, mode: ControlMode, demand0: float, demand1: float = 0.0):
        """
        Sets the appropriate output on the talon, depending on the mode.

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
            self.setDemand(self.sendMode, int(1023 * demand0), 0)
        elif self.controlMode == ControlMode.Follower:
            # did caller specify device ID
            if 0 <= demand0 <= 62:
                work = self.getBaseID()
                work >>= 16
                work <<= 8
                work |= int(demand0) & 0xFF
            else:
                work = int(demand0)
            self.setDemand(self.sendMode, work, 0)
        elif self.controlMode in [ControlMode.Velocity, ControlMode.Position, ControlMode.MotionMagic, ControlMode.MotionMagicArc, ControlMode.MotionProfile]:
            self.setDemand(self.sendMode, int(demand0), 0)
        elif self.controlMode == ControlMode.Current:
            self.setDemand(self.sendMode, int(1000. * demand0), 0) # milliamps 
        else:
            self.setDemand(self.sendMode, 0, 0)

    def neutralOutput(self):
        """
        Neutral the motor output by setting control mode to disabled."""
        self.set(ControlMode.Disabled, 0, 0)

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
        super().setInverted(invert)

    def getInverted(self):
        """:returns: invert setting of motor output"""
        return self.invert

    def getMotorOutputVoltage(self):
        """:returns: applied voltage to motor in volts"""
        return self.getBusVoltage() * self.getMotorOutputPercent()

    def getMotionProfileStatus(self, statusToFill: MotionProfileStatus = None):
        """
        Retrieve all status information.
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
        if statusToFill is not None:
            (retval, statusToFill.topBufferRem, statusToFill.topBufferCnt, statusToFill.btmBufferCnt, 
            statusToFill.hasUnderrun, statusToFill.isUnderrun, statusToFill.activePointValid, statusToFill.isLast, 
            statusToFill.profileSlotSelect, statusToFill.outputEnable)  = super().getMotionProfileStatus()

            return retval
        else:
            return super().getMotionProfileStatus()

    def getStickyFaults(self, toFill: StickyFaults):
        """
        Polls the various sticky fault flags.
        
        :param toFill:
            Caller's object to fill with latest sticky fault flags.
        :returns: Last Error Code generated by a function.
        """
        _, bits = super().getStickyFaults()
        toFill.update(bits)
        return self.getLastError()

    def getBaseID(self) -> int:
        return self.arbId

    def follow(self, masterToFollow: 'BaseMotorController'):
        """
        Set the control mode and output value so that this motor controller will
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
        """
        When master makes a device, this routine is called to signal the update."""
        pass
        
    def getSensorCollection(self) -> SensorCollection:
        """
        :returns: object that can get/set individual raw sensor values."""
        return self.sensorColl
        
    def getControlMode(self) -> ControlMode:
        """
        :returns: control mode motor controller is in"""
        return self.controlMode
