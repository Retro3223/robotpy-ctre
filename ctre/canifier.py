# validated: 2018-01-05 EN 8f8595a20145 java/src/com/ctre/phoenix/CANifier.java
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
import enum
import hal

from ._impl import GeneralPin, ParamEnum, CANifierStatusFrame, CANifierControlFrame, CANifierFaults, CANifierStickyFaults, CANifier as CANifierImpl


__all__ = ['CANifier', 'LEDChannel', 'PWMChannel']


class LEDChannel(enum.IntEnum):
    """Enum for the LED Output Channels"""
    LEDChannelA = 0
    LEDChannelB = 1
    LEDChannelC = 2


class PWMChannel(enum.IntEnum):
    """Enum for the PWM Input Channels"""
    PWMChannel0 = 0
    PWMChannel1 = 1
    PWMChannel2 = 2
    PWMChannel3 = 3


class PinValues:
    """Class to hold the pin values."""
    def __init__(self):
        self.QUAD_IDX = False
        self.QUAD_B = False
        self.QUAD_A = False
        self.LIMR = False
        self.LIMF = False
        self.SDA = False
        self.SCL = False
        self.SPI_CS_PWM3 = False
        self.SPI_MISO_PWM2 = False
        self.SPI_MOSI_PWM1 = False
        self.SPI_CLK_PWM0 = False


class CANifier:
    """CTRE CANifier
        
    Device for interfacing common devices to the CAN bus.
    """

    PWMChannelCount = 4

    def __init__(self, deviceId: int):
        """Constructor.

        :param deviceId: The CAN Device ID of the CANifier.
        """
        self.impl = CANifierImpl()
        self.handle = self.impl.create1(deviceId)
        self.tempPins = [False] * 11
        hal.report(hal.UsageReporting.kResourceType_CANifier, deviceId + 1)

    def setLEDOutput(self, percentOutput: float, ledChannel: LEDChannel):
        """Sets the LED Output

        :param percentOutput: Output duty cycle expressed as percentage.
        :param ledChannel: Channel to set the output of.
        """
        if percentOutput > 1:
            percentOutput = 1
        if percentOutput < 0:
            percentOutput = 0
        dutyCycle = int(percentOutput * 1023) # [0,1023]

        self.impl.setLEDOutput(self.handle, dutyCycle, ledChannel)

    def setGeneralOutput(self, outputPin: GeneralPin, outputValue: bool, outputEnable: bool):
        """Sets the output of a General Pin

        :param outputPin: The pin to use as output.
        :param outputValue: The desired output state.
        :param outputEnable: Whether this pin is an output. "True" enables output.
        """
        self.impl.setGeneralOutput(self.handle, outputPin, outputValue, outputEnable)

    def setGeneralOutputs(self, outputBits: int, isOutputBits: int):
        """Sets the output of all General Pins

        :param outputBits: A bit mask of all the output states.  LSB->MSB is in the order of the #GeneralPin enum.
        :param isOutputBits: A boolean bit mask that sets the pins to be outputs or inputs.  A bit of 1 enables output.
        """
        self.impl.setGeneralOutputs(self.handle, outputBits, isOutputBits)

    def getGeneralInputs(self, allPins: PinValues):
        """Gets the state of all General Pins

        :param allPins: A structure to fill with the current state of all pins.
        """
        self.impl.getGeneralInputs(self.handle, self.tempPins)
        allPins.LIMF = self.tempPins[GeneralPin.LIMF]
        allPins.LIMR = self.tempPins[GeneralPin.LIMR]
        allPins.QUAD_A = self.tempPins[GeneralPin.QUAD_A]
        allPins.QUAD_B = self.tempPins[GeneralPin.QUAD_B]
        allPins.QUAD_IDX = self.tempPins[GeneralPin.QUAD_IDX]
        allPins.SCL = self.tempPins[GeneralPin.SCL]
        allPins.SDA = self.tempPins[GeneralPin.SDA]
        allPins.SPI_CLK_PWM0 = self.tempPins[GeneralPin.SPI_CLK_PWM0P]
        allPins.SPI_MOSI_PWM1 = self.tempPins[GeneralPin.SPI_MOSI_PWM1P]
        allPins.SPI_MISO_PWM2 = self.tempPins[GeneralPin.SPI_MISO_PWM2P]
        allPins.SPI_CS_PWM3 = self.tempPins[GeneralPin.SPI_CS]

    def getGeneralInput(self, inputPin: GeneralPin) -> bool:
        """Gets the state of the specified pin

        :param inputPin: The index of the pin.
        :returns: The state of the pin.
        """
        return self.impl.getGeneralInput(self.handle, inputPin)

    def getLastError(self):
        """Call GetLastError() generated by this object.
        Not all functions return an error code but can
        potentially report errors.

        This function can be used to retrieve those error codes.

        :returns: The last ErrorCode generated.
        """
        return self.impl.getLastError(self.handle)

    def setPWMOutput(self, pwmChannel: int, dutyCycle: float):
        """Sets the PWM Output
        Currently supports PWM 0, PWM 1, and PWM 2

        :param pwmChannel: Index of the PWM channel to output.
        :param dutyCycle: Duty Cycle (0 to 1) to output.  Default period of the signal is 4.2 ms.
        """
        dutyCycle = max(dutyCycle, 0)
        dutyCycle = min(dutyCycle, 1)
        pwmChannel = max(pwmChannel, 0)

        dutyCyc10bit = int(1023 * dutyCycle)

        self.impl.setPWMOutput(self.handle, pwmChannel, dutyCyc10bit)

    def enablePWMOutput(self, pwmChannel: int, bEnable: bool):
        """Enables PWM Outputs
        Currently supports PWM 0, PWM 1, and PWM 2

        :param pwmChannel: Index of the PWM channel to enable.
        :param bEnable: True" enables output on the pwm channel.
        """
        pwmChannel = max(pwmChannel, 0)
        self.impl.enablePWMOutput(self.handle, pwmChannel, bEnable)

    def getPWMInput(self, pwmChannel: PWMChannel, dutyCycleAndPeriod):
        """Gets the PWM Input

        :param pwmChannel: PWM channel to get.
        :param dutyCycleAndPeriod: Double array to hold Duty Cycle [0] and Period [1].
        """
        self.impl.getPWMInput(self.handle, pwmChannel, dutyCycleAndPeriod)

    def configSetCustomParam(self, newValue: int, paramIndex: int, timeoutMs: int):
        """Sets the value of a custom parameter. This is for arbitrary use.

        Sometimes it is necessary to save calibration/duty cycle/output
        information in the device. Particularly if the
        device is part of a subsystem that can be replaced.
        
        :param newValue:
            Value for custom parameter.
        :param paramIndex:
            Index of custom parameter [0-1].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configSetCustomParam(self.handle, newValue, paramIndex, timeoutMs)

    def configGetCustomParam(self, paramIndex: int, timeoutMs: int) -> int:
        """Gets the value of a custom parameter. This is for arbitrary use.

        Sometimes it is necessary to save calibration/duty cycle/output
        information in the device. Particularly if the
        device is part of a subsystem that can be replaced.

        :param paramIndex:
            Index of custom parameter [0-1].
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Value of the custom param.
        """
        return self.impl.configGetCustomParam(self.handle, paramIndex, timeoutMs)

    def configSetParameter(self, param: ParamEnum, value: float, subValue: int, ordinal: int, timeoutMs: int):
        """Sets a parameter. Generally this is not used.

        This can be utilized in

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
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configSetParameter(self.handle, param, value, subValue, ordinal, timeoutMs)

    def configGetParameter(self, param: ParamEnum, ordinal: int, timeoutMs: int) -> float:
        """Gets a parameter. Generally this is not used.

        This can be utilized in

        - Using new features without updating API installation.
        - Errata workarounds to circumvent API implementation.
        - Allows for rapid testing / unit testing of firmware.

        :param param:
            Parameter enumeration.
        :param ordinal:
            Ordinal of parameter.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Value of parameter.
        """
        return self.impl.configGetParameter(self.handle, param, ordinal, timeoutMs)

    def setStatusFramePeriod(self, statusFrame: CANifierStatusFrame, periodMs: int, timeoutMs: int):
        """Sets the period of the given status frame.
        
        :param statusFrame:
            Frame whose period is to be changed.
        :param periodMs:
            Period in ms for the given frame.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setStatusFramePeriod(self.handle, statusFrame, periodMs, timeoutMs)

    def getStatusFramePeriod(self, frame: CANifierStatusFrame, timeoutMs: int) -> int:
        """Gets the period of the given status frame.
        
        :param frame:
            Frame to get the period of.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Period of the given status frame.
        """
        return self.impl.getStatusFramePeriod(self.handle, frame, timeoutMs)

    def setControlFramePeriod(self, frame: CANifierControlFrame, periodMs: int):
        """Sets the period of the given control frame.

        :param frame:
            Frame whose period is to be changed.
        :param periodMs:
            Period in ms for the given frame.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setControlFramePeriod(self.handle, frame, periodMs)

    def getFirmwareVersion(self) -> int:
        """Gets the firmware version of the device.

        :returns: Firmware version of device.
        """
        return self.impl.getFirmwareVersion(self.handle)

    def hasResetOccurred(self) -> bool:
        """Returns true if the device has reset since last call.

        :returns: Has a Device Reset Occurred?
        """
        return self.impl.hasResetOccurred(self.handle)

    def getFaults(self, toFill: CANifierFaults):
        """Gets the CANifier fault status
        
        :param toFill:
            Container for fault statuses.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        bits = self.impl.getFaults(self.handle)
        toFill.update(bits)
        return self.getLastError()

    def getStickyFaults(self, toFill: CANifierStickyFaults):
        """Gets the CANifier sticky fault status

        :param toFill:
            Container for sticky fault statuses.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        bits = self.impl.getStickyFaults(self.handle)
        toFill.update(bits)
        return self.getLastError()

    def clearStickyFaults(self, timeoutMs: int):
        """Clears the Sticky Faults
        
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.clearStickyFaults(self.handle, timeoutMs)

    def getBusVoltage(self) -> float:
        """Gets the bus voltage seen by the device.
        
        :returns: The bus voltage value (in volts).
        """
        return self.impl.getBusVoltage(self.handle)
