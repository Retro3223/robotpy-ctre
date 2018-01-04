# validated: 2018-01-05 EN 2d574139d4e2 java/src/com/ctre/phoenix/motorcontrol/SensorCollection.java
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
from ._impl import MotController


__all__ = ["SensorCollection"]


class SensorCollection:

    def __init__(self, motorController):
        """:type motorController: :class:`.BaseMotorController`"""
        self.motorController = motorController

    @property
    def impl(self):
        return self.motorController.impl

    @property
    def handle(self):
        return self.motorController.handle

    def getAnalogIn(self) -> int:
        """Get the position of whatever is in the analog pin of the Talon,
        regardless of whether it is actually being used for feedback.

        :returns: 
            the 24bit analog value. The bottom ten bits is the ADC (0 - 1023)
            on the analog pin of the Talon. The upper 14 bits tracks the
            overflows and underflows (continuous sensor).
        """
        return self.impl.getAnalogIn(self.handle)

    def setAnalogPosition(self, newPosition: int, timeoutMs: int):
        """Sets analog position.

        :param newPosition:
            The new position.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: an ErrorCode.
        """
        return self.impl.setAnalogPosition(self.handle, newPosition, timeoutMs)

    def getAnalogInRaw(self) -> int:
        """Get the position of whatever is in the analog pin of the Talon,
        regardless of whether it is actually being used for feedback.

        :returns: the ADC (0 - 1023) on analog pin of the Talon.
        """
        return self.impl.getAnalogInRaw(self.handle)

    def getAnalogInVel(self) -> int:
        """Get the velocity of whatever is in the analog pin of the Talon,
        regardless of whether it is actually being used for feedback.

        :returns: the speed in units per 100ms where 1024 units is one rotation.
        """
        return self.impl.getAnalogInVel(self.handle)

    def getQuadraturePosition(self) -> int:
        """Get the quadrature position of the Talon, regardless of whether
        it is actually being used for feedback.

        :returns: the Error code of the request.
        """
        return self.impl.getQuadraturePosition(self.handle)

    def setQuadraturePosition(self, newPosition: int, timeoutMs: int):
        """Change the quadrature reported position. Typically this is used to "zero"
        the sensor. This only works with Quadrature sensor. To set the selected
        sensor position regardless of what type it is, see
        SetSelectedSensorPosition in the motor controller class.

        :param newPosition:
            The position value to apply to the sensor.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: error code.
        """
        return self.impl.setQuadraturePosition(self.handle, newPosition, timeoutMs)

    def getQuadratureVelocity(self) -> int:
        """Get the quadrature velocity, regardless of whether
        it is actually being used for feedback.

        :returns: the value (0 - 1023) on the analog pin of the Talon.
        """
        return self.impl.getQuadratureVelocity(self.handle)

    def getPulseWidthPosition(self) -> int:
        """Gets pulse width position, regardless of whether
        it is actually being used for feedback.

        :returns: the pulse width position.
        """
        return self.impl.getPulseWidthPosition(self.handle)

    def setPulseWidthPosition(self, newPosition: int, timeoutMs: int):
        """Sets pulse width position.

        :param newPosition:
            The position value to apply to the sensor.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: an ErrErrorCode
        """
        return self.impl.setPulseWidthPosition(self.handle, newPosition, timeoutMs)

    def getPulseWidthVelocity(self) -> int:
        """ Gets pulse width velocity, regardless of whether
        it is actually being used for feedback.

        :returns: the pulse width velocity  in units per 100ms (where 4096 units is 1 rotation).
        """
        return self.impl.getPulseWidthVelocity(self.handle)

    def getPulseWidthRiseToFallUs(self) -> int:
        """Gets pulse width rise to fall time.

        :returns: the pulse width rise to fall time in microseconds.
        """
        return self.impl.getPulseWidthRiseToFallUs(self.handle)

    def getPulseWidthRiseToRiseUs(self) -> int:
        """Gets pulse width rise to rise time.

        :returns: the pulse width rise to rise time in microseconds.
        """
        return self.impl.getPulseWidthRiseToRiseUs(self.handle)

    def getPinStateQuadA(self) -> bool:
        """Gets pin state quad a.

        :returns: the pin state quad a (1 if asserted, 0 if not asserted).
        """
        return self.impl.getPinStateQuadA(self.handle) != 0

    def getPinStateQuadB(self) -> bool:
        """Gets pin state quad b.

        :returns: Digital level of QUADB pin (1 if asserted, 0 if not asserted).
        """
        return self.impl.getPinStateQuadB(self.handle) != 0

    def getPinStateQuadIdx(self) -> bool:
        """Gets pin state quad index.

        :returns: Digital level of QUAD Index pin (1 if asserted, 0 if not asserted).
        """
        return self.impl.getPinStateQuadIdx(self.handle) != 0

    def isFwdLimitSwitchClosed(self) -> bool:
        """Is forward limit switch closed.

        This function works regardless if limit switch feature is
        enabled.

        :returns: '1' iff forward limit switch is closed, 0 iff switch is open.
        """
        return self.impl.isFwdLimitSwitchClosed(self.handle) != 0

    def isRevLimitSwitchClosed(self) -> bool:
        """Is reverse limit switch closed.

        This function works regardless if limit switch feature is
        enabled.

        :returns: '1' iff reverse limit switch is closed, 0 iff switch is open.
        """
        return self.impl.isRevLimitSwitchClosed(self.handle) != 0
