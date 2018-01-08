# validated: 2018-01-05 EN 8f8595a20145 java/src/com/ctre/phoenix/sensors/PigeonIMU.java
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
from wpilib._impl.utils import match_arglist
from .talonsrx import TalonSRX
from ._impl import ErrorCode, ParamEnum, PigeonIMU_StatusFrame, PigeonIMU as PigeonImuImpl, PigeonIMU_ControlFrame, PigeonIMU_Faults, PigeonIMU_StickyFaults


__all__ = ['PigeonIMU', 'FusionStatus', 'GeneralStatus', 'PigeonState', 'CalibrationMode']


class FusionStatus:
    """Data object for holding fusion information."""

    def __init__(self):
        self.heading = 0.0
        self.bIsValid = False
        self.bIsFusing = False
        self.lastError = 0

    def __str__(self):
        if self.lastError != ErrorCode.OK:
            description = "Could not receive status frame.  Check wiring and web-config."
        elif not self.bIsValid:
            description = "Fused Heading is not valid."
        elif not self.bIsFusing:
            description = "Fused Heading is valid."
        else:
            description = "Fused Heading is valid and is fusing compass."
        return description


class CalibrationMode(enum.IntEnum):
    """Various calibration modes supported by Pigeon."""
    BootTareGyroAccel = 0
    Temperature = 1
    Magnetometer12Pt = 2
    Magnetometer360 = 3
    Accelerometer = 5
    Unknown = -1


class PigeonState(enum.IntEnum):
    """Overall state of the Pigeon."""
    NoComm = 0
    Initializing = 1
    Ready = 2
    UserCalibration = 3
    Unknown = -1


class GeneralStatus:
    """Data object for status on current calibration and general status.
    
    Pigeon has many calibration modes supported for a variety of uses. The
    modes generally collects and saves persistently information that makes
    the Pigeon signals more accurate. This includes collecting temperature,
    gyro, accelerometer, and compass information.
    
    For FRC use-cases, typically compass and temperature calibration is not
    required.
    
    Additionally when motion driver software in the Pigeon boots, it will
    perform a fast boot calibration to initially bias gyro and setup
    accelerometer.
    
    These modes can be enabled with the EnterCalibration mode.
    
    When a calibration mode is entered, caller can expect...
    
    - 
        PigeonState to reset to Initializing and bCalIsBooting is set to true.
        Pigeon LEDs will blink the boot pattern. This is similar to the normal
        boot cal, however it can an additional ~30 seconds since calibration
        generally requires more information. currentMode will reflect the user's
        selected calibration mode.
    
    - 
        PigeonState will eventually settle to UserCalibration and Pigeon LEDs
        will show cal specific blink patterns. bCalIsBooting is now false.
    
    - 
        Follow the instructions in the Pigeon User Manual to meet the
        calibration specific requirements. When finished calibrationError will
        update with the result. Pigeon will solid-fill LEDs with red (for
        failure) or green (for success) for ~5 seconds. Pigeon then perform
        boot-cal to cleanly apply the newly saved calibration data.
    """

    def __init__(self):
        #: The current state of the motion driver. This reflects if the sensor
        #: signals are accurate. Most calibration modes will force Pigeon to
        #: reinit the motion driver.
        state = PigeonState.Unknown
        #: The currently applied calibration mode if state is in UserCalibration
        #: or if bCalIsBooting is true. Otherwise it holds the last selected
        #: calibration mode (when calibrationError was updated).
        self.currentMode = CalibrationMode.Unknown
        #: The error code for the last calibration mode. Zero represents a
        #: successful cal (with solid green LEDs at end of cal) and nonzero is a
        #: failed calibration (with solid red LEDs at end of cal). Different
        #: calibration
        self.calibrationError = 0
        #: After caller requests a calibration mode, pigeon will perform a
        #: boot-cal before entering the requested mode. During this period, this
        #: flag is set to true.
        self.bCalIsBooting = False
        #: Temperature in Celsius
        self.tempC = 0.0
        #: Number of seconds Pigeon has been up (since boot). This register is
        #: reset on power boot or processor reset. Register is capped at 255
        #: seconds with no wrap around.
        self.upTimeSec = 0
        #: Number of times the Pigeon has automatically rebiased the gyro. This
        #: counter overflows from 15 -> 0 with no cap.
        self.noMotionBiasCount = 0
        #: Number of times the Pigeon has temperature compensated the various
        #: signals. This counter overflows from 15 -> 0 with no cap.
        self.tempCompensationCount = 0
        #: Same as getLastError()
        self.lastError = 0

    def __str__(self):
        """
        general string description of current status"""
        if self.lastError != ErrorCode.OK: # same as NoComm
            description = "Status frame was not received, check wired connections and web-based config."
        elif self.bCalIsBooting:
            description = "Pigeon is boot-caling to properly bias accel and gyro.  Do not move Pigeon.  When finished biasing, calibration mode will start."
        elif self.state == PigeonState.UserCalibration:
            # mode specific descriptions 
            if self.currentMode == CalibrationMode.BootTareGyroAccel:
                description = "Boot-Calibration: Gyro and Accelerometer are being biased."
            elif self.currentMode == CalibrationMode.Temperature:
                description = "Temperature-Calibration: Pigeon is collecting temp data and will finish when temp range is reached. \n"
                description += "Do not move Pigeon."
            elif self.currentMode == CalibrationMode.Magnetometer12Pt:
                description = "Magnetometer Level 1 calibration: Orient the Pigeon PCB in the 12 positions documented in the User's Manual."
            elif self.currentMode == CalibrationMode.Magnetometer360:
                description = "Magnetometer Level 2 calibration: Spin robot slowly in 360' fashion."
            elif self.currentMode == CalibrationMode.Accelerometer:
                description = "Accelerometer Calibration: Pigeon PCB must be placed on a level source.  Follow User's Guide for how to level surface."
            else:
                description = "Unknown status"
        elif self.state == PigeonState.Ready:
            # definitely not doing anything cal-related. So just instrument
            # the motion driver state
            description = "Pigeon is running normally.  Last CAL error code was %s." % (self.calibrationError,)
        elif self.state == PigeonState.Initializing:
            # definitely not doing anything cal-related. So just instrument
            # the motion driver state
            description = "Pigeon is boot-caling to properly bias accel and gyro.  Do not move Pigeon."
        else:
            description = "Not enough data to determine status."

        return description


class PigeonIMU(PigeonImuImpl):
    """Pigeon IMU Class. Class supports communicating over CANbus and over
    ribbon-cable (CAN Talon SRX).
    """

    def __init__(self, *args, **kwargs):
        """
        Arguments can be structured as follows:

        - deviceNumber
        - talonSrx
        
        :param deviceNumber:
            CAN Device Id of Pigeon [0,62]
        :param talonSrx:
            Object for the TalonSRX connected via ribbon cable.
        """
        deviceNumber_arg = ("deviceNumber", [int])
        talonSrx_arg = ("talonSrx", [TalonSRX])
        templates = [[deviceNumber_arg], [talonSrx_arg]]
        index, results = match_arglist('PIDController.__init__',
                                   args, kwargs, templates)
        self.generalStatus = [0] * 10
        self.fusionStatus = [0] * 10

        if index == 0:
            self.deviceNumber = results['deviceNumber']
            self.create1(self.deviceNumber)
        elif index == 1:
            self.deviceNumber = results['talonSrx'].getDeviceID()
            self.create2(self.deviceNumber)
            hal.report(64, m_deviceNumber + 1)
        hal.report(hal.UsageReporting.kResourceType_PigeonIMU, self.deviceNumber + 1)

    def getGeneralStatus(self, toFill: GeneralStatus):
        """
        Get the status of the current (or previousley complete) calibration.
        
        :param generalStatus:
        :param toFill: Container for the status information.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        (toFill.lastError, toFill.state, toFill.currentMode, toFill.calibrationError, 
         toFill.bCalIsBooting, toFill.tempC, toFill.upTimeSec, toFill.noMotionBiasCount, 
         toFill.tempCompensationCount) = super().getGeneralStatus()
        return toFill.lastError

    def getFusedHeading(self, toFill: FusionStatus = None) -> float:
        """
        :param status:
            object reference to fill with fusion status flags.
            Caller may pass null if flags are not needed.
        :returns: The fused heading in degrees.
        """

        if toFill is None:
            return self.getFusedHeading1()
        else:
            (toFill.lastError, toFill.heading, toFill.bIsFusing, toFill.bIsValid) = self.getFusedHeading2()
            return toFill.lastError
        
    def getFaults(self, toFill: PigeonIMU_Faults):
        """
        Gets the fault status

        :param toFill:
            Container for fault statuses.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        _, bits = super().getFaults()
        toFill.update(bits)
        return self.getLastError()

    def getStickyFaults(self, toFill: PigeonIMU_StickyFaults):
        """
        Gets the sticky fault status

        :param toFill:
            Container for sticky fault statuses.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        _, bits = super().getStickyFaults()
        toFill.update(bits)
        return self.getLastError()
