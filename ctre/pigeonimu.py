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
        """general string description of current status"""
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


class PigeonIMU:
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

        self.impl = PigeonImuImpl()
        if index == 0:
            self.deviceNumber = results['deviceNumber']
            self.handle = self.impl.create1(self.deviceNumber)
        elif index == 1:
            self.deviceNumber = results['talonSrx'].getDeviceID()
            self.handle = self.impl.create2(self.deviceNumber)
            hal.report(64, m_deviceNumber + 1)
        hal.report(hal.UsageReporting.kResourceType_PigeonIMU, self.deviceNumber + 1)

    def setYaw(self, angleDeg: float, timeoutMs: int):
        """Sets the Yaw register to the specified value.

        :param angleDeg: Degree of Yaw [+/- 23040 degrees]
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setYaw(self.handle, angleDeg, timeoutMs)

    def addYaw(self, angleDeg: float, timeoutMs: int):
        """Atomically add to the Yaw register.

        :param angleDeg: Degrees to add to the Yaw register.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.addYaw(self.handle, angleDeg, timeoutMs)

    def setYawToCompass(self, timeoutMs: int):
        """Sets the Yaw register to match the current compass value.

        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setYawToCompass(self.handle, timeoutMs)

    def setFusedHeading(self, angleDeg: float, timeoutMs: int):
        """Sets the Fused Heading to the specified value.

        :param angleDeg: Degree of heading [+/- 23040 degrees]
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setFusedHeading(self.handle, angleDeg, timeoutMs)

    def addFusedHeading(self, angleDeg: float, timeoutMs: int):
        """Atomically add to the Fused Heading register.

        :param angleDeg: Degrees to add to the Fused Heading register.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.addFusedHeading(self.handle, angleDeg, timeoutMs)

    def setFusedHeadingToCompass(self, timeoutMs: int):
        """Sets the Fused Heading register to match the current compass value.
        
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setFusedHeadingToCompass(self.handle, timeoutMs)

    def setAccumZAngle(self, angleDeg: float, timeoutMs: int):
        """Sets the AccumZAngle.
        
        :param angleDeg: Degrees to set AccumZAngle to.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setAccumZAngle(self.handle, angleDeg, timeoutMs)

    def configTemperatureCompensationEnable(self, bTempCompEnable: bool, timeoutMs: int):
        """Enable/Disable Temp compensation. Pigeon defaults with this on at boot.

        :param bTempCompEnable: Set to "True" to enable temperature compensation.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.configTemperatureCompensationEnable(self.handle, int(bTempCompEnable), timeoutMs)

    def setCompassDeclination(self, angleDegOffset: float, timeoutMs: int):
        """Set the declination for compass. Declination is the difference between
        Earth Magnetic north, and the geographic "True North".

	:param angleDegOffset:  Degrees to set Compass Declination to.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setCompassDeclination(self.handle, angleDegOffset, timeoutMs)

    def setCompassAngle(self, angleDeg: float, timeoutMs: int):
        """Sets the compass angle. Although compass is absolute [0,360) degrees, the
        continuous compass register holds the wrap-arounds.

        :param angleDeg:
            Degrees to set continuous compass angle to.
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for config
            success and report an error if it times out. If zero, no
            blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setCompassAngle(self.handle, angleDeg, timeoutMs)

    def enterCalibrationMode(self, calMode: CalibrationMode, timeoutMs: int):
        """Enters the Calbration mode.  See the Pigeon IMU documentation for More
        information on Calibration.

        :param calMode: Calibration to execute
        :param timeoutMs:
            Timeout value in ms. If nonzero, function will wait for
            config success and report an error if it times out.
            If zero, no blocking or checking is performed.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.enterCalibrationMode(self.handle, calMode.value, timeoutMs)

    def getGeneralStatus(self, toFill: GeneralStatus):
        """Get the status of the current (or previousley complete) calibration.
        
        :param generalStatus:
        :param toFill: Container for the status information.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        retval = self.impl.getGeneralStatus(self.handle, self.generalStatus)
        toFill.state = self.generalStatus[0]
        toFill.currentMode = self.generalStatus[1]
        toFill.calibrationError = self.generalStatus[2]
        toFill.bCalIsBooting = self.generalStatus[3] != 0
        toFill.tempC = self.generalStatus[4]
        toFill.upTimeSec = self.generalStatus[5]
        toFill.noMotionBiasCount = self.generalStatus[6]
        toFill.tempCompensationCount = self.generalStatus[7]
        toFill.lastError = retval
        return toFill.lastError

    def getLastError(self):
        """Call GetLastError() generated by this object.

        Not all functions return an error code but can
        potentially report errors.

        This function can be used to retrieve those error codes.
        
        :returns: The last ErrorCode generated.
        """
        return self.impl.getLastError(self.handle)

    def get6dQuaternion(self, wxyz):
        """Get 6d Quaternion data.

        :param wxyz: Array to fill with quaternion data w[0], x[1], y[2], z[3]
        :returns: The last ErrorCode generated.
        """
        return self.impl.get6dQuaternion(self.handle, wxyz)

    def getYawPitchRoll(self, ypr_deg):
        """Get Yaw, Pitch, and Roll data.

        :param ypr_deg: Array to fill with yaw[0], pitch[1], and roll[2] data
        :returns: The last ErrorCode generated.
        """
        return self.impl.getYawPitchRoll(self.handle, ypr_deg)

    def getAccumGyro(self, xyz_deg):
        """Get AccumGyro data.
        AccumGyro is the integrated gyro value on each axis.

        :param xyz_deg: Array to fill with x[0], y[1], and z[2] AccumGyro data
        :returns: The last ErrorCode generated.
        """
        return self.impl.getAccumGyro(self.handle, xyz_deg)

    def getAbsoluteCompassHeading(self) -> float:
        """Get the absolute compass heading.

        :returns: compass heading [0,360) degrees."""
        return self.impl.getAbsoluteCompassHeading(self.handle)

    def getCompassHeading(self) -> float:
        """Get the continuous compass heading.

        :returns: 
            continuous compass heading [-23040, 23040) degrees. 
            Use SetCompassHeading to modify the wrap-around portion.
        """
        return self.impl.getCompassHeading(self.handle)

    def getCompassFieldStrength(self) -> float:
        """Gets the compass' measured magnetic field strength.
        
        :returns: field strength in Microteslas (uT)."""
        return self.impl.getCompassFieldStrength(self.handle)

    def getTemp(self) -> float:
        """Gets the temperature of the pigeon.

        :returns: Temperature in ('C)
        """
        return self.impl.getTemp(self.handle)

    def getState(self) -> PigeonState:
        """Gets the current Pigeon state

        :returns: PigeonState enum
        """
        return self.impl.getState(self.handle)

    def getUpTime(self) -> int:
        """Gets the current Pigeon uptime.
        
        :returns: How long has Pigeon been running in whole seconds. Value caps at 255."""
        return self.impl.getUpTime(self.handle)

    def getRawMagnetometer(self, rm_xyz):
        """Get Raw Magnetometer data.

        :param rm_xyz: Array to fill with x[0], y[1], and z[2] data
        :returns: The last ErrorCode generated.
        """
        return self.impl.getRawMagnetometer(self.handle, rm_xyz)

    def getBiasedMagnetometer(self, bm_xyz):
        """Get Biased Magnetometer data.

        :param bm_xyz: Array to fill with x[0], y[1], and z[2] data
        :returns: The last ErrorCode generated.
        """
        return self.impl.getBiasedMagnetometer(self.handle, bm_xyz)

    def getBiasedAccelerometer(self, ba_xyz):
        """Get Biased Accelerometer data.

        :param ba_xyz: Array to fill with x[0], y[1], and z[2] data
        :returns: The last ErrorCode generated.
        """
        return self.impl.getBiasedAccelerometer(self.handle, ba_xyz)

    def getRawGyro(self, xyz_dps):
        """Get Biased Accelerometer data.

        :param xyz_dps: Array to fill with x[0], y[1], and z[2] data in degrees per second.
        :returns: The last ErrorCode generated.
        """
        return self.impl.getRawGyro(self.handle, xyz_dps)

    def getAccelerometerAngles(self, tiltAngles):
        """Get Accelerometer tilt angles.
        
        :param tiltAngles: Array to fill with x[0], y[1], and z[2] angles.
        :returns: The last ErrorCode generated.
        """
        return self.impl.getAccelerometerAngles(self.handle, tiltAngles)

    def getFusedHeading(self, toFill: FusionStatus = None) -> float:
        """
        :param status:
            object reference to fill with fusion status flags.
            Caller may pass null if flags are not needed.
        :returns: The fused heading in degrees.
        """

        if toFill is None:
            self.impl.getFusedHeading1(self.handle)
        else:
            errorCode = self.impl.getFusedHeading2(self.handle, self.fusionStatus)
            toFill.heading = self.fusionStatus[0]
            toFill.bIsFusing = self.fusionStatus[1] != 0
            toFill.bIsValid = self.fusionStatus[2] != 0
            toFill.lastError = errorCode
            return self.fusionStatus[0]
        
    def getFirmwareVersion(self) -> int:
        """Gets the firmware version of the device.

        :returns: 
            param holds the firmware version of the device. Device must be powered
            cycled at least once.
        """
        return self.impl.getFirmwareVersion(self.handle)

    def hasResetOccurred(self) -> bool:
        """:returns: true iff a reset has occurred since last call."""
        return self.impl.hasResetOccurred(self.handle)

    def configSetCustomParam(self, newValue: int, paramIndex: int, timeoutMs: int):
        """Sets the value of a custom parameter. This is for arbitrary use.

        Sometimes it is necessary to save calibration/declination/offset
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

        Sometimes it is necessary to save calibration/declination/offset
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
        return self.impl.configSetParameter(self.handle, param, value, subValue, ordinal,
                                timeoutMs)

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

    def setStatusFramePeriod(self, statusFrame: PigeonIMU_StatusFrame, periodMs: int, timeoutMs: int):
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

    def getStatusFramePeriod(self, frame: PigeonIMU_StatusFrame, timeoutMs: int):
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

    def setControlFramePeriod(self, frame: PigeonIMU_ControlFrame, periodMs: int):
        """Sets the period of the given control frame.
        
        :param frame:
            Frame whose period is to be changed.
        :param periodMs:
            Period in ms for the given frame.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        return self.impl.setControlFramePeriod(self.handle, frame, periodMs)

    def getFaults(self, toFill: PigeonIMU_Faults):
        """Gets the fault status

        :param toFill:
            Container for fault statuses.
        :returns: Error Code generated by function. 0 indicates no error.
        """
        bits = self.impl.getFaults(self.handle)
        toFill.update(bits)
        return self.getLastError()

    def getStickyFaults(self, toFill: PigeonIMU_StickyFaults):
        """Gets the sticky fault status

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
