import hal

if hal.isSimulation():
    from .autogen.canifier_sim import CANifier
    from .autogen.motcontroller_sim import MotController
    from .autogen.pigeonimu_sim import PigeonIMU
else:
    from .ctre_roborio import (
        CANifier,
        MotController,
        PigeonIMU
    )


# todo: eliminate
import enum
from unittest.mock import MagicMock

class ControlMode (enum.IntEnum):
    PercentOutput = 0
    Position = 1
    Velocity = 2
    Current = 3
    Follower = 5
    MotionProfile = 6
    MotionMagic = 7
    MotionMagicArc = 8
    #TimedPercentOutput = 9
    MotionProfileArc = 10
    Disabled = 15

    
NeutralMode = MagicMock()
ErrorCode = MagicMock()
RemoteFeedbackDevice = MagicMock()
FeedbackDevice = MagicMock()
RemoteSensorSource = MagicMock()
SensorTerm = MagicMock()
ControlFrame = MagicMock()
StatusFrame = MagicMock()
StatusFrameEnhanced = MagicMock()
VelocityMeasPeriod = MagicMock()
RemoteLimitSwitchSource = MagicMock()
LimitSwitchNormal = MagicMock()
LimitSwitchSource = MagicMock()
TrajectoryPoint = MagicMock()
MotionProfileStatus = MagicMock()
Faults = MagicMock()
StickyFaults = MagicMock()
ParamEnum = MagicMock()
GeneralPin = MagicMock()
CANifierStatusFrame = MagicMock()
CANifierControlFrame = MagicMock()
CANifierFaults = MagicMock()
CANifierStickyFaults = MagicMock()
ParamEnum = MagicMock()
PigeonIMU_StatusFrame = MagicMock()
PigeonIMU_ControlFrame = MagicMock()
PigeonIMU_Faults = MagicMock()
PigeonIMU_StickyFaults = MagicMock()
