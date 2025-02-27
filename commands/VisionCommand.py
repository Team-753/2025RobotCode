import commands2
import time
from subsystems.limelight_camera import LimelightCamera
from subsystems.drivetrain import DriveTrainSubSystem
import RobotConfig as config

class Lock(commands2.Command):
    def __init__(self, limelight: LimelightCamera, driveTrain: DriveTrainSubSystem, forcedPostLockDirection: int):
        """
        forcedPostLockDirection must be -1 (left) or +1 (right).
        """
        super().__init__()
        if forcedPostLockDirection not in (-1, 1):
            raise ValueError("Forced post-lock direction must be -1 (left) or 1 (right)")
        self.limelight = limelight
        self.driveTrain = driveTrain
        self.forcedPostLockDirection = forcedPostLockDirection
        self.addRequirements(limelight, driveTrain)
        self.lockAchievedTime = None  # Records when lock conditions are first met

    def execute(self) -> None:
        # Read vision values and compute errors relative to desired offsets.
        x_error = self.limelight.getX() - config.VisionConstants.VISION_TX_OFFSET
        y_error = self.limelight.getY() - config.VisionConstants.VISION_TY_OFFSET
        ta_error = self.limelight.getA() - config.VisionConstants.VISION_TA_OFFSET

        # Zero out small errors (within tolerance) to reduce jitter.
        if abs(x_error) < config.VisionConstants.VISION_TOLERANCE:
            x_error = 0
        if abs(y_error) < config.VisionConstants.VISION_TOLERANCE:
            y_error = 0
        if abs(ta_error) < config.VisionConstants.VISION_TOLERANCE:
            ta_error = 0

        # Check if lock conditions are met.
        if (abs(self.limelight.getX() - config.VisionConstants.VISION_TX_OFFSET) < config.VisionConstants.VISION_TOLERANCE and
            abs(self.limelight.getY() - config.VisionConstants.VISION_TY_OFFSET) < config.VisionConstants.VISION_TOLERANCE and
            abs(self.limelight.getA() - config.VisionConstants.VISION_TA_OFFSET) < config.VisionConstants.VISION_TOLERANCE):

            if self.lockAchievedTime is None:
                self.lockAchievedTime = time.time()
            elapsed = time.time() - self.lockAchievedTime

            # For the duration of the post-lock period, force a lateral movement in the specified direction.
            if elapsed < config.VisionConstants.POST_LOCK_DURATION:
                forward_speed = 0
                rotation_speed = 0
                lateral_speed = config.VisionConstants.POST_LOCK_LATERAL_SPEED * self.forcedPostLockDirection
                self.driveTrain.joystickOverride = (forward_speed, lateral_speed, rotation_speed)
            else:
                # Once the post-lock period expires, stop movement.
                self.driveTrain.joystickOverride = (0, 0, 0)
        else:
            # Lock conditions not met: reset timer and apply normal corrections.
            self.lockAchievedTime = None
            lateral_speed = -0.015 * x_error    # Regular lateral adjustment (if needed)
            forward_speed = -0.05 * ta_error      # Regular forward/backward adjustment.
            rotation_speed = -0.01 * x_error      # Regular rotation adjustment.
            self.driveTrain.joystickOverride = (forward_speed, lateral_speed, rotation_speed)

    def isFinished(self) -> bool:
        # Automatically finish once the lock is achieved and the post-lock duration has elapsed.
        if self.lockAchievedTime is not None:
            elapsed = time.time() - self.lockAchievedTime
            if elapsed >= config.VisionConstants.POST_LOCK_DURATION:
                return True
        return False

    def end(self, interrupted: bool) -> None:
        # Clear the override and reset the lock timer.
        self.driveTrain.joystickOverride = None
        self.lockAchievedTime = None
