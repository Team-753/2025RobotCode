import commands2
from subsystems.limelight_camera import LimelightCamera
from subsystems.drivetrain import DriveTrainSubSystem
import RobotConfig as rc

class Lock(commands2.Command):
    def __init__(self, limelight: LimelightCamera, driveTrain: DriveTrainSubSystem):
        """
        Uses vision feedback to adjust the robot’s movement.
        Parameters are loaded from RobotConfig.visionConstants.
        """
        super().__init__()
        self.limelight = limelight
        self.driveTrain = driveTrain

        # Get tuning parameters from RobotConfig
        self.x_offset = rc.visionConstants.x_offset
        self.x_tolerance = rc.visionConstants.x_tolerance
        self.lateralCorrectionConstant = rc.visionConstants.lateralCorrectionConstant
        self.rotationalCorrectionConstant = rc.visionConstants.rotationalCorrectionConstant
        self.forwardSpeedMultiplier = rc.visionConstants.forwardSpeedMultiplier
        self.desired_tag_area = rc.visionConstants.desired_tag_area

        self.addRequirements(limelight, driveTrain)

    def execute(self):
        if not self.limelight.hasDetection():
            print("Lock command: No target detected; skipping override.")
            self.driveTrain.joystickOverride = None
            return

        # Get the raw x value (in degrees) and adjust by the physical offset.
        raw_x = self.limelight.getX()
        x_error = raw_x - self.x_offset
        tag_area = self.limelight.getA()

        # Compute lateral and rotational corrections if the error exceeds tolerance.
        if abs(x_error) > self.x_tolerance:
            lateral_speed = -self.lateralCorrectionConstant * x_error
            rotation_speed = -self.rotationalCorrectionConstant * x_error
        else:
            lateral_speed = 0
            rotation_speed = 0

        # Compute forward speed based on the tag area.
        area_error = self.desired_tag_area - tag_area
        forward_speed = self.forwardSpeedMultiplier * area_error if area_error > 0 else 0

        self.driveTrain.joystickOverride = (forward_speed, lateral_speed, rotation_speed)

    def end(self, interrupted: bool):
        self.driveTrain.joystickOverride = None

