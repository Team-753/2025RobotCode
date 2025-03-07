import commands2
from wpimath import geometry, controller, trajectory
from subsystems.drivetrain import DriveTrainSubSystem
from RobotConfig import driveConstants as config
from wpilib import DriverStation, SmartDashboard
from math import pi


class TurnToPosition(commands2.Command):
    def __init__(self, desiredPos, driveTrainSubsystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubsystem)
        self.driveTrainSubsystem = driveTrainSubsystem
        self.desiredPos = desiredPos
        self.constraints = trajectory.TrapezoidProfile.Constraints(config.ThetaPIDConstants.autoVelLimit, config.ThetaPIDConstants.autoAccelLimit)
        self.angleController = controller.ProfiledPIDController(config.poseConstants.rotationPIDConstants.kP, config.poseConstants.rotationPIDConstants.kI, config.poseConstants.rotationPIDConstants.kD, self.constraints)
        #self.state = trajectory.TrapezoidProfile.State(self.driveTrainSubsystem.poseEstimatior.getEstimatedPosition().rotation().radians())
        #self.angleController.reset(self.state)
        self.angleController.setTolerance(config.poseConstants.thetaPoseToleranceRadians)
        self.angleController.enableContinuousInput(-pi, pi)


    def initialize(self):
        self.targetState =  trajectory.TrapezoidProfile.State(self.desiredPos, 0)

    def execute(self):
        self.output = self.angleController.calculate(self.driveTrainSubsystem.getCurrentPose().rotation().radians(), self.targetState)
        self.driveTrainSubsystem.setSwerveStates(0.0, 0.0, self.output)
    
    def end(self, interrupted):
        self.driveTrainSubsystem.stationary()
    
    def isFinished(self):
        return self.angleController.atSetpoint()
    
class GoToPosition(commands2.Command):
    def __init__(self, desiredPos: geometry.Pose2d, driveTrainSubsystem: DriveTrainSubSystem):
        self.addRequirements(driveTrainSubsystem)
        self.driveTrain = driveTrainSubsystem
        self.desiredPos = desiredPos
        self.constants = trajectory.TrapezoidProfile.Constraints(config.RobotSpeeds.maxSpeed, config.RobotSpeeds.maxAcceleration) # this is going to get confusing but this is max speed and acceleration for auto drive thats not whole robot spinny
        self.xController = controller.ProfiledPIDController(config.poseConstants.translationPIDConstants.kP, config.poseConstants.translationPIDConstants.kI, config.poseConstants.translationPIDConstants.kD, self.constants)
        self.yController = controller.ProfiledPIDController(config.poseConstants.translationPIDConstants.kP, config.poseConstants.translationPIDConstants.kI, config.poseConstants.translationPIDConstants.kD, self.constants)
        #self.xController.reset(self.driveTrain.getCurrentPose().X())
        #self.xController.reset(self.driveTrain.getCurrentPose().X())
        self.xController.setTolerance(config.poseConstants.xPoseToleranceMeters)
        self.yController.setTolerance(config.poseConstants.yPoseToleranceMeters)
        self.constraints = trajectory.TrapezoidProfile.Constraints(config.ThetaPIDConstants.autoVelLimit, config.ThetaPIDConstants.autoAccelLimit)
        self.angleController = controller.ProfiledPIDController(config.poseConstants.rotationPIDConstants.kP, config.poseConstants.rotationPIDConstants.kI, config.poseConstants.rotationPIDConstants.kD, self.constraints)
        self.angleController.setTolerance(config.poseConstants.thetaPoseToleranceRadians)
        self.angleController.enableContinuousInput(-pi, pi)

    def initialize(self):
        self.targetXState = trajectory.TrapezoidProfile.State(self.desiredPos.X())
        self.targetYState = trajectory.TrapezoidProfile.State(self.desiredPos.Y())
        self.targetState =  trajectory.TrapezoidProfile.State(self.desiredPos.rotation().radians())
        
    def execute(self):
        self.toX = self.xController.calculate(self.driveTrain.getCurrentPose().X())
        self.toY = self.yController.calculate(self.driveTrain.getCurrentPose().Y())
        self.output = self.angleController.calculate(self.driveTrain.getCurrentPose().rotation().radians(), self.targetState)
        self.driveTrain.setSwerveStates(self.toX, self.toY, self.output)
        SmartDashboard.putBoolean("auto going", True)

    def end(self, interrupted):
        self.driveTrain.stationary()
        
    def isFinished(self):
        return self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint()
            
