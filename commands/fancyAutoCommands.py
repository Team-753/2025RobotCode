import commands2
from wpimath import geometry, controller, trajectory
from subsystems.drivetrain import DriveTrainSubSystem
from RobotConfig import driveConstants as config
from wpilib import DriverStation
from math import pi


class TurnToPosition(commands2.Command):
    def __init__(self, desiredPos, driveTrainSubsystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubsystem)
        self.driveTrainSubsystem = driveTrainSubsystem
        self.desiredPos = desiredPos
        self.angleController = controller.ProfiledPIDController
        self.angleController.setTolerance(config.poseConstants.thetaPoseToleranceRadians)
        self.angleController.enableContinuousInput(-pi, pi)


    def initialize(self):
        self.targetState =  trajectory.TrapezoidProfile.State(self.desiredPos, 0)

    def execute(self):
        self.output = self.angleController.calculate(self.driveTrainSubsystem.getCurrentPose().rotation().radians(), self.targetState)
    
    def end(self, interrupted):
        self.driveTrainSubsystem.stationary()
    
    def isFinished(self):
        return self.angleController.atGoal()
