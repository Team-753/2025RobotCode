import commands2
from wpimath import geometry, controller, trajectory
import wpimath
from subsystems.drivetrain import DriveTrainSubSystem
from RobotConfig import driveConstants as config
from wpilib import DriverStation, SmartDashboard, Timer
from math import pi


class TurnToPosition(commands2.Command):
    # this command works by setting up a PID controller that compares the current rotation of the robot and comparing it with the rotation that we want to have
    #it then figures out the fastest way to get where we want to be, within some limits, and then sets the wheels to go that way.
    #TODO check and see if this should be using a ProfiledPIDControllerRadians or not
    def __init__(self, desiredPos, driveTrainSubsystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubsystem)
        self.driveTrainSubsystem = driveTrainSubsystem
        self.desiredPos = desiredPos
        self.constraints = trajectory.TrapezoidProfile.Constraints(config.ThetaPIDConstants.autoVelLimit, config.ThetaPIDConstants.autoAccelLimit)
        #setting the maximum velocity and acceleration of the robot, it will not exceed these
        self.angleController = controller.ProfiledPIDController(config.poseConstants.rotationPIDConstants.kP, config.poseConstants.rotationPIDConstants.kI, config.poseConstants.rotationPIDConstants.kD, self.constraints)
        #creating a pid controller to do the math, and sending it the max speed and acceleration
        #self.state = trajectory.TrapezoidProfile.State(self.driveTrainSubsystem.poseEstimatior.getEstimatedPosition().rotation().radians())
        #self.angleController.reset(self.state)
        self.angleController.setTolerance(config.poseConstants.thetaPoseToleranceRadians)
        #we have range of acceptable positions around the exact desired setpoint
        self.angleController.enableContinuousInput(-pi, pi)
        #the robot can spin in a complete circle so the pid controller should know that 


    def initialize(self):
        self.targetState =  trajectory.TrapezoidProfile.State(self.desiredPos, 0)
        #saving our desired pose

    def execute(self):
        self.output = self.angleController.calculate(self.driveTrainSubsystem.getCurrentPose().rotation().radians(), self.targetState)
        #compare our current position to our desired position
        self.driveTrainSubsystem.setSwerveStates(0.0, 0.0, self.output)
        #drive the robot the way we want to
    
    def end(self, interrupted):
        self.driveTrainSubsystem.stationary()
        #sets all the wheels to not move, techincally breaking them possibly?
    
    def isFinished(self):
        #check weather or not we are in the position we want to be.
        #TODO set up a timer that will verify if it has remained in position for at some time
        return self.angleController.atSetpoint()
    
class GoToPosition(commands2.Command):
    def __init__(self, desiredPos: geometry.Pose2d, driveTrainSubsystem: DriveTrainSubSystem):
        self.addRequirements(driveTrainSubsystem)
        self.driveTrain = driveTrainSubsystem
        self.desiredPos = desiredPos
        self.constants = trajectory.TrapezoidProfile.Constraints(config.poseConstants.autoVelLimit, config.poseConstants.autoAccelLimit) # this is going to get confusing but this is max speed and acceleration for auto drive thats not whole robot spinny
        self.xController = controller.ProfiledPIDController(config.poseConstants.translationPIDConstants.kP, config.poseConstants.translationPIDConstants.kI, config.poseConstants.translationPIDConstants.kD, self.constants)
        self.yController = controller.ProfiledPIDController(config.poseConstants.translationPIDConstants.kP, config.poseConstants.translationPIDConstants.kI, config.poseConstants.translationPIDConstants.kD, self.constants)
        #self.xController.reset(self.driveTrain.getCurrentPose().X())
        #self.xController.reset(self.driveTrain.getCurrentPose().X())
        self.xController.setTolerance(config.poseConstants.xPoseToleranceMeters)
        self.yController.setTolerance(config.poseConstants.yPoseToleranceMeters)
        self.constraints = trajectory.TrapezoidProfileRadians.Constraints(2 *pi, 2 * pi)
        self.angleController = controller.ProfiledPIDControllerRadians(config.poseConstants.rotationPIDConstants.kP, config.poseConstants.rotationPIDConstants.kI, config.poseConstants.rotationPIDConstants.kD, self.constraints)
        self.angleController.setTolerance(config.poseConstants.thetaPoseToleranceRadians)
        self.angleController.enableContinuousInput(-pi, pi)
        self.timer = Timer()
        self.timer.reset()

    def initialize(self):
        self.targetXState = trajectory.TrapezoidProfile.State(self.desiredPos.X())
        self.targetYState = trajectory.TrapezoidProfile.State(self.desiredPos.Y())
        self.targetState =  trajectory.TrapezoidProfileRadians.State(self.desiredPos.rotation().radians())
        SmartDashboard.putNumber("desired spin: ", self.targetState.position)
        print(self.targetXState.position, self.targetYState.position, self.targetState.position)

    def execute(self):
        self.toX = self.xController.calculate(self.driveTrain.getCurrentPose().X(), self.targetXState)
        self.toY = self.yController.calculate(self.driveTrain.getCurrentPose().Y(), self.targetYState)
        self.output = self.angleController.calculate(wpimath.angleModulus(self.driveTrain.getCurrentPose().rotation().radians()), self.targetState)
        SmartDashboard.putNumber("current spin: ", wpimath.angleModulus(self.driveTrain.getCurrentPose().rotation().radians().__float__()))
        #self.driveTrain.setSwerveStates(self.toX, self.toY, self.output)
        self.driveTrain.setSwerveStates(0, 0, self.output)
        SmartDashboard.putBoolean("auto going", True)

    def end(self, interrupted):
        self.driveTrain.stationary()
        
    def isFinished(self):
        #self.timer.start()
        if self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint() and self.timer.hasElapsed(0.25):
            self.timer.stop()
            self.timer.reset()
            return True
        elif not self.timer.hasElapsed(0.25) and self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint():
            return False
        elif self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint() and not self.timer.isRunning():
            self.timer.reset()
            self.timer.start()
        elif not self.xController.atSetpoint() or not self.yController.atSetpoint() or not self.angleController.atSetpoint():
            self.timer.stop()
            self.timer.reset()
            return False
        else:
            self.timer.stop()
            self.timer.reset()
            return False
