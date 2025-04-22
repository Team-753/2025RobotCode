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

    #it is VERY broken IGNORE this command. the bottom one works tho
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
    """Command that will autonomously drive the robot to any position on the field... theororetically"""
    def __init__(self, desiredPos: geometry.Pose2d, driveTrainSubsystem: DriveTrainSubSystem):
        self.addRequirements(driveTrainSubsystem) #tell the command that we are using the drivetrain
        self.driveTrain = driveTrainSubsystem 
        self.desiredPos = desiredPos #store the desired position
        self.constants = trajectory.TrapezoidProfile.Constraints(config.poseConstants.autoVelLimit, config.poseConstants.autoAccelLimit) # this is going to get confusing but this is max speed and acceleration for auto drive thats not whole robot spinny, but translates x and y
        self.xController = controller.ProfiledPIDController(config.poseConstants.translationPIDConstants.kP, config.poseConstants.translationPIDConstants.kI, config.poseConstants.translationPIDConstants.kD, self.constants)
        self.yController = controller.ProfiledPIDController(config.poseConstants.translationPIDConstants.kP, config.poseConstants.translationPIDConstants.kI, config.poseConstants.translationPIDConstants.kD, self.constants)
        #the above two lines create PID controllers for the x and y corrdinates. basically this function works by having those two pid controllers and a third one for the spin check the current pose of the robot against the desired one and then trying to match the desired pose. The pid controllers cannot give speeds or accelerations above the ones outlined
        #self.xController.reset(self.driveTrain.getCurrentPose().X())
        #self.xController.reset(self.driveTrain.getCurrentPose().X())
        self.xController.setTolerance(config.poseConstants.xPoseToleranceMeters) #the amount we can be off in the x direction and still be considered to be where we want
        self.yController.setTolerance(config.poseConstants.yPoseToleranceMeters) #same as above but for the y dimension
        self.constraints = trajectory.TrapezoidProfileRadians.Constraints(pi/2, pi/2) # max speed and acceleration for robot spin, in radians
        self.angleController = controller.ProfiledPIDControllerRadians(config.poseConstants.rotationPIDConstants.kP, config.poseConstants.rotationPIDConstants.kI, config.poseConstants.rotationPIDConstants.kD, self.constraints)
        #pid controller for the robot spin. compares the current rotation of the robot with the desired one and then tries really hard to make the two values the same. cant set speed or acceleration faster than outlined above
        self.angleController.setTolerance(config.poseConstants.thetaPoseToleranceRadians) # the amount the spin can be off and still be considered in the right spot
        self.angleController.enableContinuousInput(-pi, pi) #tells the angle controller that the robot is spinning in a circle, therefor -pi and pi are the same. also says that we want to use values between pi and -pi whenever possible
        self.timer = Timer() #creates a timer for checking to see if were staying in the spot we want to be in
        self.timer.reset() #sets the time on the timer to 0

    def initialize(self):
        """code that runs when the command starts"""
        self.targetXState = trajectory.TrapezoidProfile.State(self.desiredPos.X()) #tells the command the x position we want to have
        self.targetYState = trajectory.TrapezoidProfile.State(self.desiredPos.Y()) #tells the command the y position we want to have
        self.targetState =  trajectory.TrapezoidProfileRadians.State(self.desiredPos.rotation().radians()) # tells the command the rotation we wnat to have
        SmartDashboard.putNumber("desired spin: ", self.targetState.position) #putting our desired spin location on shuffleboard for debugging
        print(self.targetXState.position, self.targetYState.position, self.targetState.position) #printing all the relevant values for debugging

    def execute(self):
        """code that runs every robot loop (20ms)"""
        self.toX = self.xController.calculate(self.driveTrain.getCurrentPose().X(), self.targetXState) #calculate what the robot needs to do to get to the x pose we want
        self.toY = self.yController.calculate(self.driveTrain.getCurrentPose().Y(), self.targetYState) #calculate what the robot needs to do to get to the y pose we want
        self.output = self.angleController.calculate(-1 * wpimath.angleModulus(self.driveTrain.getCurrentPose().rotation().radians()), self.targetState) #same as above two but for rotation
        SmartDashboard.putNumber("current spin: ", -1 * wpimath.angleModulus(self.driveTrain.getCurrentPose().rotation().radians().__float__())) #puts the current rotation on shuffleboard. we have to invert it because the navx got inverted. also we are constraining the value to being between -pi and pi
        SmartDashboard.putNumber("current x", self.driveTrain.getCurrentPose().X()) #Putting the current position of the robot onto shuffleboard
        SmartDashboard.putNumber("current y", self.driveTrain.getCurrentPose().Y()) #same as above, but for y
        self.driveTrain.setSwerveStates(self.toX, self.toY, self.output) #actually drives the drivetrain
        #self.driveTrain.setSwerveStates(0, 0, self.output)
        SmartDashboard.putBoolean("auto going", True) #puts a boolean on shuffleboard saying that the robot is not done driving

    def end(self, interrupted):
        """Code that runs when the command ends"""
        self.driveTrain.stationary() #stop the robot from moving, brakes the motors, might stall them if we do it too long
        SmartDashboard.putBoolean("auto going", False) # changes the boolean to show that the code is done now
        
    def isFinished(self):
        """checks to see if the code has acheive its finish condition"""
        # this basically works by checking if there is a timer is running and if the pid controllers have reach their setpoints
        # if both are true then it stops the timer and returns that the end condition has been achieved
        #if the pid controllers have not reached their setpoints it stops any timer that might be running and sets it back to zero
        #if the pid controllers have reached their setpoints but there is no timer it starts a timer
        #if the pid controllers have reached thier setpoints and a timer is running but a quarter second has not passed since it started it lets the timer keep running
        #theres also a catchall for any other situations that stops time timer and resets it

        SmartDashboard.putNumber("timer", self.timer.get())# puts the current time on the timer on shuffleboard
       
        if self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint() and self.timer.hasElapsed(0.25):
            #checks to see if all the pid controllers are at their setpoints and if the timer has been running for at least a quarter second
            self.timer.stop() #stop the timer
            self.timer.reset() #set the timer back to 0
            SmartDashboard.putNumber("auto ending", 1) #update the shuffleboard so the operater can see where in the code the robot is
            return True #tell the command that it can run the end code now
        
        elif self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint() and not self.timer.isRunning():
            #checks to see if all the pid controllers have reached their set points and if there is not timer
            self.timer.reset() #sets the time on the timer to 0
            self.timer.start() #starts the timer
            SmartDashboard.putNumber("auto ending", 3) #updates the shuffleboard for the operater
            return False #the code is not done yet
        
        elif not self.timer.hasElapsed(0.25) and self.xController.atSetpoint() and self.yController.atSetpoint() and self.angleController.atSetpoint():
            #checks to see if the timer has not acheved a quarter second and the controllers are still at their setpoints
            SmartDashboard.putNumber("auto ending", 2) #updates the shuffleboard about where the robot is in the code
            return False #the code is not done yet
        
        elif not self.xController.atSetpoint() or not self.yController.atSetpoint() or not self.angleController.atSetpoint():
            #checks if any of the pid controllers are not at their set points
            self.timer.stop() #stops the timer if its running
            self.timer.reset() #sets the timer back to zero
            SmartDashboard.putNumber("auto ending", 4) #update the shuffleboard
            return False #the code is not done
        
        else:
            #this is if something i havent predicted happens
            self.timer.stop() #stop the timer if its running
            self.timer.reset() #set it to zero
            SmartDashboard.putNumber("auto ending", 5) #tell the operater where the robot is in the code
            return False #the code is not done
        
        
