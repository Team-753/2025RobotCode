import commands2
from subsystems.drivetrain import DriveTrainSubSystem
from wpimath import geometry
from wpilib import Timer

class superSimpleAuto(commands2.Command):
    """class that wraps our structures for autos that are are almost a stop auto, but stop with a command"""
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem, directions: tuple, stopTime):
        super().__init__()
        self.addRequirements(driveTrainSubSystem) #not really sure what this does, but i think its telling the command that we need to use the drive train subsystem for this command
        self.driveTrain = driveTrainSubSystem
        self.timer = Timer() #creates an instance of the wpilib timer class to be used later
        self.directions = directions #the faked joystick values that we are passing i think it goes [y, x, z]
        self.endTime = stopTime #the amount of time we want to be going for
    
    def initialize(self):
        """Function that runs at the beginning of the class"""
        self.timer.reset() #set the time on the timer to zero
        self.timer.start() #starts the timer, hopefully from zero

    def execute(self):
        """Code that runs every robot loop (20ms)"""
        self.driveTrain.joystickDrive(self.directions) #passing the faked joystick values to the drivetrain so that the robot moves
        print("running auto") #a debug statement
    
    def isFinished(self):
        """checks every robot loop to see if the end condition has been met"""
        if self.timer.hasElapsed(self.endTime):
            return True
        #if the robot has been driving long enough tell the command that it is done
        self.timer.reset() #return the timer value to zero, it is entirely possible that this is braking this function
        
    def end(self, interrupted: bool):
        #this appears to be very broken but this is the function that runs when the command ends. it stops the timer
        #self.driveTrain.joystickDrive([0, 0, 0])
        self.timer.stop()

    
