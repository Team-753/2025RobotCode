#algae code goes here
import rev
import wpilib
import commands2
import RobotConfig as rc

class AlgaeSquisher(commands2.Subsystem):
    def __init__(self):
        #initializes the motor
        self.algaeMotor = rev.SparkMax(rc.algaeSquisher.squisherMotorID,rev.SparkMax.MotorType.kBrushless)
    
    def IntakeAlgae(self):
        self.algaeMotor.set(-.75)

    def ReleaseAlgae(self):
        self.algaeMotor.set(.75)

    def stop(self):
        self.algaeMotor.set(0)