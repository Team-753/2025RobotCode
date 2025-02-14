import wpilib
from wpilib import SmartDashboard
from subsystems.vision import Vision

class MyRobot(wpilib.TimedRobot):
    def __init__(self):
        # Call the parent class constructor
        super().__init__()

        # Initialize your subsystems and other components here
        self.vision_system = Vision()
        SmartDashboard.putBoolean("False?", False)

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Update the vision system
        self.vision_system.update()

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
