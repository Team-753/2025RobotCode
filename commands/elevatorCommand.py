import commands2,wpilib
from subsystems.elevator import elevatorSubSystem,posElevatorSubsystem


class ElevatorJoystickCommand(commands2.Command):
    def __init__(self, kelevatorSubsystem: elevatorSubSystem, joystick):
        super().__init__()
        self.elevator = kelevatorSubsystem
        self.joystick = joystick
        self.addRequirements(kelevatorSubsystem)  # Ensures only one command controls the elevator

    def execute(self):
        joystick_value = self.joystick.getLeftY()

        if joystick_value > 0.2:  # Move up
            self.elevator.goUp(self)
        elif joystick_value < -0.2:  # Move down
            self.elevator.goDown(self)
        else:  # Stop when joystick is centered
            self.elevator.idle()

    def isFinished(self):
        return False  # Runs continuously as the default command
#maybe works
class elevatorToPos(commands2.Command):
    def __init__(self, kElevSub:posElevatorSubsystem):
        super().__init__()
        self.addRequirements
        self.eSub=kElevSub
        self.xbox=wpilib.XboxController(1)
    def execute(self):
        self.eSub.setPosition(self.xbox.getLeftY())
        return super().execute()

