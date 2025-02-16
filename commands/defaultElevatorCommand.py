import commands2
from subsystems.elevator import elevatorSubSystem

class DefaultElevatorCommand(commands2.Command):
    def __init__(self, elevatorSubSystem: elevatorSubSystem):
        super().__init__()
        self.addRequirements(elevatorSubSystem)
        self.elevator = elevatorSubSystem
        print("default elevator command Running.")

    def execute(self):
        print("uh")
        self.elevator.ManualControl(self.elevator.GetJoystickInput())
        print("manually driving elevator")
    def end(self, interrupted):
        self.elevator.Brake()
        return super().end(interrupted)