import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self) -> None:
        self.intakeMotor = rev.CanSparkMax(RobotConfig.coralCannon.intakeMotorID, rev.CanSparkMax.MotorType.kBrushed)
        
    def place(self) -> None:
        print(self.intakeMotor)
        self.intakeMotor.set(1)
        
    def intake(self) -> None:
        print(self.intakeMotor)
        self.intakeMotor.set(-1)

    def idle(self) -> None:
        self.intakeMotor.set(0)
