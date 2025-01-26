import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self) -> None:
        self.intakeMotor = rev.SparkMax(RobotConfig.coralCannon.intakeMotorID, rev.SparkMax.MotorType.kBrushed)
        
    def place(self) -> None:
        print(self.intakeMotor)
        self.intakeMotor.set(1)
        
        #add move to intake position
    def intake(self) -> None:
        print(self.intakeMotor)
        self.intakeMotor.set(-1)
