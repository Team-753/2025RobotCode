import rev,wpimath
import commands2
import wpimath.controller
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.topMotor = rev.SparkMax(RobotConfig.coralCannon.TopMotorID, rev.SparkMax.MotorType.kBrushed)
        self.bottomMotor = rev.SparkMax(RobotConfig.coralCannon.BottomMotorID, rev.SparkMax.MotorType.kBrushed)
        config = rev.SparkMaxConfig()
        config.follow(RobotConfig.coralCannon.TopMotorID, True)
        self.bottomMotor.configure(config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)
        self.encoder = self.pivotMotor.getAbsoluteEncoder()
        self.pivotMotor.IdleMode(0)
    def place(self):
        print("cannon is placing")
        self.topMotor.set(0.5)
        
    def intake(self):
        self.topMotor.set(-0.25)

    def stop(self):
        self.topMotor.set(0)
    
    def idle(self):
        self.topMotor.set(0)


    def spinup(self):
        self.pivotMotor.set(-0.1)

    def spindown(self):
        self.pivotMotor.set(0.1)

    def angleStop(self):
        self.pivotMotor.set(-0.017)
    def angleIdle(self):
        self.pivotMotor.set(-0.017)

    def goToPos(self,desPos):
        myPid=wpimath.controller.PIDController(0.1,0.001,0,period=0.02)
        myPid.setIZone(0.2)
        myPid.setSetpoint(desPos)
        pidOut=myPid.calculate(self.encoder.getPosition())
        #self.pivotMotor.set(pidOut)
        print("AAAAAAAAAAAAAAAAAAAAAAH",self.encoder.getPosition())