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

        pivotConfig = rev.SparkMaxConfig()
        self.pid = self.pivotMotor.getClosedLoopController()
        pivotConfig.closedLoop.pid(2.0, 0.000, 0.001, slot=rev.ClosedLoopSlot.kSlot0)
        pivotConfig.closedLoop.IMaxAccum(0.1,slot=rev.ClosedLoopSlot.kSlot0)
        pivotConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        pivotConfig.inverted(True)
        self.pivotMotor.configure(pivotConfig, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        
    def place(self):
        print("cannon is placing")
        self.topMotor.set(-0.35)
        
    def intake(self):
        self.topMotor.set(1) #full speed

    def stop(self):
        self.topMotor.set(0)
    
    def idle(self):
        self.topMotor.set(0)
        print("current cannon postition: " + str(self.encoder.getPosition()))


    def spinup(self):
        self.pivotMotor.set(0.1) # slow speed, because otherwise things break
        print("current cannon postition: " + str(self.encoder.getPosition()))

    def spindown(self):
        self.pivotMotor.set(-0.1)

    def angleStop(self):
        self.pivotMotor.set(0.016)
    def angleIdle(self):
        self.pivotMotor.set(0.016)

    def goToPos(self,desPos):  #makes the cannon go to the desired position
        self.desiredPos = desPos
        self.pid.setReference(self.desiredPos, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, 0)
        print("CANNONING",desPos)
        
    def periodic(self):
        #print("cannon position: " + str(self.encoder.getPosition()))
        pass