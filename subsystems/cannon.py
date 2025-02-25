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

        self.pid = self.topMotor.getClosedLoopController()
        config.closedLoop.pid(0.1,0.00015,0.01,slot=rev.ClosedLoopSlot.kSlot0)
        config.closedLoop.IMaxAccum(0.1,slot=rev.ClosedLoopSlot.kSlot0)
        config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

    def place(self):
        print("cannon is placing")
        self.topMotor.set(-0.35)
        
    def intake(self):
        self.topMotor.set(1)

    def stop(self):
        self.topMotor.set(0)
    
    def idle(self):
        self.topMotor.set(0)
        print("current cannon postition: " + str(self.encoder.getPosition()))


    def spinup(self):
        self.pivotMotor.set(-0.1)
        print("current cannon postition: " + str(self.encoder.getPosition()))

    def spindown(self):
        self.pivotMotor.set(0.1)

    def angleStop(self):
        self.pivotMotor.set(-0.017)
    def angleIdle(self):
        self.pivotMotor.set(-0.017)

    def goToPos(self,desPos):
        '''myPid=wpimath.controller.PIDController(0.1,0.001,0,period=0.02)
        myPid.setIZone(0.2)
        myPid.setSetpoint(desPos)
        pidOut=myPid.calculate(self.encoder.getPosition())'''
        self.desiredPos = desPos
        #self.pivotMotor.set(pidOut)
        self.pid.setReference(self.desiredPos, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)
        print("AAAAAAAAAAAAAAAAAAAAAAH",self.encoder.getPosition())