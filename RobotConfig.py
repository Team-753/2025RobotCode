from wpimath import geometry

class robotDimensions:
    trackWidth = 0.5
    wheelBase = 0.5

class SwerveModules:

    drivingGearRatio = 5.68 # find a real number
    turningGearRatio = 13.3714 # find a real number
    #turning gear notes from motor to output: 10: 30: 22/16 : 88
    #contiued: 1/3* 3/2 * 5/3 * 1/6

    class frontLeft:
        driveMotorID = 1
        CANCoderID = 2
        encoderOffset = 0.7062 
        isInverted = False
        turnMotorID = 3

    class frontRight:
        driveMotorID = 4
        CANCoderID = 5
        encoderOffset = 0.184
        isInverted = False
        turnMotorID = 6
    
    class rearRight:
        driveMotorID = 7
        CANCoderID = 8
        encoderOffset = -0.5139
        isInverted = False
        turnMotorID = 9
    
    class rearLeft:
        driveMotorID = 10
        CANCoderID = 11
        encoderOffset = -0.7866
        isInverted = False
        turnMotorID = 12
        
class coralCannon:
    TopMotorID = 13
    BottomMotorID = 14
    pivotMotorID = 15
    
class Climber:
    solenoidForward = 2
    solenoidReverse = 3

    pneumaticsHubID = 20

class algaeSquisher:
    squisherPistonForward = 0
    squisherPistonReverse = 1

    squisherMotorID = 2123904

class elevator:
    leftMotorID= 16
    rightMotorID= 17

    
class driveConstants:
    wheelDiameter = 3

    class joystickConstants:
        USB_ID = 0
        xDeadband = 0.1
        yDeadband = 0.1
        theataDeadband = 0.15

    class RobotSpeeds:
        maxSpeed = 4.8
        maxAcceleration = 3
        manualRotationSpeedFactor = 0.2

    class poseConstants:
        class translationPIDConstants:
            kP = 5.0
            kI = 0.0
            kD = 0.0
            period = 0.025

        class rotationPIDConstants:
            kP = 1.0
            kI = 0.0
            kD = 0.0
            period = 0.025

        xPoseToleranceMeters = 0.05
        yPoseToleranceMeters = 0.05
        thetaPoseToleranceRadians = 0.01745
        teleopVelLimit = 4.25
        teleopAccelLimit = 3

    class ThetaPIDConstants:
        autoVelLimit = 2
        autoAccelLimit = 2
        xPoseToleranceMeters = 0.03
        yPoseToleranceMeters = 0.03
        period = 0.05
        class translationPIDConstants:
            kP = 3.0
            kI = 0.0
            kD = 0.0
            period = 0.05


