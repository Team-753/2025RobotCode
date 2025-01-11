from wpimath import geometry

class robotDimensions:
    trackWidth = 0.5
    wheelBase = 0.5

class SwerveModules:

    class frontLeft:
        driveMotorID = 1
        CANCoderID = 2
        turnMotorID = 3

    class frontRight:
        driveMotorID = 4
        CANCoderID = 5
        turnMotorID = 6
    
    class rearRight:
        driveMotorID = 7
        CANCoderID = 8
        turnMotorID = 9
    
    class rearLeft:
        driveMotorID = 10
        CANCoderID = 11
        turnMotorID = 12
    
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


