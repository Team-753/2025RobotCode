from phoenix6 import swerve, hardware, configs
from RobotConfig import SwerveModules, robotDimensions

        

'''class OurSwerveModule(swerve.SwerveModule):
    def __init__(self):
        self.driveID 
        self.steerID = steerID
        self.encoderID = encoderID
        self.encoderOffset = encoderOffset
        self.index = index
        self.constants = swerve.SwerveModuleConstants()
        self.constants.create_module_constants(self.steerID, self.driveID, self.encoderID, self.encoderOffset, robotDimensions.trackWidth, robotDimensions.wheelBase, False, False, False)
        #self.module = swerve.SwerveModule(hardware.TalonFX, hardware.TalonFX, hardware.CANcoder, self.constants, "", 0, self.index)

#frontLeft = swerve.SwerveModuleConstants()
frontLeft = swerve.SwerveModuleConstants(SwerveModules.frontLeft.driveMotorID, SwerveModules.frontLeft.turnMotorID, 
                                  SwerveModules.frontLeft.CANCoderID, SwerveModules.frontLeft.encoderOffset, 
                                  robotDimensions.trackWidth, robotDimensions.wheelBase, False, False, False)
#frontLeft.
frontRightFactory = swerve.SwerveModuleConstantsFactory()
frontRightFactory.create_module_constants(SwerveModules.frontRight.driveMotorID, SwerveModules.frontRight.turnMotorID, 
                                   SwerveModules.frontRight.CANCoderID, SwerveModules.frontRight.encoderOffset,
                                   robotDimensions.trackWidth, robotDimensions.wheelBase, False, False, False)
frontRight = swerve.SwerveModuleConstants()
frontRight.
rearLeft = swerve.SwerveModuleConstantsFactory()
rearLeft.create_module_constants(SwerveModules.rearLeft.driveMotorID, SwerveModules.rearLeft.turnMotorID, 
                                 SwerveModules.rearLeft.CANCoderID, SwerveModules.rearLeft.encoderOffset, 
                                 robotDimensions.trackWidth, robotDimensions.wheelBase, False, False, False)
rearRight = swerve.SwerveModuleConstantsFactory()
rearRight.create_module_constants(SwerveModules.rearRight.driveMotorID, SwerveModules.rearRight.turnMotorID, 
                                  SwerveModules.rearRight.CANCoderID, SwerveModules.rearRight.encoderOffset, 
                                  robotDimensions.trackWidth, robotDimensions.wheelBase, False, False, False)
'''
frontLeftConstants = swerve.SwerveModuleConstants()
frontLeftConstants.drive_motor_id = SwerveModules.frontLeft.driveMotorID
frontLeftConstants.drive_motor_initial_configs = configs.TalonFXConfiguration()
frontLeftConstants.drive_motor_gear_ratio = SwerveModules.drivingGearRatio
frontLeftConstants.steer_motor_id = SwerveModules.frontLeft.turnMotorID
frontLeftConstants.steer_motor_initial_configs = configs.TalonFXConfiguration()
frontLeftConstants.steer_motor_gear_ratio = 1
frontLeftConstants.encoder_id = SwerveModules.frontLeft.CANCoderID
frontLeftConstants.encoder_initial_configs = configs.CANcoderConfiguration()
frontLeftConstants.encoder_offset = SwerveModules.frontLeft.encoderOffset
frontLeftConstants.location_x = robotDimensions.trackWidth
frontLeftConstants.location_y = robotDimensions.wheelBase

frontRightConstants = swerve.SwerveModuleConstants()
frontRightConstants.drive_motor_id = SwerveModules.frontRight.driveMotorID
frontRightConstants.drive_motor_initial_configs = configs.TalonFXConfiguration()
frontRightConstants.drive_motor_gear_ratio = SwerveModules.drivingGearRatio
frontRightConstants.steer_motor_id = SwerveModules.frontRight.turnMotorID
frontRightConstants.steer_motor_initial_configs = configs.TalonFXConfiguration()
frontRightConstants.steer_motor_gear_ratio = 1
frontRightConstants.encoder_id = SwerveModules.frontRight.CANCoderID
frontRightConstants.encoder_initial_configs = configs.CANcoderConfiguration()
frontLeftConstants.encoder_offset = SwerveModules.frontRight.encoderOffset
frontLeftConstants.location_x = robotDimensions.trackWidth
frontLeftConstants.location_y = robotDimensions.wheelBase

rearLeftConstants = swerve.SwerveModuleConstants()
rearLeftConstants.drive_motor_id = SwerveModules.rearLeft.driveMotorID
rearLeftConstants.drive_motor_initial_configs = configs.TalonFXConfiguration()
rearLeftConstants.drive_motor_gear_ratio = SwerveModules.drivingGearRatio
rearLeftConstants.steer_motor_id = SwerveModules.rearLeft.turnMotorID
rearLeftConstants.steer_motor_initial_configs = configs.TalonFXConfiguration()
rearLeftConstants.steer_motor_gear_ratio = 1
rearLeftConstants.encoder_id = SwerveModules.rearLeft.CANCoderID
rearLeftConstants.encoder_initial_configs = configs.CANcoderConfiguration()
rearLeftConstants.encoder_offset = SwerveModules.rearLeft.encoderOffset
rearLeftConstants.location_x = robotDimensions.trackWidth
rearLeftConstants.location_y = robotDimensions.wheelBase

rearRightConstants = swerve.SwerveModuleConstants()
rearRightConstants.drive_motor_id = SwerveModules.rearRight.driveMotorID
rearRightConstants.drive_motor_initial_configs = configs.TalonFXConfiguration()
rearRightConstants.drive_motor_gear_ratio = SwerveModules.drivingGearRatio
rearRightConstants.steer_motor_id = SwerveModules.rearRight.turnMotorID
rearRightConstants.steer_motor_initial_configs = configs.TalonFXConfiguration()
rearRightConstants.steer_motor_gear_ratio = 1
rearRightConstants.encoder_id = SwerveModules.rearRight.CANCoderID
rearRightConstants.encoder_initial_configs = configs.CANcoderConfiguration()
rearRightConstants.encoder_offset = SwerveModules.rearRight.encoderOffset
rearRightConstants.location_x = robotDimensions.trackWidth
rearRightConstants.location_y = robotDimensions.wheelBase

myModules = [frontLeftConstants, frontRightConstants, rearLeftConstants, rearRightConstants]
    
