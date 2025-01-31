from subsystems.swerveModule import *
from phoenix6 import swerve, hardware
import navx
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from phoenix6 import SignalLogger, swerve, units, utils
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d
import copy
import ctypes
from typing import final, overload, Callable, Generic, TypeVar
from threading import RLock
from phoenix6.status_code import StatusCode
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.hardware.traits.common_talon import CommonTalon
from phoenix6.hardware.parent_device import ParentDevice
from phoenix6.signals.spn_enums import NeutralModeValue
from phoenix6.swerve.swerve_drivetrain_constants import SwerveDrivetrainConstants
from phoenix6.swerve.swerve_module_constants import SwerveModuleConstants
from phoenix6.swerve.swerve_module import SwerveModule
from phoenix6.swerve.utility.geometry import *
from phoenix6.swerve.utility.kinematics import *
from phoenix6.swerve import requests
from phoenix6.units import *
from phoenix6.phoenix_native import (
    Native,
    Pose_t,
    SwerveControlParams_t,
    SwerveDriveState_t,
    SwerveModulePosition_t,
    SwerveModuleState_t,
)
from wpimath.kinematics import SwerveDrive2Kinematics, SwerveDrive3Kinematics, SwerveDrive4Kinematics, SwerveDrive6Kinematics
from wpimath.geometry import Rotation2d, Rotation3d
from phoenix6.swerve.sim_swerve_drivetrain import SimSwerveDrivetrain
from phoenix6.swerve.swerve_drivetrain import SwerveControlParameters

"""""class DriveTrainConstants(swerve.SwerveDrivetrainConstants):
    def __init__(self, NavXID):
        super().__init__()"""
'''driveTrainConstants = swerve.SwerveDrivetrainConstants()
driveTrainConstants.pigeon2_configs = None
driveTrainConstants.with_pigeon2_configs(None)'''

DriveMotorT = TypeVar("DriveMotorT", bound="CommonTalon")
SteerMotorT = TypeVar("SteerMotorT", bound="CommonTalon")
EncoderT = TypeVar("EncoderT", bound="ParentDevice")

_NUM_CONFIG_ATTEMPTS = 2
class MyDriveTrain(Subsystem, swerve.SwerveDrivetrain):

    _SIM_LOOP_PERIOD: units.second = 0.005
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants: swerve.SwerveDrivetrainConstants, arg0, arg1= None, arg2= None, arg3= None):
        Subsystem.__init__(self)
        #swerve.SwerveDrivetrain.__init__(self, driveMotorType, steerMotorType, CANCoderType, driveTrainConstants, myModules)
        
        self._drivetrain_id = 0
        """ID of the native drivetrain instance, used for native calls."""
        USE_WPILIB = True

        if (
            isinstance(arg0, list) and isinstance(arg0[0], SwerveModuleConstants) and
            arg1 is None and
            arg2 is None and
            arg3 is None
        ):
            # Self(drivetrain_constants, modules)
            modules: list[SwerveModuleConstants] = arg0

            native_drive_constants = drivetrain_constants._create_native_instance()
            native_module_constants = SwerveModuleConstants._create_native_instance(modules)

            self._drivetrain_id = Native.api_instance().c_ctre_phoenix6_swerve_create_drivetrain(
                native_drive_constants,
                native_module_constants,
                len(modules),
            )

            Native.instance().c_ctre_phoenix6_free_memory(ctypes.byref(ctypes.cast(native_drive_constants, ctypes.c_char_p)))
            Native.instance().c_ctre_phoenix6_free_memory(ctypes.byref(ctypes.cast(native_module_constants, ctypes.c_char_p)))
        elif (
            isinstance(arg0, (hertz, float)) and
            isinstance(arg1, list) and isinstance(arg1[0], SwerveModuleConstants) and
            arg2 is None and
            arg3 is None
        ):
            # Self(drivetrain_constants, odometry_update_frequency, modules)
            modules: list[SwerveModuleConstants] = arg1

            native_drive_constants = drivetrain_constants._create_native_instance()
            native_module_constants = SwerveModuleConstants._create_native_instance(modules)

            self._drivetrain_id = Native.api_instance().c_ctre_phoenix6_swerve_create_drivetrain_with_freq(
                native_drive_constants,
                arg0,
                native_module_constants,
                len(modules),
            )

            Native.instance().c_ctre_phoenix6_free_memory(ctypes.byref(ctypes.cast(native_drive_constants, ctypes.c_char_p)))
            Native.instance().c_ctre_phoenix6_free_memory(ctypes.byref(ctypes.cast(native_module_constants, ctypes.c_char_p)))
        elif (
            isinstance(arg0, (hertz, float)) and
            isinstance(arg1, tuple[float, float, float]) and
            isinstance(arg2, tuple[float, float, float]) and
            isinstance(arg3, list) and isinstance(arg3[0], SwerveModuleConstants)
        ):
            # Self(drivetrain_constants, odometry_update_frequency, odometry_standard_deviation, vision_standard_deviation, modules)
            modules: list[SwerveModuleConstants] = arg3

            odometry_standard_deviation = (ctypes.c_double * 3)()
            odometry_standard_deviation[0] = arg1[0]
            odometry_standard_deviation[1] = arg1[1]
            odometry_standard_deviation[2] = arg1[2]

            vision_standard_deviation = (ctypes.c_double * 3)()
            vision_standard_deviation[0] = arg2[0]
            vision_standard_deviation[1] = arg2[1]
            vision_standard_deviation[2] = arg2[2]

            native_drive_constants = drivetrain_constants._create_native_instance()
            native_module_constants = SwerveModuleConstants._create_native_instance(modules)

            self._drivetrain_id = Native.api_instance().c_ctre_phoenix6_swerve_create_drivetrain_with_freq(
                native_drive_constants,
                arg0,
                odometry_standard_deviation,
                vision_standard_deviation,
                native_module_constants,
                len(modules),
            )

            Native.instance().c_ctre_phoenix6_free_memory(ctypes.byref(ctypes.cast(native_drive_constants, ctypes.c_char_p)))
            Native.instance().c_ctre_phoenix6_free_memory(ctypes.byref(ctypes.cast(native_module_constants, ctypes.c_char_p)))
        else:
            raise TypeError('Invalid arguments for SwerveDrivetrain.__init__')

        self.__modules: list[SwerveModule[DriveMotorT, SteerMotorT, EncoderT]] = []
        self.__module_locations: list[Translation2d] = []
        for i, module in enumerate(modules):
            self.__modules.append(
                SwerveModule(
                    drive_motor_type,
                    steer_motor_type,
                    encoder_type,
                    module,
                    drivetrain_constants.can_bus_name,
                    self._drivetrain_id,
                    i
                )
            )
            self.__module_locations.append(Translation2d(module.location_x, module.location_y))

        if USE_WPILIB:
            if len(modules) == 2:
                self.__kinematics = SwerveDrive2Kinematics(self.__module_locations[0], self.__module_locations[1])
            elif len(modules) == 3:
                self.__kinematics = SwerveDrive3Kinematics(self.__module_locations[0], self.__module_locations[1], self.__module_locations[2])
            elif len(modules) == 4:
                self.__kinematics = SwerveDrive4Kinematics(self.__module_locations[0], self.__module_locations[1], self.__module_locations[2], self.__module_locations[3])
            elif len(modules) == 6:
                self.__kinematics = SwerveDrive6Kinematics(self.__module_locations[0], self.__module_locations[1], self.__module_locations[2], self.__module_locations[3], self.__module_locations[4], self.__module_locations[5])
            else:
                self.__kinematics = None

        self.__control_params = SwerveControlParameters()
        self.__control_params.drivetrain_id = self._drivetrain_id
        if USE_WPILIB:
            self.__control_params.kinematics = self.__kinematics
        self.__control_params.module_locations = self.__module_locations

        self.__swerve_request: requests.SwerveRequest = requests.Idle()
        self.__control_handle = None

        self.__telemetry_function: Callable[['MyDriveTrain.SwerveDriveState'], None] = None
        self.__telemetry_handle = None

        self.__state_lock = RLock()
        self.__cached_state = self.SwerveDriveState()
        self.__cached_state.module_states = [SwerveModuleState() for _ in modules]
        self.__cached_state.module_targets = [SwerveModuleState() for _ in modules]
        self.__cached_state.module_positions = [SwerveModulePosition() for _ in modules]

        #self.__pigeon2 = Pigeon2(drivetrain_constants.pigeon2_id, drivetrain_constants.can_bus_name)
        #if USE_WPILIB:
            #self.__sim_drive = SimSwerveDrivetrain(self.__module_locations, self.__pigeon2.sim_state, modules)

        if drivetrain_constants.pigeon2_configs is not None:
            for _ in range(_NUM_CONFIG_ATTEMPTS):
                retval = self.pigeon2.configurator.apply(drivetrain_constants.pigeon2_configs)
                if retval.is_ok():
                    break
            if not retval.is_ok():
                print(f"Pigeon2 ID {self.pigeon2.device_id} failed to config with error: {retval.name}")

        # do not start thread until after applying Pigeon 2 configs
        self.__odometry_thread = self.OdometryThread(self._drivetrain_id)
        self.__odometry_thread.start()
        self.NavX = navx.AHRS.create_spi()


        
        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        # Swerve request to apply during path following
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()
        self._configure_auto_builder()

    def _configure_auto_builder(self):
        pass
        '''config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,   # Supplier of current robot pose
            self.reset_pose,                 # Consumer for seeding pose against auto
            lambda: self.get_state().speeds, # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                # PID constants for translation
                PIDConstants(10.0, 0.0, 0.0),
                # PID constants for rotation
                PIDConstants(7.0, 0.0, 0.0)
            ),
            config,
            # Assume the path needs to be flipped for Red vs Blue, this is normally the case
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
            self # Subsystem for requirements
        )'''

    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def add_vision_measurement(self, vision_robot_pose: Pose2d, timestamp: units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None):
        """
        Adds a vision measurement to the Kalman Filter. This will correct the
        odometry pose estimate while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        set_vision_measurement_std_devs or this method.

        :param vision_robot_pose:           The pose of the robot as measured by the vision camera.
        :type vision_robot_pose:            Pose2d
        :param timestamp:                   The timestamp of the vision measurement in seconds.
        :type timestamp:                    second
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians.
        :type vision_measurement_std_devs:  tuple[float, float, float] | None
        """
        swerve.SwerveDrivetrain.add_vision_measurement(self, vision_robot_pose, utils.fpga_to_current_time(timestamp), vision_measurement_std_devs)

    

    def get_rotation3d(self):
        return self.NavX.getRotation3d()
    
    def set_control(self, request: requests.SwerveRequest):
        """
        Applies the specified control request to this swerve drivetrain.

        :param request: Request to apply
        :type request: requests.SwerveRequest
        """
        if self.__swerve_request is not request:
            self.__swerve_request = request

            if request is None:
                Native.api_instance().c_ctre_phoenix6_swerve_drivetrain_set_control(self._drivetrain_id, None, None)
                self.__control_handle = None
            elif isinstance(request, requests.NativeSwerveRequest):
                request._apply_native(self._drivetrain_id)
                self.__control_handle = None
            else:
                def control_callback(_, control_params_ptr: ctypes._Pointer):
                    control_params: SwerveControlParams_t = control_params_ptr.contents
                    self.__control_params._update_from_native(control_params)
                    if request:
                        return request.apply(self.__control_params, self.__modules).value
                    else:
                        return StatusCode.OK

                c_control_func_t = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p, ctypes.POINTER(SwerveControlParams_t))
                c_control_func = c_control_func_t(control_callback)

                Native.api_instance().c_ctre_phoenix6_swerve_drivetrain_set_control(self._drivetrain_id, None, c_control_func)
                self.__control_handle = c_control_func
        elif isinstance(request, requests.NativeSwerveRequest):
            request._apply_native(self._drivetrain_id)
    




        