package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.SwerveModuleConstants;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {
  /** All joystick, button, and axis IDs. */
  public static class Controls {
    public static final double AXIS_DEADZONE = 0.1;

    public static final int DRIVE_JOYSTICK_ID = 0;

    public static final int TRANSLATION_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int TRANSLATION_Y_AXIS = XboxController.Axis.kLeftY.value;
    public static final int ROTATION_AXIS = XboxController.Axis.kRightX.value;

    public static final int GYRO_RESET_BUTTON = XboxController.Button.kY.value;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class Swerve {
    /** Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75); // TODO: Set width of the drivetrain measured
                                                                          // from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(14.75); // TODO: Set length of the drivetrain measured
                                                                         // from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0 / 1.04085); // TODO: validate
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // TODO: set based on gearing of specific modules used.
                                                              // 6.75:1 for MK4i L2
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0); // TODO: set based on specific modules used
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;

    /** Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5; // TODO: Set according to Modules used
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 4.0;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = false;
    public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /**
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */
    public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants(
        0,
        1,
        2,
        3,
        203.115234,
        "Front Left");

    public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants(
        1,
        4,
        5,
        6,
        191.074219,
        "Front Right");

    public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants(
        2,
        7,
        8,
        9,
        203.906250,
        "Rear Left");

    public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants(
        3,
        10,
        11,
        12,
        155.214844,
        "Rear Right");
  }

  public static class AutoConstants {
    /** PID Values. */
    public static final double X_CONTROLLER_KP = 1.0;
    public static final double Y_CONTROLLER_KP = 1.0;
    public static final double THETA_CONTROLLER_KP = 1.0;

    /** Constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 5.0;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND_SQUARED = Math.PI;;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_RADIANS_PER_SECOND, MAX_ANGULAR_RADIANS_PER_SECOND_SQUARED);
  }
}