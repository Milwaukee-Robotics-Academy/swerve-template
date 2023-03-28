package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Swerve;

public class SwerveModule extends SubsystemBase {

  /**
   * Class to represent and handle a swerve module
   * A module's state is measured by a CANCoder for the absolute position,
   * integrated CANEncoder for relative position
   * for both rotation and linear movement
   */

  public final String description;
  public final int number;


  public PIDController testRotationController;

  private static final double rotationkP = 0.5;
  private static final double rotationkD = 0.0;

  private static final double drivekP = 0.015;

  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;

  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  public CANSparkMax getRotationMotor() {
    return rotationMotor;
  }

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANCoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final Rotation2d offset;

  private final SparkMaxPIDController rotationController;
  private final SparkMaxPIDController driveController;

  public SwerveModule(SwerveModuleConstants constants){

    number = constants.number;
    description = constants.description;
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    rotationMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();
    testRotationController = new PIDController(0.5, 0, 0.0);
    testRotationController.enableContinuousInput(-Math.PI, Math.PI);

    canCoder = new CANCoder(constants.cancoderID);

    offset = constants.angleOffset;

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kCoast);

    rotationController = rotationMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    rotationController.setP(rotationkP);
    rotationController.setD(rotationkD);

    driveController.setP(drivekP);

    // set the output of the drive encoder to be in radians for linear measurement
    driveEncoder.setPositionConversionFactor(
        2.0 * Math.PI / Swerve.DRIVE_GEAR_RATIO);

    // set the output of the drive encoder to be in radians per second for velocity
    // measurement
    driveEncoder.setVelocityConversionFactor(
        2.0 * Math.PI / 60 / Swerve.DRIVE_GEAR_RATIO);

    // set the output of the rotation encoder to be in radians
    rotationEncoder.setPositionConversionFactor(2 * Math.PI / Swerve.ANGLE_GEAR_RATIO);

    // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  public void resetDistance() {

    driveEncoder.setPosition(0.0);

  }

  public double getDriveDistanceRadians() {

    return driveEncoder.getPosition();

  }

  public Rotation2d getCanCoderAngle() {

    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians())
        % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);

  }

  public Rotation2d getIntegratedAngle() {

    double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

    if (unsignedAngle < 0)
      unsignedAngle += 2 * Math.PI;

    return new Rotation2d(unsignedAngle);

  }

  public double getCurrentVelocityRadiansPerSecond() {

    return driveEncoder.getVelocity();

  }

  public double getCurrentVelocityMetersPerSecond() {

    return driveEncoder.getVelocity() * (Swerve.WHEEL_DIAMETER / 2.0);

  }

  public double getCurrentDistanceMetersPerSecond() {
    return driveEncoder.getPosition() * (Swerve.WHEEL_DIAMETER / 2.0);
  }

  // unwraps a target angle to be [0,2π]
  public static double placeInAppropriate0To360Scope(double unwrappedAngle) {

    double modAngle = unwrappedAngle % (2.0 * Math.PI);

    if (modAngle < 0.0)
      modAngle += 2.0 * Math.PI;

    double wrappedAngle = modAngle;

    return wrappedAngle;

  }

  // initialize the integrated NEO encoder to the offset (relative to home
  // position)
  // measured by the CANCoder
  public void initRotationOffset() {

    rotationEncoder.setPosition(getCanCoderAngle().getRadians());

  }

  /**
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to
   * include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of
   * writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {

    double targetAngle = placeInAppropriate0To360Scope(desiredState.angle.getRadians());

    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = (targetAngle - currentAngle.getRadians());
    if (Math.abs(delta) > (Math.PI / 2)) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
    }
    return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  }

  /**
   * Method to set the desired state of the swerve module
   * Parameter:
   * SwerveModuleState object that holds a desired linear and rotational setpoint
   * Uses PID and a feedforward to control the output
   */
  public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState) {
    if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());

    double angularSetPoint = placeInAppropriate0To360Scope(
        optimizedDesiredState.angle.getRadians());

    rotationMotor.set(testRotationController.calculate(getIntegratedAngle().getRadians(), angularSetPoint));

    double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond /
        (Swerve.WHEEL_DIAMETER / 2.0);
    if (RobotState.isAutonomous()) {
      driveMotor.setVoltage(Swerve.DRIVE_KF.calculate(angularVelolictySetpoint));

    } else {

      driveMotor.set(optimizedDesiredState.speedMetersPerSecond / Swerve.MAX_VELOCITY_METERS_PER_SECOND);
    }
  }

  public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState, boolean isAutoBalancing) {
    if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());

    double angularSetPoint = placeInAppropriate0To360Scope(
        optimizedDesiredState.angle.getRadians());

    rotationMotor.set(testRotationController.calculate(getIntegratedAngle().getRadians(), angularSetPoint));

    double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond /
        (Swerve.WHEEL_DIAMETER / 2.0);
    if (RobotState.isAutonomous() || isAutoBalancing == true) {
      driveMotor.setVoltage(Swerve.driveFF.calculate(angularVelolictySetpoint));

    } else {

      driveMotor.set(optimizedDesiredState.speedMetersPerSecond / Swerve.MAX_VELOCITY_METERS_PER_SECOND);
    }
  }

  public void resetEncoders() {

    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);

  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }
}