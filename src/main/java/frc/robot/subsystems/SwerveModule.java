package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

/**
 * Class to represent and handle a swerve module
 * A module's state is measured by a CANCoder for the absolute position,
 * integrated rotation NEO encoder for relative position
 * for both rotation and linear movement
 */
public class SwerveModule extends SubsystemBase {

  public final String description;
  public final int number;

  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;

  private final CANSparkMax rotationMotor;
  private final RelativeEncoder rotationEncoder;

  private final CANCoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final double canCoderOffset;

  /**
   * PID Controllers to control the drive motor and rotation motors
   */
  private final SparkMaxPIDController rotationController;
  private final SparkMaxPIDController driveController;

  /**
   * Helper class to provide the FeedForward outputs for velocity controlled drive
   * motor
   */
  private final SimpleMotorFeedforward driveFeedforward;

  /**
   * Used to keep an angle if not commanded to change it.
   */
  private double lastAngle;

  /**
   * Constructor for the Module. take in the constraints and use it to set up the
   * sepcific module
   * 
   * @param constants
   */
  public SwerveModule(SwerveModuleConstants constants) {
    this.number = constants.number;
    this.description = constants.description;

    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV,
        Constants.Swerve.DRIVE_KA);

    rotationMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    rotationEncoder = rotationMotor.getEncoder();
    rotationController = rotationMotor.getPIDController();

    canCoder = new CANCoder(constants.cancoderID);
    canCoderOffset = Units.degreesToRadians(constants.angleOffset);

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  /**
   * Based on the passed in state, this method will give commands to the motors.
   * If open loop, pass the speed calculated by the commanded speed / max speed.
   * If Closed loop, set the drive setpoint to the velocity
   * 
   * @param state
   * @param isOpenLoop
   */
  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to.
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    state = SwerveModuleState.optimize(state, getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
      driveController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    } else {
      driveController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0,
          driveFeedforward.calculate(state.speedMetersPerSecond));
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
        ? lastAngle
        : state.angle.getRadians();
    SmartDashboard.putNumber(description + " Angle", angle);
    rotationController.setReference(angle, CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(rotationEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  /**
   * 
   * @return unsigned (0-360) angle of the Absolute (CANCoder)
   */
  public double getCanCoderDegrees() {

    return canCoder.getAbsolutePosition();
  }

  /**
   * This used to set the encoder on the turn motor to be in sync with the
   * absolute encoder.
   * 
   * @return CANCoder minus the offset
   */
  public Rotation2d getCanCoderWithOffset() {

    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - canCoderOffset)
        % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);
  }

  public double getRotationEncoderDegrees() {

    double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

    if (unsignedAngle < 0)
      unsignedAngle += 2 * Math.PI;

    return Units.radiansToDegrees(unsignedAngle);
   
  }

  /**
   * Provides the sensors in a standard ServeModulePosition object
   * 
   * @return Current Position of sensors for this module
   */
  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(rotationEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getDriveMotorTemperature() {
    return driveMotor.getMotorTemperature();
  }

  public double getLastAngle() {
    return Units.radiansToDegrees(lastAngle);
  }
  /**
   * Initialize the integrated rotation NEO encoder to the offset (relative to
   * home
   * position)
   * measured by the CANCoder
   */
  public void initRotationOffset() {

    // rotationEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition()
    // - canCoderOffsetDegrees));
    rotationEncoder.setPosition(getCanCoderWithOffset().getRadians());
    System.out.println(this.description+" "+getCanCoderWithOffset().getRadians());
  }



  /**
   * The method used to set up the SparkMaxs and the Cancoders initially
   */
  private void configureDevices() {
    // Drive motor configuration.
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERSION);
    driveMotor.setIdleMode(Constants.Swerve.DRIVE_IDLE_MODE);
    driveMotor.setOpenLoopRampRate(Constants.Swerve.OPEN_LOOP_RAMP);
    driveMotor.setClosedLoopRampRate(Constants.Swerve.CLOSED_LOOP_RAMP);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CURRENT_LIMIT);

    driveController.setP(Constants.Swerve.DRIVE_KP);
    driveController.setI(Constants.Swerve.DRIVE_KI);
    driveController.setD(Constants.Swerve.DRIVE_KD);
    driveController.setFF(Constants.Swerve.DRIVE_KF);

    driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    rotationMotor.restoreFactoryDefaults();
    rotationMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERSION);
    rotationMotor.setIdleMode(Constants.Swerve.ANGLE_IDLE_MODE);
    rotationMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CURRENT_LIMIT);

    rotationController.setP(Constants.Swerve.ANGLE_KP);
    rotationController.setI(Constants.Swerve.ANGLE_KI);
    rotationController.setD(Constants.Swerve.ANGLE_KD);
    rotationController.setFF(Constants.Swerve.ANGLE_KF);

    rotationController.setPositionPIDWrappingEnabled(true);
    rotationController.setPositionPIDWrappingMaxInput(2 * Math.PI);
    rotationController.setPositionPIDWrappingMinInput(0);

    rotationEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_ROTATIONS_TO_RADIANS);
    rotationEncoder.setVelocityConversionFactor(Constants.Swerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
    rotationEncoder.setPosition(getCanCoderWithOffset().getRadians());


    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // canCoderConfiguration.sensorDirection = Constants.Swerve.CANCODER_INVERSION;
    // canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    // canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

    canCoder.configFactoryDefault();
    canCoder.configAllSettings(canCoderConfiguration);
  }

  public void resetEncoders() {

    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);

  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(this.description + " #" + this.number);
    builder.addDoubleProperty("CANCoder", this::getCanCoderDegrees, null);
    builder.addDoubleProperty("Rotation", this::getRotationEncoderDegrees, null);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("DriveTemp", this::getDriveMotorTemperature, null);
    builder.addDoubleProperty("LastAngle", this::getLastAngle, null);
    // builder.addDoubleProperty("VelocitySetpoint", this::getMeasurement, null);

    // builder.addDoubleProperty("Distance", this::getError, null);
    // builder.addBooleanProperty("atSetpoint", this::atSetpoint, null);

  }
}