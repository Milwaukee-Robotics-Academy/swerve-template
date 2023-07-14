package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RotateToHeading extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private double rotation;
    private DoubleSupplier suppliedHeading;
    private DoubleSupplier m_speedReduction;
    private PIDController headingCorrectionPID = new PIDController(0.09, 0.00, 0.00, 0.04);
    private double previousHeading = 0;


    public RotateToHeading(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, double heading, DoubleSupplier speedReduction, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotation = heading;
        this.m_speedReduction = speedReduction;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  //      desiredHeading = s_Swerve.getYaw().getDegrees();
        headingCorrectionPID.enableContinuousInput(-180, 180);
        // headingCorrectionPID.setTolerance(10, 10);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        /* Sup stands for suppliers, Val stands value, */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controls.AXIS_DEADZONE);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controls.AXIS_DEADZONE);
        double rotationVal = MathUtil.applyDeadband(headingCorrectionPID.calculate(s_Swerve.getPose().getRotation().getDegrees(), rotation), Constants.Controls.AXIS_DEADZONE);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVal * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * m_speedReduction.getAsDouble(),
                strafeVal * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * m_speedReduction.getAsDouble(),
                rotationVal * Constants.Swerve.MAX_ANGULAR_RADIANS_PER_SECOND * m_speedReduction.getAsDouble(),
                s_Swerve.getYaw());

        /* Drive */
        s_Swerve.drive(speeds);

    }

    @Override
    public void end(boolean interrupted) {
        headingCorrectionPID.reset();
        s_Swerve.drive(new ChassisSpeeds());
    }
}