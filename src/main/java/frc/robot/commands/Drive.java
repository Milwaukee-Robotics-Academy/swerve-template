package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Drive extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier suppliedHeading;
    private DoubleSupplier m_speedReduction;
    private PIDController driftCorrectionPID = new PIDController(0.09, 0.00, 0.00, 0.04);
    private double desiredHeading = 0;
    private double commandedHeading;

    public Drive(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup,
            DoubleSupplier commandedHeading, DoubleSupplier speedReduction, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.suppliedHeading = commandedHeading;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.m_speedReduction = speedReduction;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        desiredHeading = s_Swerve.getYaw().getDegrees();
        driftCorrectionPID.enableContinuousInput(-180, 180);
        driftCorrectionPID.setTolerance(10, 10);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        /* Sup stands for suppliers, Val stands value, */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controls.AXIS_DEADZONE);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controls.AXIS_DEADZONE);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Controls.AXIS_DEADZONE);
        if (Math.abs(this.suppliedHeading.getAsDouble()) < 181)
            commandedHeading = this.suppliedHeading.getAsDouble();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVal * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * m_speedReduction.getAsDouble(),
                strafeVal * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * m_speedReduction.getAsDouble(),
                rotationVal * Constants.Swerve.MAX_ANGULAR_RADIANS_PER_SECOND * m_speedReduction.getAsDouble(),
                s_Swerve.getYaw());

        // if d-pad desired heading is commanded, then rotate to that
        if (Math.abs(commandedHeading) < 181) {
            if (driftCorrectionPID.atSetpoint()) {
                commandedHeading = 999;
                driftCorrectionPID.reset();
            } else {
                speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(
                        s_Swerve.getYaw().getDegrees(),
                        commandedHeading);
                // keep the desired heading set to our current heading
                desiredHeading = s_Swerve.getYaw().getDegrees();
            }

        } else { // no dpad, just correct for drift
            if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0) {
                // we are turning, so set the desired and the current the same
                desiredHeading = s_Swerve.getYaw().getDegrees();
            }
            if ((Math.abs(translationVal) + Math.abs(strafeVal)) > 0) {
                // we are moving x or y, but should not be moving theta so add drift correction
                if (driftCorrectionPID.atSetpoint()) {
                    driftCorrectionPID.reset();
                } else {
                    speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(
                            s_Swerve.getYaw().getDegrees(),
                            desiredHeading);
                }
            } else {
                desiredHeading = s_Swerve.getYaw().getDegrees();
            }
        }

        /* Drive */
        s_Swerve.drive(speeds);

    }

    @Override
    public void end(boolean interrupted) {
        driftCorrectionPID.reset();
        s_Swerve.drive(new ChassisSpeeds());
    }
}