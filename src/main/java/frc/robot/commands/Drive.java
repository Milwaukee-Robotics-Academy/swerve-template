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
    private double previousHeading = 0;


    public Drive(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedReduction, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.m_speedReduction = speedReduction;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  //      desiredHeading = s_Swerve.getYaw().getDegrees();
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

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVal * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * m_speedReduction.getAsDouble(),
                strafeVal * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * m_speedReduction.getAsDouble(),
                rotationVal * Constants.Swerve.MAX_ANGULAR_RADIANS_PER_SECOND * m_speedReduction.getAsDouble(),
                s_Swerve.getYaw());


        if (Constants.Swerve.DRIFT_CORRECTION) {
            speeds.omegaRadiansPerSecond += driftCorrection(speeds);
        }

        /* Drive */
        s_Swerve.drive(speeds);

    }

    public double driftCorrection(ChassisSpeeds speeds){
        double desiredHeading = s_Swerve.getPose().getRotation().getDegrees();
        double correction = 0;
        double translationSpeeds = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
        
        if(Math.abs(speeds.omegaRadiansPerSecond) <= 0.0 && translationSpeeds >= 0) {
            correction = driftCorrectionPID.calculate( desiredHeading, previousHeading);
        } 
        previousHeading = desiredHeading;
        return correction;
    }

    @Override
    public void end(boolean interrupted) {
        driftCorrectionPID.reset();
        s_Swerve.drive(new ChassisSpeeds());
    }
}