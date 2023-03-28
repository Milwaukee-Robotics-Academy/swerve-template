package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public GenericEntry[] canCoderValues;
    public GenericEntry[] rotationValues;
    public GenericEntry[] velocityValues;
    
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    public Swerve() {

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(Constants.Swerve.MOD_0_Constants),
            new SwerveModule(Constants.Swerve.MOD_1_Constants),
            new SwerveModule(Constants.Swerve.MOD_2_Constants),
            new SwerveModule(Constants.Swerve.MOD_3_Constants)
        };
        canCoderValues = new GenericEntry[4];
        rotationValues = new GenericEntry[4];
        velocityValues = new GenericEntry[4];

        Shuffleboard.getTab("Swerve").add("navx",gyro).withSize(2,2).withPosition(0,0);

        for(SwerveModule mod : mSwerveMods){
            canCoderValues[mod.number] = Shuffleboard.getTab("Swerve").add(mod.description + " Cancoder", mod.getCanCoderAngle().getDegrees()).withPosition((2+mod.number),0).getEntry();
            rotationValues[mod.number] = Shuffleboard.getTab("Swerve").add(mod.description + " Integrated", mod.getIntegratedAngle().getDegrees()).withPosition((2+mod.number),1).getEntry();
            velocityValues[mod.number] = Shuffleboard.getTab("Swerve").add(mod.description + " Velocity", mod.getCurrentVelocityMetersPerSecond()).withPosition((2+mod.number),2).getEntry();    
        }
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getYaw(), getModulePositions());
        zeroHeading(0);
    }

    /**
     * Drift Correction driving
     */
    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
      //  SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
       
       ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
        );

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? speeds
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveDriveKinematics getKinematics(){
        return Constants.Swerve.KINEMATICS;
    }
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroHeading(){
        gyro.zeroYaw();
        swerveOdometry.update(getYaw(), getModulePositions());
    }
    public void zeroHeading(double heading){
        gyro.zeroYaw();
        gyro.setAngleAdjustment(heading); 
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw()*-1);
    }
    public double getPitch() {
        return gyro.getPitch();
    }
    public double getRoll() {
        return gyro.getRoll();
    }
    
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public void limitCanCoderTraffic(boolean limit){
        if (limit){
            for(SwerveModule mod : mSwerveMods){
                mod.limitCanCoderTraffic(true);
            }
            resetModulesToAbsolute();
        } else {
            for(SwerveModule mod : mSwerveMods){
                mod.limitCanCoderTraffic(false);
            }
        }
    
    }
    
    @Override
    public void periodic(){


        for(SwerveModule mod : mSwerveMods){
            canCoderValues[mod.moduleNumber].setDouble(mod.getCanCoderAngle().getDegrees());
            rotationValues[mod.moduleNumber].setDouble(mod.getPosition().angle.getDegrees());
            velocityValues[mod.moduleNumber].setDouble(mod.getState().speedMetersPerSecond);    
        }



        SmartDashboard.putNumber("Pitch", this.getPitch());

        Logger.getInstance().recordOutput("Robot",(swerveOdometry.update(getYaw(), getModulePositions())));
        SmartDashboard.putNumber("Yaw",getYaw().getDegrees());



        SmartDashboard.putNumber("Roll", this.getRoll());
        

    }
}