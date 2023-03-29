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

  /**
     * TODO: description
     */
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

          /**
     * TODO: description
     */
        canCoderValues = new GenericEntry[4];
        rotationValues = new GenericEntry[4];
        velocityValues = new GenericEntry[4];
// TODO: Explain
        Shuffleboard.getTab("Swerve").add("navx",gyro).withSize(2,2).withPosition(0,0);
        for(SwerveModule mod : mSwerveMods){
            canCoderValues[mod.number] = Shuffleboard.getTab("Swerve").add(mod.description + " Cancoder", mod.getCanCoderAngle().getDegrees()).withPosition((2+mod.number),0).getEntry();
            rotationValues[mod.number] = Shuffleboard.getTab("Swerve").add(mod.description + " Integrated", mod.getState().angle.getDegrees()).withPosition((2+mod.number),1).getEntry();
            velocityValues[mod.number] = Shuffleboard.getTab("Swerve").add(mod.description + " Velocity", mod.getState().speedMetersPerSecond).withPosition((2+mod.number),2).getEntry();    
        }
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getYaw(), getModulePositions());
        zeroHeading(0);
    }

    /**
     * TODO: description
     */
    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
      //  SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

// TODO: Explain
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

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        for(SwerveModule mod : mSwerveMods){
            mod.setState(swerveModuleStates[mod.number], isOpenLoop);
        }
    }

    /**
     *  TODO: Explain
     * 
     *  Used by SwerveControllerCommand in Auto
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setState(desiredStates[mod.number], false);
        }
    }    

    // TODO: Explain
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    // TODO: Explain
    public SwerveDriveKinematics getKinematics(){
        return Constants.Swerve.KINEMATICS;
    }

    // TODO: Explain
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    // TODO: Explain
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.number] = mod.getState();
        }
        return states;
    }

    // TODO: Explain
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.number] = mod.getPosition();
        }
        return positions;
    }

    // TODO: Explain
    public void zeroHeading(){
        gyro.zeroYaw();
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    // TODO: Explain (and rename)
    public void zeroHeading(double heading){
        gyro.zeroYaw();
        gyro.setAngleAdjustment(heading); 
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    // TODO: Explain
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw()*-1);
    }

    // TODO: Explain
    public double getPitch() {
        return gyro.getPitch();
    }
    // TODO: Explain
    public double getRoll() {
        return gyro.getRoll();
    }
    
    // TODO: Explain
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.initRotationOffset();;
        }
    }

    // TODO: Explain
    @Override
    public void periodic(){


        for(SwerveModule mod : mSwerveMods){
            canCoderValues[mod.number].setDouble(mod.getCanCoderAngle().getDegrees());
            rotationValues[mod.number].setDouble(mod.getPosition().angle.getDegrees());
            velocityValues[mod.number].setDouble(mod.getState().speedMetersPerSecond);    
        }



        SmartDashboard.putNumber("Pitch", this.getPitch());

        Logger.getInstance().recordOutput("Robot",(swerveOdometry.update(getYaw(), getModulePositions())));
        SmartDashboard.putNumber("Yaw",getYaw().getDegrees());



        SmartDashboard.putNumber("Roll", this.getRoll());
        

    }
}