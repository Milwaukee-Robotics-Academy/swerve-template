package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Swerve class defines the drivetrain subsystem.
 * it includes 4 {@link frc.robot.subsystems.SwerveModule}
 */
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    public Swerve() {

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(Constants.Swerve.MOD_0_Constants),
                new SwerveModule(Constants.Swerve.MOD_1_Constants),
                new SwerveModule(Constants.Swerve.MOD_2_Constants),
                new SwerveModule(Constants.Swerve.MOD_3_Constants)
        };


        
        /**
         *  Adding a Tab "Swerve" to the Shuffleboard to display Swerve data
         */
        ShuffleboardTab swerveTab =  Shuffleboard.getTab("Swerve");
        swerveTab.add("navx", gyro).withSize(2, 2).withPosition(0, 0);
        for (SwerveModule mod : mSwerveMods) {
            swerveTab.add("Swerve:"+mod.description, mod).withSize(2, 2).withPosition((1 + mod.number)*2, 0);
        }
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getYaw(), getModulePositions());
        setHeading(0);
    }

    /**
     * This is the method that takes {@link edu.wpi.first.math.kinematics.ChassisSpeeds} and sends to each of the modules
     * It is used by the default command {@link frc.robot.commands.Drive} to drive the drivetrain
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(states,
        // Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    /**
     * Not sure if this is needed any more. Another way to pass in data on how to drive the drivetrain
     * @param translation
     * @param rotation
     * @param fieldRelative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw());

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? speeds
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        for (SwerveModule mod : mSwerveMods) {
            mod.setState(swerveModuleStates[mod.number], isOpenLoop);
        }
    }

    /**
     * Once the speeds are converted into {@link edu.wpi.first.math.kinematics.SwerveModuleState} they are passed
     * to each module.
     * 
     * Used by SwerveControllerCommand in Auto
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        for (SwerveModule mod : mSwerveMods) {
            mod.setState(desiredStates[mod.number], false);
        }
    }

    /**
     * We keep track of the "pose" of the robot with the gyro, encoders (and vision data) This returns the current state
     * 
     * @return {@link edu.wpi.first.math.geometry.Posed}
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     *  This method returns a helper {@link edu.wpi.first.math.kinematics.SwerveDriveKinematics} This takes info about our robot
     * and uses it to turn desired chassis velocities into desired states for each module
     * @return
     */
    public SwerveDriveKinematics getKinematics() {
        return Constants.Swerve.KINEMATICS;
    }

   /**
    *  resets our robots position on the field.
    * @param pose
    */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

   /**
    * 
    * @return The module states for each module
    */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.number] = mod.getState();
        }
        return states;
    }

    /**
     *  
     * @return the module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.number] = mod.getPosition();
        }
        return positions;
    }

    /**
     *  Set the Gyro heading to Zero. Used for when the robot is facing directly away from driverstation
     */
    public void zeroHeading() {
        gyro.zeroYaw();
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    /**
     * Set the heading of the robot to the param passed in
     * @param heading
     */
    public void setHeading(double heading) {
        gyro.zeroYaw();
        gyro.setAngleAdjustment(heading);
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    /**
     * 
     * @return the Yaw from the Gyro (for the NavX it is getYaw() * -1 )
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw() * -1);
    }

    /**
     * 
     * @return The Pitch of the Robot (front / back, if NavX installed correctly)
     */
    public double getPitch() {
        return gyro.getPitch();
    }

    /**
     * 
     * @return The Roll of the Robot (left / right, if NavX installed correctly)
     */
    public double getRoll() {
        return gyro.getRoll();
    }

    /**
     * Used to set the turn motor encoder = Cancoder + offset. This may need to be done
     * more freqently than on startup. Perhaps AutonomousInit and TeleopInit
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.initRotationOffset();
        }
    }

    /**
     * This runs every 20ms. Used to update the logging/dashboard.
     */
    @Override
    public void periodic() {

        if (DriverStation.isDisabled()){
            resetModulesToAbsolute();
        }

        SmartDashboard.putNumber("Pitch", this.getPitch());

        Logger.getInstance().recordOutput("Robot", (swerveOdometry.update(getYaw(), getModulePositions())));
        Logger.getInstance().recordOutput("SwerveStates", getModuleStates());


        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());

        SmartDashboard.putNumber("Roll", this.getRoll());

    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
    public Command followTrajectoryCommand(ArrayList<PathPlannerTrajectory> path, HashMap<String, Command> eventMap,
    boolean isFirstPath) {
  SmartDashboard.putNumber("intial Pose rotation", path.get(0).getInitialPose().getRotation().getDegrees());
//   m_field.getObject("traj").setTrajectory(path.get(0));

  // Create the AutoBuilder. This only needs to be created once when robot code
  // starts, not every time you want to create an auto command. A good place to
  // put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this::getPose, // Pose2d supplier
      this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.Swerve.KINEMATICS,
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    this::setModuleStates,
    eventMap,
      true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
      this // Requires this drive subsystem
  );
  return autoBuilder.fullAuto(path);
  // return new SequentialCommandGroup(
  // new InstantCommand(() -> {
  // // Reset odometry for the first path you run during auto
  // if (isFirstPath) {
  // this.resetOdometry(path[0].getInitialPose());
  // }
  // }),
  // autoBuilder.fullAuto(path));
}
    }