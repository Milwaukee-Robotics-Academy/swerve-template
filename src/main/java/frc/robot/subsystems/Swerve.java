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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Swerve class defines the drivetrain subsystem.
 * it includes 4 {@link frc.robot.subsystems.SwerveModule}
 */
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public GenericEntry[] canCoderValues;
    public GenericEntry[] rotationValues;
    public GenericEntry[] velocityValues;
    private DriveMode driveMode = DriveMode.NORMAL;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    public Swerve() {

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(Constants.Swerve.MOD_0_Constants),
                new SwerveModule(Constants.Swerve.MOD_1_Constants),
                new SwerveModule(Constants.Swerve.MOD_2_Constants),
                new SwerveModule(Constants.Swerve.MOD_3_Constants)
        };

        /**
         * These are entries for logging Module data
         */
        canCoderValues = new GenericEntry[4];
        rotationValues = new GenericEntry[4];
        velocityValues = new GenericEntry[4];

        /**
         * Adding a Tab "Swerve" to the Shuffleboard to display Swerve data
         */
        Shuffleboard.getTab("Swerve").add("navx", gyro).withSize(2, 2).withPosition(0, 0);
        for (SwerveModule mod : mSwerveMods) {
            canCoderValues[mod.number] = Shuffleboard.getTab("Swerve")
                    .add(mod.description + " Cancoder", mod.getCanCoderAngle().getDegrees())
                    .withPosition((2 + mod.number), 0).getEntry();
            rotationValues[mod.number] = Shuffleboard.getTab("Swerve")
                    .add(mod.description + " Integrated", mod.getState().angle.getDegrees())
                    .withPosition((2 + mod.number), 1).getEntry();
            velocityValues[mod.number] = Shuffleboard.getTab("Swerve")
                    .add(mod.description + " Velocity", mod.getState().speedMetersPerSecond)
                    .withPosition((2 + mod.number), 2).getEntry();
        }
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getYaw(), getModulePositions());
        setHeading(0);
    }

    /**
     * This is the method that takes
     * {@link edu.wpi.first.math.kinematics.ChassisSpeeds} and sends to each of the
     * modules
     * It is used by the default command {@link frc.robot.commands.Drive} to drive
     * the drivetrain
     */
    public void drive(ChassisSpeeds speeds) {
        if (isXstance()) {
            setXStance();
        } else {
            SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
            // SwerveDriveKinematics.desaturateWheelSpeeds(states,
            // Constants.Swerve.maxSpeed);
            setModuleStates(states);
        }
    }

    /**
     * Not sure if this is needed any more. Another way to pass in data on how to
     * drive the drivetrain
     * 
     * @param translation
     * @param rotation
     * @param fieldRelative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (isXstance()) {
            setXStance();
        } else {
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
    }

    /**
     * Once the speeds are converted into
     * {@link edu.wpi.first.math.kinematics.SwerveModuleState} they are passed
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
     * We keep track of the "pose" of the robot with the gyro, encoders (and vision
     * data) This returns the current state
     * 
     * @return {@link edu.wpi.first.math.geometry.Pose3d}
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * This method returns a helper
     * {@link edu.wpi.first.math.kinematics.SwerveDriveKinematics} This takes info
     * about our robot
     * and uses it to turn desired chassis velocities into desired states for each
     * module
     * 
     * @return
     */
    public SwerveDriveKinematics getKinematics() {
        return Constants.Swerve.KINEMATICS;
    }

    /**
     * resets our robots position on the field.
     * 
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
     * Sets the swerve modules in the x-stance orientation. In this orientation the
     * wheels are aligned
     * to make an 'X'. This makes it more difficult for other robots to push the
     * robot, which is
     * useful when shooting.
     */
    public void setXStance() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = getKinematics().toSwerveModuleStates(chassisSpeeds);
        states[0].angle = new Rotation2d(
                Math.PI / 2 - Math.atan(Constants.Swerve.TRACK_WIDTH / Constants.Swerve.WHEEL_BASE));
        states[1].angle = new Rotation2d(
                Math.PI / 2 + Math.atan(Constants.Swerve.TRACK_WIDTH / Constants.Swerve.WHEEL_BASE));
        states[2].angle = new Rotation2d(
                Math.PI / 2 + Math.atan(Constants.Swerve.TRACK_WIDTH / Constants.Swerve.WHEEL_BASE));
        states[3].angle = new Rotation2d(
                3.0 / 2.0 * Math.PI - Math.atan(Constants.Swerve.TRACK_WIDTH / Constants.Swerve.WHEEL_BASE));
        for (SwerveModule swerveModule : mSwerveMods) {
            swerveModule.setState(states[swerveModule.number], true);
        }
    }

    public void relax() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = getKinematics().toSwerveModuleStates(chassisSpeeds);
        states[0].angle = Rotation2d.fromDegrees(0);
        states[1].angle = Rotation2d.fromDegrees(0);
        states[2].angle = Rotation2d.fromDegrees(0);
        states[3].angle = Rotation2d.fromDegrees(0);
        for (SwerveModule swerveModule : mSwerveMods) {
            swerveModule.setState(states[swerveModule.number], true);
        }
    }

    /**
     * Puts the drivetrain into the x-stance orientation. In this orientation the
     * wheels are aligned
     * to make an 'X'. This makes it more difficult for other robots to push the
     * robot, which is
     * useful when shooting. The robot cannot be driven until x-stance is disabled.
     */
    public void enableXstance() {
        this.driveMode = DriveMode.X;
        this.setXStance();
    }

    /** Disables the x-stance, allowing the robot to be driven. */
    public void disableXstance() {
        this.driveMode = DriveMode.NORMAL;
    }

    /**
     * Returns true if the robot is in the x-stance orientation.
     *
     * @return true if the robot is in the x-stance orientation
     */
    public boolean isXstance() {
        return this.driveMode == DriveMode.X;
    }

    /**
     * Set the Gyro heading to Zero. Used for when the robot is facing directly away
     * from driverstation
     */
    public void zeroHeading() {
        gyro.zeroYaw();
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    /**
     * Set the heading of the robot to the param passed in
     * 
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
     * Used to set the turn motor encoder = Cancoder + offset. This may need to be
     * done
     * more freqently than on startup. Perhaps AutonomousInit and TeleopInit
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.initRotationOffset();
            ;
        }
    }

    /**
     * This runs every 20ms. Used to update the logging/dashboard.
     */
    @Override
    public void periodic() {

        for (SwerveModule mod : mSwerveMods) {
            canCoderValues[mod.number].setDouble(mod.getCanCoderAngle().getDegrees());
            rotationValues[mod.number].setDouble(mod.getPosition().angle.getDegrees());
            velocityValues[mod.number].setDouble(mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Pitch", this.getPitch());

        Logger.getInstance().recordOutput("Robot", (swerveOdometry.update(getYaw(), getModulePositions())));
        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());

        SmartDashboard.putNumber("Roll", this.getRoll());

    }

    private enum DriveMode {
        NORMAL,
        X
    }
}