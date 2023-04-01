package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.DriveSegment;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        /* Controllers */
        private final Joystick driver = new Joystick(0);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Driver Buttons */
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
        private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
        private final JoystickButton slow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
        private final JoystickButton modAbsoluteOffSet = new JoystickButton(driver,
                        XboxController.Button.kStart.value);

        /* Subsystems */
        private final Swerve swerve = new Swerve();
        SendableChooser<Command> autoChooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                swerve.setDefaultCommand(
                                new Drive(
                                                swerve,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> getDesiredHeading(),
                                                () -> speedReduction(),
                                                () -> robotCentric.getAsBoolean()));

                // driverUP = new POVButton(driver, 0);
                // driverRIGHT = new POVButton(driver, 90);
                // driverDOWN = new POVButton(driver, 180);
                // driverLEFT = new POVButton(driver, 270);
                // operatorUP = new POVButton(operator, 0);
                // operatorRIGHT = new POVButton(operator, 90);
                // operatorDOWN = new POVButton(operator, 180);
                // operatorLEFT = new POVButton(operator, 270);
                // Configure the button bindings
                configureButtonBindings();

                autoChooser.setDefaultOption("Do nothing", new InstantCommand());
                updateAutoChoices();
                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.putData(swerve);
                Shuffleboard.getTab("Autonomous").add(autoChooser).withSize(2, 1);

        }

        /**
         * Return how much the speed should be reduced.
         * 
         * @return speed reduced percent
         */
        public double speedReduction() {
                if (slow.getAsBoolean()) {
                        SmartDashboard.putBoolean("SpeedReduced", true);
                        return 0.4;
                } else {
                        SmartDashboard.putBoolean("SpeedReduced", false);
                        return 1.0;
                }
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
                modAbsoluteOffSet.onTrue(new InstantCommand(() -> swerve.resetModulesToAbsolute()));
        }

        public double getDesiredHeading() {
                // if (driverUP.getAsBoolean() || operatorUP.getAsBoolean()) {
                // return 0.0;
                // } else if (driverRIGHT.getAsBoolean() || operatorRIGHT.getAsBoolean()) {
                // return -90.0;
                // } else if (driverDOWN.getAsBoolean() || operatorDOWN.getAsBoolean()) {
                // return 180.0;
                // } else if (driverLEFT.getAsBoolean() || operatorLEFT.getAsBoolean()) {
                // return 90.0;
                // } else
                return 999;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Command Selected from Shuffleboard
                return autoChooser.getSelected();

        }

        public void teleopInit() {
                swerve.resetModulesToAbsolute();
        }

        public void autonomousInit() {
                swerve.resetModulesToAbsolute();
        }

        public void disabledInit() {

        }

        public void updateAutoChoices() {
                /**
                 * Add potential autos to the chooser
                 */
                autoChooser.addOption("Drive for 1 meter", (new DriveSegment(swerve,
                                List.of(
                                                new Translation2d(0, 0),
                                                new Translation2d(1, 0)),
                                Rotation2d.fromDegrees(0),
                                Rotation2d.fromDegrees(0),
                                true,
                                true).withTimeout(2.5)));
        }
}
