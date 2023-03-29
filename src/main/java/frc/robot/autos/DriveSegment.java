package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class DriveSegment extends SwerveControllerCommand{
    private Swerve m_swerveBase;
    private Pose2d m_initialPose;
    private boolean m_resetPostion;

    public DriveSegment(Swerve swerve,
        List<Translation2d> waypoints,
        Rotation2d startRotation,
        Rotation2d endRotation,
        boolean stopAtEnd,
        boolean resetPosition) {
            super(getTrajectory(waypoints, getDefaultTrajectoryConfig(swerve, stopAtEnd)),
                swerve::getPose, // Functional interface to feed supplier
                swerve.getKinematics(),
                // Position controllers
                new PIDController(Constants.AutoConstants.X_CONTROLLER_KP, 0, 0),
                new PIDController(Constants.AutoConstants.Y_CONTROLLER_KP, 0, 0),
                getThetaController(),
                () -> endRotation,
                swerve::setModuleStates,
                swerve);
        m_swerveBase = swerve;
        var firstWaypoint = waypoints.get(0);
        m_initialPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), startRotation);     
        m_resetPostion = resetPosition;
        }
        
        @Override
        public void initialize() {
            super.initialize();
            
            if (m_resetPostion)
            {
                m_swerveBase.zeroHeading(m_initialPose.getRotation().getDegrees());
                m_swerveBase.resetOdometry(m_initialPose);
            }   
        }
        private static TrajectoryConfig getDefaultTrajectoryConfig(Swerve swerve, boolean stopAtEnd) {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                    AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
            // Add kinematics to ensure max speed is actually obeyed
            config.setKinematics(swerve.getKinematics());
            if (stopAtEnd)
                config.setEndVelocity(0.0);
            return config;
    }

        private static ProfiledPIDController getThetaController() {
            var thetaController = new ProfiledPIDController(Constants.AutoConstants.THETA_CONTROLLER_KP, 0, 0, Constants.AutoConstants.THETA_CONSTRAINTS);
            thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
            return thetaController;
        }

        private static Trajectory getTrajectory(List<Translation2d> waypoints, TrajectoryConfig config) {
            if (waypoints.size() < 2) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(0, 0, new Rotation2d(0)),
                        config);
            }
            var rotation = getTrajectoryRotation(waypoints);
            var interiorPoints = waypoints.subList(1, waypoints.size() - 1);
            var startPoint = waypoints.get(0);
            var endPoint = waypoints.get(waypoints.size() - 1);
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(startPoint.getX(), startPoint.getY(), rotation),
                    interiorPoints,
                    new Pose2d(endPoint.getX(), endPoint.getY(), rotation),
                    config);
        }
        private static Rotation2d getTrajectoryRotation(List<Translation2d> waypoints) {
            if (waypoints.size() < 2)
                return Rotation2d.fromDegrees(0);
            var startPoint = waypoints.get(0);
            var endPoint = waypoints.get(waypoints.size() - 1);
            double xdist = endPoint.getX() - startPoint.getX();
            double ydist = endPoint.getY() - startPoint.getY();
            double angle = Math.atan2(ydist, xdist);
            return new Rotation2d(angle);
        }
        protected static final Pose2d WAYPOINT_START1 = new Pose2d(1.37, 1.07, Rotation2d.fromDegrees(0));
        protected static final Pose2d WAYPOINT_START2 = new Pose2d(0, 2.75, Rotation2d.fromDegrees(0));
        protected static final Pose2d WAYPOINT_START3 = new Pose2d(1.37, 4.42, Rotation2d.fromDegrees(0));
        protected static final Pose2d WAYPOINT_GP1 = new Pose2d(7.05, .91, Rotation2d.fromDegrees(0));
        protected static final Pose2d WAYPOINT_GP2 = new Pose2d(7.05, 2.13, Rotation2d.fromDegrees(90));
        protected static final Pose2d WAYPOINT_CENTER_CROSS = new Pose2d(7.05, 2.75, Rotation2d.fromDegrees(0));
        protected static final Pose2d WAYPOINT_GP3 = new Pose2d(7.05, 3.35, Rotation2d.fromDegrees(-90));
        protected static final Pose2d WAYPOINT_GP4 = new Pose2d(7.05, 4.75, Rotation2d.fromDegrees(-45));
        protected static final Pose2d WAYPOINT_CHARGESTATION = new Pose2d(-3.1, 2.75, Rotation2d.fromDegrees(0));
        protected static final Pose2d WAYPOINT_RIGHT_OF_CS = new Pose2d(2.0, 0.55, Rotation2d.fromDegrees(0));
        
       
}
