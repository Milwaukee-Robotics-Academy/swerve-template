package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DrivePath extends PPSwerveControllerCommand {
    private Swerve m_swerveBase;
    private Pose2d m_initialPose;
    private boolean m_resetPostion;

    public DrivePath(Swerve swerve,
        PathPlannerTrajectory path,
        PathConstraints constraints,
        boolean stopAtEnd,
        boolean resetPosition) {
            super(path,
                swerve::getPose, 
                new PIDController(Constants.AutoConstants.X_CONTROLLER_KP, 0, 0),
                new PIDController(Constants.AutoConstants.Y_CONTROLLER_KP, 0, 0),
                getThetaController(),
                swerve::drive,
                true,
                swerve);

        m_swerveBase = swerve;
        m_resetPostion = resetPosition;
        }
        
        @Override
        public void initialize() {
            super.initialize();
            
            if (m_resetPostion)
            {
                m_swerveBase.setHeading(m_initialPose.getRotation().getDegrees());
                m_swerveBase.resetOdometry(m_initialPose);
            }   
        }
    //     private static TrajectoryConfig getDefaultTrajectoryConfig(Swerve swerve, boolean stopAtEnd) {
    //             // Create config for trajectory
    //             TrajectoryConfig config = new TrajectoryConfig(
    //                 AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
    //                 AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
    //         // Add kinematics to ensure max speed is actually obeyed
    //         config.setKinematics(swerve.getKinematics());
    //         if (stopAtEnd)
    //             config.setEndVelocity(0.0);
    //         return config;
    // }

        private static PIDController getThetaController() {
            var thetaController = new PIDController(Constants.AutoConstants.THETA_CONTROLLER_KP, 0, 0);
            thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
            return thetaController;
        }

        // private static Trajectory getTrajectory(List<Translation2d> waypoints, TrajectoryConfig config) {
        //     if (waypoints.size() < 2) {
        //         return TrajectoryGenerator.generateTrajectory(
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 List.of(),
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 config);
        //     }
        //     var rotation = getTrajectoryRotation(waypoints);
        //     var interiorPoints = waypoints.subList(1, waypoints.size() - 1);
        //     var startPoint = waypoints.get(0);
        //     var endPoint = waypoints.get(waypoints.size() - 1);
        //     return TrajectoryGenerator.generateTrajectory(
        //             new Pose2d(startPoint.getX(), startPoint.getY(), rotation),
        //             interiorPoints,
        //             new Pose2d(endPoint.getX(), endPoint.getY(), rotation),
        //             config);
        // }
        // private static Rotation2d getTrajectoryRotation(List<Translation2d> waypoints) {
        //     if (waypoints.size() < 2)
        //         return Rotation2d.fromDegrees(0);
        //     var startPoint = waypoints.get(0);
        //     var endPoint = waypoints.get(waypoints.size() - 1);
        //     double xdist = endPoint.getX() - startPoint.getX();
        //     double ydist = endPoint.getY() - startPoint.getY();
        //     double angle = Math.atan2(ydist, xdist);
        //     return new Rotation2d(angle);
        // }
    
}
