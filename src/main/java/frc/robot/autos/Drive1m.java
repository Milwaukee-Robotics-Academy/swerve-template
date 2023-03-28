package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class Drive1m extends DriveSegment {
    public Drive1m(Swerve swerve) {
        super(swerve,
                List.of(LOCAL_WP_1.getTranslation(), LOCAL_WP_2.getTranslation()),
                LOCAL_WP_1.getRotation(),
                LOCAL_WP_2.getRotation(),
                true,
                true);
    }

    static final Pose2d LOCAL_WP_1 = new Pose2d(0.0, 0.0, new Rotation2d(0));
    static final Pose2d LOCAL_WP_2 = new Pose2d(1.0, 0.0, new Rotation2d(0));

}
