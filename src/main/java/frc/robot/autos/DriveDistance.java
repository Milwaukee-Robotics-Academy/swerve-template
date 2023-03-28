package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;

public class DriveDistance extends DriveSegment {
    public DriveDistance(Swerve swerve, double Distance) {
        super(swerve,
                List.of(new Translation2d(0,0),new Translation2d(Distance, 0)),
                new Rotation2d(0),
                new Rotation2d(0),
                true,
                true);
    }


}
