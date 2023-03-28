package frc.robot.autos;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class AutoBalance extends CommandBase {    
    private Swerve s_Swerve;

    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double driveSpeed;

        if(s_Swerve.getRoll() > -5){
                driveSpeed = -.20;
            } else if(s_Swerve.getRoll() < -5){
            driveSpeed = .20;
            } else {driveSpeed = 0;}

        /* Drive */
        s_Swerve.drive(
            new Translation2d(driveSpeed * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND, 0),
            0, 
            true, 
            true
        );
    }

}