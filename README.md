# Milwaukee Robotics Academy Swerve Drive Base Template

This code was designed for the Swerve Drive Specialties MK4 and MK4i modules using NEO motors, CANCoders and NavX2 Gyro. We learned (and copied) a LOT from Team364 

## Setting Constants
All the settings that you need to change for SURE are set in the TODOs. Make sure you address all of the TODOs in order to use properly with your robot.

    - Width and Length of the Wheelbase
    - Wheel Diameter
    - Angle Motor Invert
    - Drive Motor Invert
    - CANCoder Sensor Invert
    - CAN IDs for each module
    - Angle Motor Gear Ratio
    - Drive Motor Gear Ratio
    - Angle Falcon Motor PID Values

## Drive Motor, Angle Motor, and CANCoder Invert
<br>This can always remain false, since you set your offsets in the next step such that a positive input to the drive motor will cause the robot to drive forwards.
<br>However this can be set to true if for some reason you prefer the bevel gears on the wheel to face one direction or another when setting offsets. 

```Module Specific Constants```: set the Can Id's of the motors and CANCoders for the respective modules, see the next step for setting offsets.

## Setting Offsets

For finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight. 

- Point the bevel gears of all the wheels in the same direction (either facing left or right), where a postive input to the drive motor drives the robot forward (you can use phoenix tuner to test this). If for some reason you set the offsets with the wheels backwards, you can change the ```DRIVE_MOTOR_INVERSION``` value to fix.

- Open smartdashboard (or shuffleboard and go to the smartdashboard tab), you will see 4 printouts called 
    - ```Front Left Cancoder```
    - ```Rear Left Cancoder```
    - etc... 

- If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```angleOffset``` variable in constants.
**Note:** The CANcoder values printed to smartdashboard are in degrees, when copying the values to ```angleOffset``` you must use ```Rotation2d.fromDegrees("copied value")```.

## Feedback Settings

### Angle Motor PID Values
- To tune start with a low P value (0.01).
- Multiply by 10 until the module starts oscilating around the set point
- Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module doesn't oscillate around the setpoint.
- If there is any overshoot you can add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.

### Maxes
- ```MAX_VELOCITY_METERS_PER_SECOND```: In Meters Per Second. 
- ```MAX_ANGULAR_RADIANS_PER_SECOND```: In Radians Per Second. 
For these you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.

### Robot Characterization
Get the drive characterization values (```DRIVE_KS```, ```DRIVE_KV```, ```DRIVE_KA```) by using the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock your modules straight forward, and complete the characterization as if it was a standard tank drive.

After completing characterization and inserting the KS, KV, and KA values into the code, tune the ```DRIVE_KP``` until it doesn't overshoot and doesnt oscilate around a target velocity.
Leave ```DRIVE_KI``` , ```DRIVE_KD```, and ```DRIVE_KF``` at 0.0.
