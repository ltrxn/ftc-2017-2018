## Liwen's Code

This folder includes all of my code. 

## File Description
* R1 = Red alliance position closest to relic drop off area (top right)
* R2 = Red alliance position away from relic drop off area (bottom right)
* B1 = Blue alliance position closest to relic drop off area (top left)
* B2 = Blue alliance position away from relic drop off area (bottom left)

## Class Info
* (A)AutonomousR1ColorDrive = Ideal autonomous code that uses a color sensor to reach position.
* (A)AutonomousR1EncoderDrive = Ideal autonomous code that uses only encoders to reach position.
* (A)AutonomousForTesting = Test distance and random stuff in this file.

* (T)TeleOpForTesting = This teleop is driven by tank drive.
* (T)TeleOpIdeal = Ideal teleop code for real thing (mecanum drive).

* HardwareMichaelScott = Hardware of the robot. This class initializes all the motors, servos, and sensors.

You can use the hardware class by this code:
```
//in the class
HardwareMichaelScott robot = new HardwareMichaelScott();
//in the method
robot.init(hardwareMap);
robot.exampleMotor.setPosition(.4);
```