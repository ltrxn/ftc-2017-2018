## Liwen's Code

This folder includes all of my code. 

## File Description
* R1 = Red alliance position closest to relic drop off area (top right)
* R2 = Red alliance position away from relic drop off area (bottom right)
* B1 = Blue alliance position closest to relic drop off area (top left)
* B2 = Blue alliance position away from relic drop off area (bottom left)

## Class Info
* AutonomousR1ColorDrive = Ideal autonomous code that uses a color sensor to reach position.
* AutonomousR1EncoderDrive = Ideal autonomous code that uses only encoders to reach position.
* AutonomousForTesting = Test distance and random stuff in this file.

* TeleOpForTesting = This teleop is driven by tank drive.
* TeleOpIdeal = Ideal teleop code for real thing (mecanum drive).

* HardwareMichaelScott = Hardware of the robot. This class initializes all the motors, servos, and sensors. Initialize using nameOfTheRobot.init(hardwareMap);