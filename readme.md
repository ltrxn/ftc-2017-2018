## Liwen's Code

This folder includes all of my code. 

## File Description
* R1 = Red alliance position closest to relic drop off area (top right)
* R2 = Red alliance position away from relic drop off area (bottom right)
* B1 = Blue alliance position closest to relic drop off area (top left)
* B2 = Blue alliance position away from relic drop off area (bottom left)

## Class Info
* (A)B1 = autonomous for the position B1.
* (A)B2 = autonomous for the position B2.
* (A)R1 = autonomous for the position R1.
* (A)R2 = autnomous for the position R2.

* (T)TeleOpMichaelScott = Teleop

* HardwareMichaelScott = Hardware of the robot. This class initializes all the motors, servos, and sensors.

You can use the hardware class by this code:
```
//in the class
HardwareMichaelScott robot = new HardwareMichaelScott();
//in the method
robot.init(hardwareMap);
robot.exampleMotor.setPosition(.4);
```
