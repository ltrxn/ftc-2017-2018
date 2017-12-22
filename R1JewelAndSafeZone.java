package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Trxn on 12/18/2017.
 */

@Autonomous(name="R1/Jewel/SZ", group="testing")

public class R1JewelAndSafeZone extends LinearOpMode {
    //Robot
    HardwareMichaelScott robot = new HardwareMichaelScott();
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    //Encoders
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double ANDYMARK_TICKS_PER_REV = 1120; //# of ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = .5;   //Since gears go from big to small, one rotation of the gear is actually only half a rotation of the wheel
    static final double WHEEL_DIAMETER_INCHES = 4;   //Diameter of the wheel
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); //# of ticks to be rotated to drive an inch

    //Time
    private ElapsedTime runtime = new ElapsedTime();

    //Values
    float hsvValues[] = {0F, 0F, 0F}; //holds hue, saturation, and value information
    final float values[] = hsvValues; //reference to ^^
    final double SCALE_FACTOR = 255; //amplify the difference
    int redSensor;
    int blueSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");
        telemetry.addData(">  ", "اضغط على تشغيل لبدء");
        telemetry.update();
        robot.resetEncoders();
        waitForStart();
        robot.lowerJewelKnockerRight();
        sleep(1500);
        redSensor = sensorColor.red();
        blueSensor = sensorColor.blue();
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.update();

        if (blueSensor<redSensor && redSensor > 20) {
            encoderDrive(.06, -5, 5, 5);
            encoderDrive(.06, 4, -4, 5);
        } else {
            encoderDrive(.1, 5, -5, 5);
            encoderDrive(.1, -4, 4, 5);
        }
        sleep(1000);
        robot.raiseJewelKnockerRight();
        sleep(1500);

        //encoderDrive(.5,70,70,5);
        robot.driveForward(1);
        sleep(1750);
        robot.stopDriving();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.resetEncoders();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.driveForward(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Goal Position", "%7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Current Position", "%7d :%7d :%7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.stopDriving();

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
