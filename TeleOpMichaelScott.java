package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Locale;

/**
 * Created by Trxn on 11/7/2017.
 */

@TeleOp(name = "Drive Michael Scott", group = "Michael Scott")
//@Disabled
public class TeleOpMichaelScott extends LinearOpMode {

    HardwareMichaelScott robot = new HardwareMichaelScott();


    /******VALUES******/
    int jewelKnockerCounter;
    int target;
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)
    private double rightGlyphClawOpen = .8;
    private double rightGlyphClawClose = .5;
    private double leftGlyphClawOpen = .2;
    private double leftGlyphClawClose = .5;

    private double currentRight = rightGlyphClawOpen;
    private double currentLeft = leftGlyphClawOpen;
    private double lightPower = 0;
    /******VALUES******/
    private ElapsedTime runtime = new ElapsedTime(); // For time out (encoder drive)
    private double rightClawPos = 0;
    private double leftClawPos = 0;

    //gyro
    private Orientation angles;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        jewelKnockerCounter = 1;
        telemetry.addData(">", "Initialization Complete");
        telemetry.update();
        telemetry.setMsTransmissionInterval(100);
        waitForStart();


        while (opModeIsActive()) {

            angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //robot.relicTrackables.activate(); //activate vuforia

            //Gamepad 1 - Left Joystick - Strafes robot
            double findRadius = Math.hypot(-gamepad1.right_stick_x, -gamepad1.right_stick_y);
            double findAngle = (Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 3.5);
            double leftY = gamepad1.left_stick_y / 1.2;
            final double v1 = findRadius * Math.cos(findAngle) - leftY;
            final double v2 = findRadius * Math.sin(findAngle) - leftY;
            final double v3 = findRadius * Math.sin(findAngle) - leftY;
            final double v4 = findRadius * Math.cos(findAngle) - leftY;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);


            //Gamepad 2 - Left and Right Triggers = precise movement
            while (gamepad2.left_trigger>0) {
                robot.leftFront.setPower(.1);
                robot.leftBack.setPower(.1);
            }
            while (gamepad2.right_trigger>0) {
                robot.rightFront.setPower(.1);
                robot.rightBack.setPower(.1);
            }

            //Gamepad 2 - Right Joystick - moves pulley
            double pulleyPowerRight = -gamepad2.right_stick_y;
            robot.pulley.setPower(scaleInput(pulleyPowerRight)); //scaled

            //Gamepad 1 - Right Trigger - Robot turns clockwise
            while (gamepad1.right_trigger > 0) {
                double speed = scaleInput(gamepad1.right_trigger);
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(-speed);
                robot.leftBack.setPower(speed);
                robot.rightBack.setPower(-speed);
            }

            //Gamepad 1 - Right Trigger - Robot turns clockwise
            while (gamepad1.left_trigger > 0) {
                double speed = scaleInput(gamepad1.left_trigger);
                robot.leftFront.setPower(-speed);
                robot.rightFront.setPower(speed);
                robot.leftBack.setPower(-speed);
                robot.rightBack.setPower(speed);
            }

            //Gamepad 1/2 - Right Bumper - Claws open
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                currentRight = rightGlyphClawOpen;
                currentLeft = leftGlyphClawOpen;
                lightPower = 0;
            }
            //Gamepad 1/2 - Left Bumper - Claws closes
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                currentRight = rightGlyphClawClose;
                currentLeft = leftGlyphClawClose;
                lightPower = .2;
            }

            //Gamepad 1/2 - dpad down - Put claw in middle position
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                currentRight = .60;
                currentLeft = .40;
            }

            if (gamepad2.dpad_right) {
                pulleyByTicks(0);
            }
            if (gamepad2.dpad_up) {
                pulleyByTicks(1500);
            }
            if (gamepad2.dpad_left) {
                pulleyByTicks(2500);
            }


            robot.glyphClawRight.setPosition(currentRight);
            robot.glyphClawLeft.setPosition(currentLeft);

            //Gamepad 1/2 - a - Raises jewel knocker right
            if (gamepad1.a || gamepad2.a) {
                robot.raiseJewelKnockerRight();
            }

            //Gamepad 1/2 - b - Lowers jewel knocker right
            if (gamepad1.b || gamepad2.b) {
                robot.lowerJewelKnockerRight();
            }

            if (gamepad1.x || gamepad2.x) {
                robot.resetEncoders();
            }



                robot.lights.setPower(lightPower);

            //telemetry
            telemetry.addData("Right jewel knocker position", robot.jewelKnockerRight.getPosition());
            telemetry.addData("Left glyph claw position", robot.glyphClawLeft.getPosition());
            telemetry.addData("Right glyph claw position", robot.glyphClawRight.getPosition());
            telemetry.addData("Encoder rightFront", robot.rightFront.getCurrentPosition());
            telemetry.addData("Encoder rightBack", robot.rightBack.getCurrentPosition());
            telemetry.addData("Encoder leftFront", robot.leftFront.getCurrentPosition());
            telemetry.addData("Encoder leftBack", robot.leftBack.getCurrentPosition());
            telemetry.addData("Encoder pulley", robot.pulley.getCurrentPosition());
            telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            idle();
        }
    }

    //Makes joystick easier to control
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }

    private void pulleyByTicks(int ticks) {
        robot.pulley.setTargetPosition(ticks);

        robot.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.pulley.setPower(.6);
        while (opModeIsActive() &&
                (robot.pulley.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Goal Position", "%7d", ticks);
            telemetry.addData("Current Position", "%7d",
                    robot.pulley.getCurrentPosition());
            telemetry.update();
        }
        robot.pulley.setPower(0);

        robot.pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);   // optional pause after each move
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
