package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Trxn on 10/17/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpIdeal", group = "real")
@Disabled

public class TeleOpIdeal extends LinearOpMode {

    //Wheel Motors
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    //Motor to pull up and down the glyph claw
    public DcMotor glyphPully;

    //Servos
    public Servo glyphClawRight;
    public Servo glyphClawLeft;
    public Servo jewelKnockerRight;
    public Servo jewelKnockerLeft;

    //Sensors
    public ColorSensor rightColorSensor;
    public ColorSensor leftColorSensor;
    public GyroSensor sensorGyro;
    public ModernRoboticsI2cGyro mrGyro;


    @Override
    public void runOpMode() throws InterruptedException {

        //Declaring hardware
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        glyphPully = hardwareMap.dcMotor.get("glyphPully");

        glyphClawLeft = hardwareMap.servo.get("glyphClawLeft");
        glyphClawRight = hardwareMap.servo.get("glyphClawRight");
        jewelKnockerLeft = hardwareMap.servo.get("jewelKnockerLeft");
        jewelKnockerRight = hardwareMap.servo.get("jewelKnockerRight");

        rightColorSensor = hardwareMap.colorSensor.get("colorRihgt");
        leftColorSensor = hardwareMap.colorSensor.get("colorLeft");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        //Setting up motors
        rightFront.setDirection(DcMotor.Direction.REVERSE); //reverse the
        rightBack.setDirection(DcMotor.Direction.REVERSE);  //right motors


        waitForStart();


        while (opModeIsActive()) {

            /*
            MECANUM DRIVE
            gamepad 1 - left and right joystick
            */
            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 3.5;
            double rightX = gamepad1.right_stick_x / 2;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(-v1);
            rightFront.setPower(-v2);
            leftBack.setPower(-v3);
            rightBack.setPower(-v4);

            /*
            PULLY CONTROL
            gamepad 2 - left and right trigger
            */
            if (gamepad1.right_trigger > .1) {
                glyphPully.setPower(.5);
            } else if (gamepad1.left_trigger > .1) {
                glyphPully.setPower(-.5);
            }

            /*
            GLYPH CLAW
            gamepad 1 - a button and b button
            */
            if (gamepad1.a) {
                closeGlyphClaw();
            }
            if (gamepad1.b) {
                openGlyphClaw();
            }

            /*
            JEWEL KNOCKER
            gampad 1 - dpad left and right
            */
            if (gamepad1.dpad_left) {
                jewelKnockerLeft.setPosition(0.1);
            }
            if (gamepad1.dpad_right) {
                jewelKnockerRight.setPosition(0.9);
            }

        }
        idle();
    }

    private void closeGlyphClaw() {
        //Closes the glyph claw to grasp the block
        double closedLeftGlyphClaw = .5;
        double closedRightGlyphClaw = .5;

        glyphClawLeft.setPosition(closedLeftGlyphClaw);
        glyphClawRight.setPosition(closedRightGlyphClaw);
    }

    private void openGlyphClaw() {
        //Open the glyph claw
        double openLeftGlyphClaw = .1;
        double openRightGlyphClaw = .9;

        glyphClawLeft.setPosition(openLeftGlyphClaw);
        glyphClawRight.setPosition(openRightGlyphClaw);
    }

}