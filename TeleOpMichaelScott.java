package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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
    private double rightGlyphClawOpen = .7;
    private double rightGlyphClawClose = .25;
    private double leftGlyphClawOpen = .35;
    private double leftGlyphClawClose = .7;

    private double currentRight = rightGlyphClawOpen;
    private double currentLeft = leftGlyphClawOpen;
    /******VALUES******/
    private ElapsedTime runtime = new ElapsedTime(); // For time out (encoder drive)
    private double rightClawPos = 0;
    private double leftClawPos = 0;




    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        jewelKnockerCounter = 1;
        //telemetry.addData(">", "초기화가 완료되었습니다 (Initialization is finished)");
        telemetry.addData(">", "Инициализация завершена!");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            //Gamepad 1 - Left Joystick - Strafes robot
            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 3.5;
            double rightY = gamepad1.right_stick_y / 2;
            final double v1 = r * Math.cos(robotAngle) - rightY;
            final double v2 = r * Math.sin(robotAngle) - rightY;
            final double v3 = r * Math.sin(robotAngle) - rightY;
            final double v4 = r * Math.cos(robotAngle) - rightY;

            robot.leftFront.setPower(v1);
            //robot.leftFront.setPower(v1*.90);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            //robot.leftBack.setPower(v3*.88);
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
            float pulleyPowerRight = -gamepad2.right_stick_y;
            pulleyPowerRight *= .3;
            robot.pulley.setPower(pulleyPowerRight);

            //Gamepad 1 - Right Trigger - Robot turns clockwise
            while (gamepad1.right_trigger > 0) {
                double speed = gamepad1.right_trigger;
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(-speed);
                robot.leftBack.setPower(speed);
                robot.rightBack.setPower(-speed);
            }

            //Gamepad 1 - Right Trigger - Robot turns clockwise
            while (gamepad1.left_trigger > 0) {
                double speed = gamepad1.left_trigger;
                robot.leftFront.setPower(-speed);
                robot.rightFront.setPower(speed);
                robot.leftBack.setPower(-speed);
                robot.rightBack.setPower(speed);
            }

            //Gamepad 1/2 - Right Bumper - Claws open
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                currentRight = rightGlyphClawOpen;
                currentLeft = leftGlyphClawOpen;
            }
            //Gamepad 1/2 - Left Bumper - Claws closes
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                currentRight = rightGlyphClawClose;
                currentLeft = leftGlyphClawClose;
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



            /* If we get a left jewel knocker
            if (gamepad2.x){
                robot.raiseJewelKnockerLeft();
            }

            if (gamepad2.y) {
                robot.lowerJewelKnockerLeft();
            }
            */

            /* Vuforia Testing Code
            robot.relicTrackables.activate();
            RelicRecoveryVuMark vuMark = robot.getVuMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "Is not visable");
            }
            */

            if (gamepad1.dpad_up) {
                robot.resetEncoders();
            }


            telemetry.addData("Right jewel knocker position", robot.jewelKnockerRight.getPosition());
            telemetry.addData("Left glyph claw position", robot.glyphClawLeft.getPosition());
            telemetry.addData("Right glyph claw position", robot.glyphClawRight.getPosition());
            telemetry.addData("Encoder rightFront", robot.rightFront.getCurrentPosition());
            telemetry.addData("Encoder rightBack", robot.rightBack.getCurrentPosition());
            telemetry.addData("Encoder leftFront", robot.leftFront.getCurrentPosition());
            telemetry.addData("Encoder leftBack", robot.leftBack.getCurrentPosition());
            telemetry.addData("Encoder pulley", robot.pulley.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

}
