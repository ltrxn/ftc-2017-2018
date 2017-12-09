package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Trxn on 11/7/2017.
 */

@TeleOp(name = "Drive Michael Scott", group = "testing")
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

    /******VALUES******/
    private ElapsedTime runtime = new ElapsedTime(); // For time out (encoder drive)




    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        jewelKnockerCounter = 1;
        telemetry.addData(">", "Initialization finished. Press play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 3.5;
            double rightY = gamepad1.right_stick_y / 2;
            final double v1 = r * Math.cos(robotAngle) - rightY;
            final double v2 = r * Math.sin(robotAngle) - rightY;
            final double v3 = r * Math.sin(robotAngle) - rightY;
            final double v4 = r * Math.cos(robotAngle) - rightY;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);


            while (gamepad1.right_trigger > 0) {
                double speed = gamepad1.right_trigger;
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(-speed);
                robot.leftBack.setPower(speed);
                robot.rightBack.setPower(-speed);
            }

            while (gamepad1.left_trigger > 0) {
                double speed = gamepad1.left_trigger;
                robot.leftFront.setPower(-speed);
                robot.rightFront.setPower(speed);
                robot.leftBack.setPower(-speed);
                robot.rightBack.setPower(speed);
            }

            if (gamepad1.right_bumper) {
                robot.openClaw();
            }

            if (gamepad1.left_bumper) {
                robot.closeClaw();
            }

            if (gamepad1.a) {
                robot.raiseJewelKnockerRight();
            }

            if (gamepad1.b) {
                robot.lowerJewelKnockerRight();
            }

            robot.relicTrackables.activate();
            RelicRecoveryVuMark vuMark = robot.getVuMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "Is not visable");
            }

            telemetry.addData("RightJewelKnocker", robot.jewelKnockerRight.getPosition());
            telemetry.update();
            idle();
        }
    }
}
