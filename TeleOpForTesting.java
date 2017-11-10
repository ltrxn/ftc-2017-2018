package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Trxn on 11/7/2017.
 */

@TeleOp(name="TeleOp with drive only", group = "testing")
public class TeleOpForTesting extends LinearOpMode{

    HardwareMichaelScott robot = new HardwareMichaelScott();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData(">", "Initialization finished. Press play to start");


        waitForStart();
        while (opModeIsActive()) {
            double rightStickY = gamepad1.left_stick_y;
            double leftStickY = gamepad1.right_stick_y;

            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;

            robot.leftFront.setPower(leftStickY);
            robot.leftBack.setPower(leftStickY);
            robot.rightFront.setPower(rightStickY);
            robot.rightBack.setPower(rightStickY);

            while (rightTrigger > leftTrigger) {
                robot.leftFront.setPower(rightTrigger);
                robot.leftBack.setPower(-rightTrigger);
                robot.rightFront.setPower(-rightTrigger);
                robot.rightBack.setPower(rightTrigger);
            }
            while (rightTrigger < leftTrigger) {
                robot.leftFront.setPower(-leftTrigger);
                robot.leftBack.setPower(leftTrigger);
                robot.rightFront.setPower(leftTrigger);
                robot.rightBack.setPower(-leftTrigger);
            }
            idle();
        }
    }


}
