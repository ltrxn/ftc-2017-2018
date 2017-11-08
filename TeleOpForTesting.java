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

        robot.leftFront.setPower(gamepad1.left_stick_y);
        robot.leftBack.setPower(gamepad1.left_stick_y);
        robot.rightFront.setPower(gamepad1.right_stick_y);
        robot.rightBack.setPower(gamepad1.right_stick_y);

        idle();
    }
}
