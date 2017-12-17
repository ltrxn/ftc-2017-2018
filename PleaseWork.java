package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Trxn on 12/16/2017.
 */

@TeleOp(name = "This Better work", group = "trash")
public class PleaseWork extends LinearOpMode {

    //Wheel Motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    public Servo jewelKnockerRight;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        jewelKnockerRight = hardwareMap.servo.get("jewelKnockerRight");


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData(">", "Inicialización completa!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                leftFront.setPower(.3);
                leftBack.setPower(.3);
                rightBack.setPower(.3);
                rightFront.setPower(.3);
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
            idle();
        }
    }
}
