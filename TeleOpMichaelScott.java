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
    //Values
    int jewelKnockerCounter;
    int target;
    //Others
    private ElapsedTime runtime = new ElapsedTime(); // For time out (encoder drive)
    //Encoder drive Values
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)


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

            if (gamepad1.y) {
                encoderDrive(.1, 3, 3, 10);

            }

            telemetry.addData("RightJewelKnocker", robot.jewelKnockerRight.getPosition());
            //telemetry.addData("rightclaw", robot.glyphClawRight.getPosition());
            //telemetry.addData("leftclaw", robot.glyphClawLeft.getPosition());


            robot.relicTrackables.activate();
            RelicRecoveryVuMark vuMark = robot.getVuMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "Is not visable");
            }
            telemetry.update();
            idle();
        }
    }

//    public void encoderDrive(double power, double distance, double timeoutS) {
//        //Drives forward. Parameters: power = how fast you want the robot to go, distance = how far (in inches)
//        int target;
//        target = (int) (distance * TICKS_PER_INCH); //Multiply to find # of ticks to drive (then type cast to an int)
//
//        telemetry.addData("Encoder Drive", target);
//
//        telemetry.update();
//        //reset encoders
//        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //set target position
//        robot.leftFront.setTargetPosition(target);
//        robot.rightFront.setTargetPosition(target);
//        robot.leftBack.setTargetPosition(target);
//        robot.rightBack.setTargetPosition(target);
//
//        //set to RUN_TO_POSITION mode
//        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        //start timer
//        runtime.reset();
//
//        //set drive power
//        robot.driveForward(power);
//
//        while (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && runtime.seconds() < timeoutS) {
//            //wait until target position is reached
//        }
//
//        //stop and change modes back to normal
//        robot.stopDriving();
//        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newRightTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftTarget);
            robot.rightFront.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
