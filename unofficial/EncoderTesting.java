package org.firstinspires.ftc.teamcode.unofficial;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMichaelScott;

/**
 * Created by Trxn on 12/27/2017.
 */

@TeleOp (name="Encoder Values Testing")
@Disabled
public class EncoderTesting extends LinearOpMode {
    //Robot Hardware
    private HardwareMichaelScott robot = new HardwareMichaelScott();

    //Encoders
    private ElapsedTime encoderTime = new ElapsedTime();
    private static final double ANDYMARK_TICKS_PER_REV = 1120; //# of ticks per revolution
    private static final double DRIVE_GEAR_REDUCTION = .5;   //Since gears go from big to small, one rotation of the gear is actually only half a rotation of the wheel
    private static final double WHEEL_DIAMETER_INCHES = 4;   //Diameter of the wheel
    private static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); //# of ticks to be rotated to drive an inch

    private int ticksToGo = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.resetWheelEncoders(); //reset encoders
        robot.stopDriving(); //ensure motors are off

        telemetry.addData(">", "초기화가 완료되었습니다 (READY TO ROCK AND ROLL)"); //alert driver that robot is finished with initialization
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Current Ticks", ticksToGo);
            telemetry.addData("Ticks Per Inch", TICKS_PER_INCH);
            if (gamepad1.a) {
                ticksToGo += 10;
                sleep(100);
            }
            if (gamepad1.b) {
                ticksToGo -= 10;
                sleep(100);

            }
            if (gamepad1.x) {
                ticksToGo += 100;
                sleep(100);

            }
            if (gamepad1.y) {
                ticksToGo += 100;
                sleep(100);

            }
            if(gamepad1.dpad_up) {
                ticksToGo=120;
                sleep(10);
            }
            if(gamepad1.dpad_down) {
                ticksToGo=0;
                sleep(10);
            }
            if(gamepad1.dpad_right) {
                ticksToGo+=120;
                sleep(10);
            }
            if(gamepad1.dpad_left) {
                ticksToGo-=120;
                sleep(10);
            }
            if (gamepad1.right_bumper) {
                driveByTicks();
            }
            if (gamepad1.left_bumper) {
                robot.resetWheelEncoders();
            }

            //Gamepad 2 - Right Joystick - moves pulley
            double pulleyPowerRight = -gamepad2.right_stick_y;
            robot.pulley.setPower(scaleInput(pulleyPowerRight)); //scaled


            telemetry.addData("Encoder rightFront", robot.rightFront.getCurrentPosition());
            telemetry.addData("Encoder rightBack", robot.rightBack.getCurrentPosition());
            telemetry.addData("Encoder leftFront", robot.leftFront.getCurrentPosition());
            telemetry.addData("Encoder leftBack", robot.leftBack.getCurrentPosition());
            telemetry.addData("Encoder pulley", robot.pulley.getCurrentPosition());

            telemetry.update();
        }

    }

    private void driveByTicks() {
        robot.resetWheelEncoders();

        robot.leftFront.setTargetPosition(ticksToGo);
        robot.leftBack.setTargetPosition(ticksToGo);
        robot.rightFront.setTargetPosition(ticksToGo);
        robot.rightBack.setTargetPosition(ticksToGo);

        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.driveForward(.2);
        while (opModeIsActive() &&
                (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Goal Position", "%7d", ticksToGo);
            telemetry.addData("Current Position", "%7d :%7d :%7d :%7d",
                    robot.leftFront.getCurrentPosition(),
                    robot.leftBack.getCurrentPosition(),
                    robot.rightFront.getCurrentPosition(),
                    robot.rightBack.getCurrentPosition());
            telemetry.update();
        }
        robot.stopDriving();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);   // optional pause after each move
    }

    //Drives forward using encoders
    private void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.resetWheelEncoders();

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
            encoderTime.reset();
            robot.driveForward(Math.abs(speed));

            // wait for motors to not be busy
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
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

            sleep(100);   // optional pause after each move
        }
    } private void pulleyByTicks() {

        robot.pulley.setTargetPosition(ticksToGo);

        robot.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.pulley.setPower(.2);
        while (opModeIsActive() &&
                (robot.pulley.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Goal Position", "%7d", ticksToGo);
            telemetry.addData("Current Position", "%7d",
                    robot.pulley.getCurrentPosition());
            telemetry.update();
        }
        robot.pulley.setPower(0);

        robot.pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);   // optional pause after each move
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


}