package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Vortex on 12/9/2017.
 */
@Autonomous(name="R1", group="Michael Scott")
public class AutonomousR1 extends LinearOpMode {

    HardwareMichaelScott robot = new HardwareMichaelScott();
    public ModernRoboticsI2cGyro gyro;
    //public ColorSensor rightColorSensor;
    private ElapsedTime runtime = new ElapsedTime();

    static final double ANDYMARK_TICKS_PER_REV  = 1120; //# of ticks per revolution
    static final double DRIVE_GEAR_REDUCTION    = .5;   //Since gears go from big to small, one rotation of the gear is actually only half a rotation of the wheel
    static final double WHEEL_DIAMETER_INCHES   =  4;   //Diameter of the wheel
    static final double TICKS_PER_INCH          = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); //# of ticks to be rotated to drive an inch

    static final double DRIVE_SPEED = 0.6; //Speed while going to crytobox
    static final double GYRO_TURN_SPEED         = 0.5; //Speed while turning

    static final int THRESHOLD                  = 2; //tolerance when turning

    static final int DISTANCE_RIGHT          = 20; //Distance from balancing stone to crytobox positions
    static final int DISTANCE_CENTER         = 28;
    static final int DISTANCE_LEFT           = 36;

    //STATES
    private enum State {
        STATE_INITIAL,
        STATE_KNOCK_JEWEL,
        STATE_DRIVE_TO_CRYPTOBOX,
        STATE_FACE_CRYPTOBOX,
        STATE_SCORE,
        STATE_STOP,
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize
        robot.init(hardwareMap);
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        robot.resetEncoders();

        //gyro.calibrate();
        //while(gyro.isCalibrating()){
        //}



        waitForStart();

        RelicRecoveryVuMark vuMark = robot.getVuMark();

        //knock jewel off
        //rightColorSensor.enableLed(true); //turn on color sensor
//        robot.lowerJewelKnockerRight();
//        if(rightColorSensor.red()> rightColorSensor.blue()) { //if is to the right of the jewel
//            //move the robot right
//            gyroTurn(GYRO_TURN_SPEED, 30);
//            gyroTurn(GYRO_TURN_SPEED, 0);
//
//        } else {
//            //move the robot left
//            gyroTurn(GYRO_TURN_SPEED, -30);
//            gyroTurn(GYRO_TURN_SPEED, 0);
//
//        }


        //drive correct distance
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                encoderDrive(DRIVE_SPEED, DISTANCE_LEFT, 10);
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                encoderDrive(DRIVE_SPEED, DISTANCE_CENTER, 10);
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                encoderDrive(DRIVE_SPEED, DISTANCE_RIGHT, 10);
            }
        } else {
            telemetry.addData("VuMark", "Is not visable");
        }

        //turn 90 degrees
        //gyroTurn(GYRO_TURN_SPEED, 90);

        //drive till cryptobox

        encoderDrive(DRIVE_SPEED, 4, 5);

        //release glyph
            //code for opening glyph claw

        telemetry.addData(">", "Michael Scott es una bestia");
        telemetry.update();
    }

    //Encoder Drive
    public void encoderDrive(double speed, double inches, double timeoutSeconds) {
        //find target position
        int newTarget;
        newTarget = robot.leftFront.getCurrentPosition() + (int)(inches * TICKS_PER_INCH);


        //set target
        robot.leftFront.setTargetPosition(newTarget);
        robot.rightFront.setTargetPosition(newTarget);
        robot.leftBack.setTargetPosition(newTarget);
        robot.rightBack.setTargetPosition(newTarget);

        //set to run to position mode
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start timer
        runtime.reset();

        //start driving
        robot.driveForward(speed);

        //while driving, don't stop
        while (runtime.seconds()<timeoutSeconds &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy())) {
            telemetry.addData("Encoder Drive", "Running to %4d", newTarget);
            telemetry.addData("Encoder Drive", "Running to %4d", robot.leftFront.getCurrentPosition());
            telemetry.update();
        }

        //once loop is done, stop
        robot.stopDriving();

        //turn off run to position mode
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Gyro turn
    public void gyroTurn(double speed, int angle) {
        int target = angle + gyro.getIntegratedZValue();

        while ((Math.abs(gyro.getIntegratedZValue() - target) > THRESHOLD)) {
            if (gyro.getIntegratedZValue() > target) {  //if gyro is positive, we will turn left
                robot.leftFront.setPower(-GYRO_TURN_SPEED);
                robot.rightFront.setPower(GYRO_TURN_SPEED);
                robot.leftBack.setPower(-GYRO_TURN_SPEED);
                robot.rightBack.setPower(GYRO_TURN_SPEED);
            }

            if (gyro.getIntegratedZValue() < target) {  //if gyro is negative, we will turn right
                robot.leftFront.setPower(GYRO_TURN_SPEED);
                robot.rightFront.setPower(-GYRO_TURN_SPEED);
                robot.leftBack.setPower(GYRO_TURN_SPEED);
                robot.rightBack.setPower(-GYRO_TURN_SPEED);
            }
            idle();
        }
        robot.stopDriving();
    }
}
