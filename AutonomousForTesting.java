package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Trxn on 11/7/2017.
 */

@Autonomous(name = "Autonomous Testing", group = "testing")
public class AutonomousForTesting extends LinearOpMode{
    HardwareMichaelScott robot = new HardwareMichaelScott();

    //Values
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final double TICKS_PER_INCH = (TETRIX_TICK_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * Math.PI); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)
    int zAccumulated; //total rotation left/right

    int rightCryptoboxDistance = 20;
    int centerCryptoboxDistance = 28;
    int leftCryptoboxDistance = 36;
    double cryptoboxDrivePower = .5; //how fast to drive when going to cryptobox
    double turnSpeed = .2;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData(">", "Initialization finished. Press play to start");

        waitForStart();
        encoderDrive(.3, 5);

    }

    //DRIVING METHODS

    public void driveForward(double power) {
        //Drives forward. Parameters: power = how fast you want the robot to go
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);
    }

    public void stopDriving() {
        //Stops Driving
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void encoderDrive(double power, double distance) {
        //Drives forward. Parameters: power = how fast you want the robot to go, distance = how far (in inches)
        int target;
        target =  (int)(distance * TICKS_PER_INCH); //Multiply to find # of ticks to drive (then type cast to an int)

        //reset encoders
        robot.leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        //set target position
        robot.leftFront.setTargetPosition(target);
        robot.rightFront.setTargetPosition(target);
        robot.leftBack.setTargetPosition(target);
        robot.rightBack.setTargetPosition(target);

        //set to RUN_TO_POSITION mode
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        driveForward(power);

        while(robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy()) {
            //wait until target position is reached
            telemetry.addData("Michael", "is a bit busy");
        }

        //stop and change modes back to normal
        this.stopDriving();
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
