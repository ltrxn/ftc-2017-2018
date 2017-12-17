package org.firstinspires.ftc.teamcode.unofficial;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jagged Edge Programming Team on 12/5/2017.
 */

public class AutonomousTeamCode extends LinearOpMode{

    //Wheel Motors
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    //Servos
    public Servo jewelKnockerRight;

    //Sensors
    public ColorSensor rightColorSensor;
    public GyroSensor sensorGyro;
    public ModernRoboticsI2cGyro mrGyro;

    //Values
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    //GYRO VALuES
    int zAccumulated;
    int heading;
    int xVAl, yVal, zVal;


    @Override
    public void runOpMode() throws InterruptedException {
        //Declaring hardware
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        jewelKnockerRight = hardwareMap.servo.get("jewelKnockerRight");

        rightColorSensor = hardwareMap.colorSensor.get("colorRight");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        //Reversing motors
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Calibrate gyro sensor
        mrGyro.calibrate();
        while (mrGyro.isCalibrating()){

        }
        telemetry.addData(">", "Initialization finished. Press play to start");
        telemetry.update();



        waitForStart();

        while(opModeIsActive()) {

            //Gyro values get put in
            zAccumulated= mrGyro.getIntegratedZValue();
            heading= 360 - mrGyro.getHeading();
            if (heading==360){
                heading=0;
            }
            xVAl= mrGyro.rawX()/ 128;
            yVal= mrGyro.rawY()/ 128;
            zVal= mrGyro.rawZ()/ 128;

            //Says to the driver that the robot is 'Running'


            //Turn on color sensor
            LEDOn();

            //Put the jewel knocker down
            jewelKnockerRight.setPosition(.5);

            Color.RGBToHSV(rightColorSensor.red() * 8, rightColorSensor.green() * 8, rightColorSensor.blue() * 8, hsvValues);
            if (rightColorSensor.blue()>rightColorSensor.red()){
                //Turn robot ~30 degrees counter clockwise

            }
            DriveForwardDistance(.2,5);

            telemetry.addData("1. heading", String.format("403d", heading));
            telemetry.addData("2. accy", String.format("403d", zAccumulated));
            telemetry.addData("3. X", String.format("403d", xVAl));
            telemetry.addData("3. Y", String.format("403d", yVal));
            telemetry.addData("3. Z", String.format("403d", zVal));
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    //LEDOn = the LED is on
    public void LEDOn() {
        rightColorSensor.enableLed(true);
    }

    //LEDOff = the LED if off
    public void LEDOff() {

        rightColorSensor.enableLed(false);

    }

    public void DriveForwardDistance (double power, int distance) {

        //encoder

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target
        leftFront.setTargetPosition(distance*1440);
        leftBack.setTargetPosition(distance*1440);
        rightFront.setTargetPosition(distance*1440);
        rightBack.setTargetPosition(distance*1440);

        //settoRunToPOS
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power of drive train
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);

        //wait for bot to go distance
        while(leftBack.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            //NOTHING GOES HERE. PROBLEM IF SOMETHING IS PUT IN

        }
        //Make robot stop
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }


    public void DriveBackwardDistance (double power, int distance) {

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set negative value so that you can go backward, because after resetting encoders,
        // value is 0 so has to be negative to go backward.
        leftFront.setTargetPosition(-distance*1440);
        leftBack.setTargetPosition(-distance*1440);
        rightBack.setTargetPosition(-distance*1440);
        rightFront.setTargetPosition(-distance*1440);


        //setMode
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power of drive train
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);

        //wait for bot to go distance
        while (leftBack.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            //NOTHING GOES HERE. PROBLEM IF SOMETHING   ff IS PUT IN

        }
        //Make robot stop
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void RotateRightDegree (double power, int distance) {

        //reset enc
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setTargetPos. To Rotate clockwise, the right side has to be negative, while left is positive.
        //Don't forget to reset encoder values
        leftFront.setTargetPosition(7200);
        leftBack.setTargetPosition(7200);
        rightFront.setTargetPosition(-7200);
        rightBack.setTargetPosition(-7200);

        //setPowerMode
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set Power
        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightFront.setPower(.5);
        rightBack.setPower(.5);

        //wait for robot to rotate
        while(leftBack.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //once again, nothing

        }
        //make powers 0 to stop robot
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }


    public void RotatecounterClockDegree (double power, int distance){

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setTargetPos
        //Because the robot is rotating counterclockwise, the left drive train has to be negative and
        //the right train has to be positive
        leftFront.setTargetPosition(-7200);
        leftBack.setTargetPosition(-7200);
        rightFront.setTargetPosition(7200);
        rightBack.setTargetPosition(7200);


        //setPowerMode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set Power
        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightFront.setPower(.5);
        rightBack.setPower(.5);


        //let robot run
        while(leftBack.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //once again, nothing

        }
        //make powers 0 to make robot stop
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    public void Strafeleft (double power, int distance) {

        //reset encoders
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set Target Position
        leftFront.setTargetPosition(-7200);
        leftBack.setTargetPosition(7200);
        rightFront.setTargetPosition(7200);
        rightBack.setTargetPosition(-7200);

        //setPowerMode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightBack.setPower(.5);
        rightFront.setPower(.5);

        //let robot run
        while (leftFront.isBusy() && leftBack.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {


        }

        //stop robot
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void StrafeRight (double power, int distance){

        //reset
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        //setTargetPosition
        leftFront.setTargetPosition(7200);
        leftBack.setTargetPosition(-7200);
        rightFront.setTargetPosition(-7200);
        rightBack.setTargetPosition(7200);

        //setpowermode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        //set power
        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightBack.setPower(.5);
        rightFront.setPower(.5);

        //let robot run
        while (leftFront.isBusy() && leftBack.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {


        }

        //stop robot
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}
