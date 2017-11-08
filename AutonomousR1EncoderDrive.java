package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Liwen Trxn on 10/19/2017.
 */
@Autonomous(name="R1 - Encoder", group="real")
@Disabled

public class AutonomousR1EncoderDrive extends LinearOpMode {


    //Wheel Motors
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    //Motor to pull up and down the glyph claw
    public DcMotor glyphPully;

    //Servos
    public Servo glyphClawRight;
    public Servo glyphClawLeft;
    public Servo jewelKnockerRight;
    public Servo jewelKnockerLeft;

    //Vuforia
    VuforiaLocalizer vuforia;

    //Sensors
    public ColorSensor rightColorSensor;
    public ColorSensor leftColorSensor;
    public GyroSensor sensorGyro;
    public ModernRoboticsI2cGyro mrGyro;

    //Values
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double TICKS_PER_INCH = TETRIX_TICK_PER_REV / (WHEEL_DIAMETER * Math.PI); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)
    double turnSpeed = .2;
    int zAccumulated; //total rotation left/right
    int rightCryptoboxDistance = 20;
    int centerCryptoboxDistance = 28;
    int leftCryptoboxDistance = 36;
    double cryptoboxDrivePower = .5; //how fast to drive when going to cryptobox
    @Override
    public void runOpMode() throws InterruptedException {

        /**********************************************************
         ********************* INITIALIZATION *********************
         **********************************************************/

        //Declaring hardware
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        glyphPully = hardwareMap.dcMotor.get("glyphPully");

        glyphClawLeft = hardwareMap.servo.get("glyphClawLeft");
        glyphClawRight = hardwareMap.servo.get("glyphClawRight");
        jewelKnockerLeft = hardwareMap.servo.get("jewelKnockerLeft");
        jewelKnockerRight = hardwareMap.servo.get("jewelKnockerRight");

        rightColorSensor = hardwareMap.colorSensor.get("colorRihgt");
        leftColorSensor = hardwareMap.colorSensor.get("colorLeft");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        //Setting up motors
        rightFront.setDirection(DcMotor.Direction.REVERSE); //reverse the
        rightBack.setDirection(DcMotor.Direction.REVERSE);  //right motors


        //Vuforia Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASqeker/////AAAAGXyaxZaNe0ngmcBSZn2x/aMgngJtpaXLUuvWMrmqLi768fIwOZ90EwGzH1f8bs6XPrz7WgklDoQCfX9QmfDMh6MxdyzlDpt5KFjmHrHrPgFi+qnZvD9qgfY4gTH8epM5T/tt7BJHwoNC4hjk9+Jf1Ane+XS6AXZrVf04wCynRUzE64zQDgGxflNMl73Q5qSd7BN3OvkAymgrQE4dsg1S97o7sX+obj+ubPZoJ7Hh8KriU6iOHmrVyTx6epZG2lWXDK0Iv6cQjku7Z5hvZNnK9Uvv8TRNIz5R71PBahS0nR4Xn0FH3beyMc+iu6ZNv33ZRuSh9MCPIUlzquqZtvMMPGbg3880nFmLkS9ytLn90kdX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // not necessary

        //GRABBING THE GLYPH AT START
        closeGlyphClaw();

        //Calibrating mrGyro sensor
        sleep(1000); //wait one second for gyro stop moving
        mrGyro.calibrate();
        while (mrGyro.isCalibrating()) {
            //waiting for mrGyro sensor to finish calibrating
        }



        telemetry.addData(">", "Initialization finished. Press play to start");
        telemetry.update();


        waitForStart();

        /**********************************************************
         ****************** START OF AUTONOMOUS *******************
         **********************************************************/



        relicTrackables.activate();

        while (opModeIsActive()) {

            //Knock off the BLUE jewel
            rightColorSensor.enableLed(true); //turn on color sensor
            lowerJewelKnockerRight();
            if(rightColorSensor.red()> rightColorSensor.blue()) { //if is to the right of the jewel
                //move the robot right
                gyroAbsoluteTurn(30);
                gyroAbsoluteTurn(0);

            } else {
                //move the robot left
                gyroAbsoluteTurn(-30);
                gyroAbsoluteTurn(0);

            }


            //vuforia - decrypts the pictograph
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

                //if a pictograph is visible...
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    //drive in front of the right column
                    scoreRight();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    //drive in front of the center column
                    scoreCenter();

                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    //drive in front of the left column
                    scoreLeft();
                }

            } else {
                telemetry.addData("Vumark", "is not visable");
            }

            //Release the glyph ***UNSURE HOW THIS WILL WORK***
            openGlyphClaw();

            //ROBOT SHOULD BE IN SAFE ZONE, AUTONOMOUS OVER

            idle();
        }
    }




    /**********************************************************
     ************************ METHODS**************************
     **********************************************************/

    private void closeGlyphClaw() {
        //Closes the glyph claw to grasp the block
        double closedLeftGlyphClaw = .5;
        double closedRightGlyphClaw = .5;

        glyphClawLeft.setPosition(closedLeftGlyphClaw);
        glyphClawRight.setPosition(closedRightGlyphClaw);
    }

    private void openGlyphClaw() {
        //Open the glyph claw
        double openLeftGlyphClaw = .1;
        double openRightGlyphClaw = .9;

        glyphClawLeft.setPosition(openLeftGlyphClaw);
        glyphClawRight.setPosition(openRightGlyphClaw);
    }

    private void lowerJewelKnockerRight() {
        //lowers jewel knocker right ***exact position yet to be determined***
        jewelKnockerRight.setPosition(.95);
    }

    private void raiseJewelKnockerRight() {
        //raises jewel knocker right ***exact position yet to be determined***
        jewelKnockerRight.setPosition(.5);
    }

    private void lowerJewelKnockerLeft() {
        //lowers jewel knocker left ***exact position yet to be determined***

        jewelKnockerLeft.setPosition(.05);
    }

    private void raisesJewelKnockerLeft () {
        //raises jewel knocker left ***exact position yet to be determined***

        jewelKnockerLeft.setPosition(.5);
    }



    //VUFORIA
    private void scoreRight() throws InterruptedException {
        /**score into right column
         * To Do:
         * drive forward ~20" (rightCryptoboxDistance), turn 90 degrees right, drive forward ~12"
         */

        encoderDrive(cryptoboxDrivePower, rightCryptoboxDistance); //Drive 20 inches at 0.5 power

        try {
            gyroTurn(90); // turn 90 degrees right
        } catch (InterruptedException e) {
            telemetry.addData("GyroTurn: ", "Failed");// if GyroTurn fails, send message to driver station.
        }

        encoderDrive(cryptoboxDrivePower, 12); //Drive 12 inches at 0.5 power;

    }

    private void scoreCenter() throws InterruptedException {
        /**score into center column
         * To Do:
         * drive forward ~28"(centerCryptoboxDistance), turn 90 degrees right, drive forward ~12"
         */

        encoderDrive(cryptoboxDrivePower, centerCryptoboxDistance); //Drive 28 inches at 0.5 power;

        try {
            gyroTurn(90); // turn 90 degrees right
        } catch (InterruptedException e) {
            telemetry.addData("GyroTurn: ", "Failed");// if GyroTurn fails, send message to driver station.
        }

        encoderDrive(cryptoboxDrivePower, 12); //Drive 12 inches at 0.5 power;

    }

    private void scoreLeft() {
        /**score into left column
         * To Do:
         * drive forward ~36" (leftCryptoboxDistance), turn 90 degrees right, drive forward ~12"
         */

        encoderDrive(cryptoboxDrivePower, leftCryptoboxDistance); //Drive 36 inches at 0.5 power;

        try {
            gyroTurn(90); // turn 90 degrees right
        } catch (InterruptedException e) {
            telemetry.addData("GyroTurn: ", "Failed"); // if GyroTurn fails, send message to driver station.
        }

        encoderDrive(cryptoboxDrivePower, 12); //Drive 12 inches at 0.5 power;

    }

    //DRIVING METHODS

    public void driveForward(double power) {
        //Drives forward. Parameters: power = how fast you want the robot to go
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void stopDriving() {
        //Drives forward. Parameters: power = how fast you want the robot to go
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void encoderDrive(double power, double distance) {
        //Drives forward. Parameters: power = how fast you want the robot to go, distance = how far (in inches)
        int target;
        target =  (int)(distance * TICKS_PER_INCH); //Multiply to find # of ticks to drive (then type cast to an int)

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        //set target position
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightBack.setTargetPosition(target);

        //set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        driveForward(power);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void gyroTurn(int target) throws InterruptedException {
        //Use this method to turn. Parameter: target = how many degrees to turn.
        gyroAbsoluteTurn(target + mrGyro.getIntegratedZValue());
    }

    public void gyroAbsoluteTurn(int target) throws InterruptedException {
        //Turns robot to the target from where the robot started at
        //DO NOT USE THIS METHOD TO TURN
        zAccumulated = mrGyro.getIntegratedZValue(); //set variable to gyro readings
        while (Math.abs(zAccumulated-target) > 3) { //run this loop if robot is 3 degrees or more away from target
            if (zAccumulated > target) {  //if gyro is positive, we will turn left
                leftFront.setPower(-turnSpeed);
                rightFront.setPower(turnSpeed);
                leftBack.setPower(-turnSpeed);
                rightBack.setPower(turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is negative, we will turn right
                leftFront.setPower(turnSpeed);
                rightFront.setPower(-turnSpeed);
                leftBack.setPower(turnSpeed);
                rightBack.setPower(-turnSpeed);
            }
            idle();
            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        }
        stopDriving();

        telemetry.addData("accu", String.format("%03d", zAccumulated));
        telemetry.update();

        idle();
    }
}
