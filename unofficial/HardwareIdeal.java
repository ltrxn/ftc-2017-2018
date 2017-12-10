package org.firstinspires.ftc.teamcode.unofficial;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;

/**
 * Created by Trxn on 11/8/2017.
 */

public class HardwareIdeal {
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
    public VuforiaLocalizer vuforia;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;

    //Sensors
    public ColorSensor rightColorSensor;
    public ColorSensor leftColorSensor;
    public ColorSensor groundColorSensor;
    public GyroSensor sensorGyro;
    public ModernRoboticsI2cGyro mrGyro;


    //Encoder Drive Values
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV  * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * Math.PI); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)


    //Others
    public DcMotor.RunMode initialMode = null;
    HardwareMap map = null;


    public HardwareIdeal(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public HardwareIdeal() {
        this(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(HardwareMap aMap) throws InterruptedException {
        map = aMap;

        //Declaring hardware
        leftFront = map.dcMotor.get("leftFront");
        rightFront = map.dcMotor.get("rightFront");
        leftBack = map.dcMotor.get("leftBack");
        rightBack = map.dcMotor.get("rightBack");
        glyphPully = map.dcMotor.get("glyphPully");

        glyphClawLeft = map.servo.get("glyphClawLeft");
        glyphClawRight = map.servo.get("glyphClawRight");
        jewelKnockerLeft = map.servo.get("jewelKnockerLeft");
        jewelKnockerRight = map.servo.get("jewelKnockerRight");

        rightColorSensor = map.colorSensor.get("colorRight");
        leftColorSensor = map.colorSensor.get("colorLeft");
        groundColorSensor = map.colorSensor.get("colorGround");
        sensorGyro = map.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;


        //ENCODERS
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //VUFORIA SETUP
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASqeker/////AAAAGXyaxZaNe0ngmcBSZn2x/aMgngJtpaXLUuvWMrmqLi768fIwOZ90EwGzH1f8bs6XPrz7WgklDoQCfX9QmfDMh6MxdyzlDpt5KFjmHrHrPgFi+qnZvD9qgfY4gTH8epM5T/tt7BJHwoNC4hjk9+Jf1Ane+XS6AXZrVf04wCynRUzE64zQDgGxflNMl73Q5qSd7BN3OvkAymgrQE4dsg1S97o7sX+obj+ubPZoJ7Hh8KriU6iOHmrVyTx6epZG2lWXDK0Iv6cQjku7Z5hvZNnK9Uvv8TRNIz5R71PBahS0nR4Xn0FH3beyMc+iu6ZNv33ZRuSh9MCPIUlzquqZtvMMPGbg3880nFmLkS9ytLn90kdX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // not necessary

        //CALIBRATE GYROSENSOR
        sleep(1000); //wait one second for gyro stop moving
        mrGyro.calibrate();
        while (mrGyro.isCalibrating()) {
            //waiting for mrGyro sensor to finish calibrating
        }

        //ALL
        leftFront.setMode(initialMode);
        rightFront.setMode(initialMode);
        leftBack.setMode(initialMode);
        rightBack.setMode(initialMode);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


    /**********************************************************
     ************************ METHODS**************************
     **********************************************************/

    //VUFORIA
    public RelicRecoveryVuMark getVuMark() {
        //vuforia - decrypts the pictograph
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }
    //CLAWS

    public void closeGlyphClaw() {
        //Closes the glyph claw to grasp the block
        double closedLeftGlyphClaw = .5;
        double closedRightGlyphClaw = .5;

        glyphClawLeft.setPosition(closedLeftGlyphClaw);
        glyphClawRight.setPosition(closedRightGlyphClaw);
    }

    public void openGlyphClaw() {
        //Open the glyph claw
        double openLeftGlyphClaw = .1;
        double openRightGlyphClaw = .9;

        glyphClawLeft.setPosition(openLeftGlyphClaw);
        glyphClawRight.setPosition(openRightGlyphClaw);
    }

    public void lowerJewelKnockerRight() {
        //lowers jewel knocker right ***exact position yet to be determined***
        jewelKnockerRight.setPosition(.95);
    }

    public void raiseJewelKnockerRight() {
        //raises jewel knocker right ***exact position yet to be determined***
        jewelKnockerRight.setPosition(.5);
    }

    public void lowerJewelKnockerLeft() {
        //lowers jewel knocker left ***exact position yet to be determined***

        jewelKnockerLeft.setPosition(.05);
    }

    public void raisesJewelKnockerLeft() {
        //raises jewel knocker left ***exact position yet to be determined***

        jewelKnockerLeft.setPosition(.5);
    }

    //DRIVE

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

    public int encoderDrive(double power, double distance) {
        //Drives forward. Parameters: power = how fast you want the robot to go, distance = how far (in inches)
        int target;
        target = (int) (distance * TICKS_PER_INCH); //Multiply to find # of ticks to drive (then type cast to an int)

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

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return target;
    }

    public void encoderStrafe(double power, double distance) {
        //Drives forward. Parameters: power = how fast you want the robot to go, distance = how far (in inches)
        int target;
        target = (int) (distance * TICKS_PER_INCH); //Multiply to find # of ticks to drive (then type cast to an int)

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        //set target position
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(-target);
        leftBack.setTargetPosition(-target);
        rightBack.setTargetPosition(target);

        //set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        driveForward(power);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //COLOR SENSOR
    public void turnOnColorSensorRight() {
        rightColorSensor.enableLed(true);
    }

    public void turnOffColorSensorRight() {
        rightColorSensor.enableLed(false);
    }

    public void turnOnColorSensorLeft() {
        leftColorSensor.enableLed(true);
    }

    public void turnOffColorSensorLeft() {
        leftColorSensor.enableLed(false);
    }

    public void turnOnColorSensorGround() {
        groundColorSensor.enableLed(true);
    }

    public void turnOffColorSensorGround() {
        groundColorSensor.enableLed(false);
    }


}
