package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.TeleOpMichaelScott.WHEEL_DIAMETER;

/**
 * Created by Trxn on 11/7/2017.
 */

public class HardwareMichaelScott {
    /******HARDWARE******/
    //Wheel Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor pulley;
    //Servos
    public Servo jewelKnockerRight = null;
    public Servo glyphClawRight = null;
    public Servo glyphClawLeft = null;
    //sensors


    /******VUFORIA******/
    public VuforiaLocalizer vuforia;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;

    /******VALUES******/
    //Encoders
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double ANDYMARK_TICKS_PER_REV = 1120; //# of ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = .5;   //Since gears go from big to small, one rotation of the gear is actually only half a rotation of the wheel
    static final double WHEEL_DIAMETER_INCHES = 4;   //Diameter of the wheel
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); //# of ticks to be rotated to drive an inch
    //Others
    private DcMotor.RunMode initialMode = null;
    HardwareMap map = null;
    private ElapsedTime runtime = new ElapsedTime(); // For time out (encoder drive)

    //Constructors
    public HardwareMichaelScott(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public HardwareMichaelScott() {
        this(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Initialize
    public void init(HardwareMap aMap) {
        map = aMap;

        //MOTORS
        leftFront = map.dcMotor.get("leftFront");
        rightFront = map.dcMotor.get("rightFront");
        leftBack = map.dcMotor.get("leftBack");
        rightBack = map.dcMotor.get("rightBack");
        pulley = map.dcMotor.get("glyphPulley");

        //SERVOS
        jewelKnockerRight = map.servo.get("jewelKnockerRight");
        glyphClawLeft = map.servo.get("glyphClawLeft");
        glyphClawRight = map.servo.get("glyphClawRight");
        //ENCODERS
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ALL
        leftFront.setMode(initialMode);
        rightFront.setMode(initialMode);
        leftBack.setMode(initialMode);
        rightBack.setMode(initialMode);
        pulley.setMode(initialMode);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        glyphClawLeft.setPosition(0);
        glyphClawRight.setPosition(1);
        raiseJewelKnockerRight();

        //VUFORIA SETUP

        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASqeker/////AAAAGXyaxZaNe0ngmcBSZn2x/aMgngJtpaXLUuvWMrmqLi768fIwOZ90EwGzH1f8bs6XPrz7WgklDoQCfX9QmfDMh6MxdyzlDpt5KFjmHrHrPgFi+qnZvD9qgfY4gTH8epM5T/tt7BJHwoNC4hjk9+Jf1Ane+XS6AXZrVf04wCynRUzE64zQDgGxflNMl73Q5qSd7BN3OvkAymgrQE4dsg1S97o7sX+obj+ubPZoJ7Hh8KriU6iOHmrVyTx6epZG2lWXDK0Iv6cQjku7Z5hvZNnK9Uvv8TRNIz5R71PBahS0nR4Xn0FH3beyMc+iu6ZNv33ZRuSh9MCPIUlzquqZtvMMPGbg3880nFmLkS9ytLn90kdX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // not necessary

    }


    /******METHODS******/
    //Drive
    public void driveForward(double power) {
        //Drives forward. Parameters: power = how fast you want the robot to go
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void stopDriving() {
        //Stops Driving
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    //Servos
    public void closeClaw() {
        glyphClawLeft.setPosition(.7);
        glyphClawRight.setPosition(.35);
    }

    public void openClaw() {
        glyphClawLeft.setPosition(.35);
        glyphClawRight.setPosition(.7);
    }

    public void lowerJewelKnockerRight() {
        //lowers jewel knocker right
        jewelKnockerRight.setPosition(.57);
    }

    public void raiseJewelKnockerRight() {
        //raises jewel knocker right
        jewelKnockerRight.setPosition(0);
    }

    //Vuforia
    public RelicRecoveryVuMark getVuMark() {
        //decrypts the pictograph, returns position
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    public boolean vuMarkIsVisable() {
        //returns true of vumark is visable
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            return true;
        } else {
            return false;
        }
    }

    //Encoders

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMode(DcMotor.RunMode newMode) {
        leftFront.setMode(newMode);
        rightFront.setMode(newMode);
        leftBack.setMode(newMode);
        rightBack.setMode(newMode);
    }

    public void encoderDrive(double speed, double inches, double timeoutSeconds) {
        //find target position
        int newTarget;
        newTarget = leftFront.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);

        //set target
        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(newTarget);
        leftBack.setTargetPosition(newTarget);
        rightBack.setTargetPosition(newTarget);

        //set to run to position mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start timer
        runtime.reset();

        //start driving
        driveForward(speed);

        //while driving, don't stop
        while (runtime.seconds() < timeoutSeconds &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
        }

        //once loop is done, stop
        stopDriving();

        //turn off run to position mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean encodersAtZero() {
        if (rightFront.getCurrentPosition() == 0 && rightBack.getCurrentPosition() == 0 && leftFront.getCurrentPosition() == 0 && leftBack.getCurrentPosition() == 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean jewelKnocerDown() {
        if (jewelKnockerRight.getPosition() > .3) {
            return true;
        } else {
            return false;
        }
    }

}
