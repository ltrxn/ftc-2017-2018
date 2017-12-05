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

/**
 * Created by Trxn on 11/7/2017.
 */

public class HardwareMichaelScott {
    //Wheel Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    //Servos
    public Servo jewelKnockerRight = null;
    //public Servo glyphClawRight = null;
    //public Servo glyphClawLeft = null;

    //Vuforia
    public VuforiaLocalizer vuforia;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;
    
    //Values
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)



    private DcMotor.RunMode initialMode = null;
    HardwareMap map = null;
    private ElapsedTime runtime = new ElapsedTime(); // For time out (encoder drive)

    public HardwareMichaelScott(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public HardwareMichaelScott() {
        this(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(HardwareMap aMap) {
        map = aMap;
        leftFront = map.dcMotor.get("leftFront");
        rightFront = map.dcMotor.get("rightFront");
        leftBack = map.dcMotor.get("leftBack");
        rightBack = map.dcMotor.get("rightBack");
        jewelKnockerRight = map.servo.get("jewelKnockerRight");
        //glyphClawLeft = map.servo.get("glyphClawLeft");
        //glyphClawRight = map.servo.get("glyphClawRight");
        //ENCODERS
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
        jewelKnockerRight.setPosition(0.0);

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

    //DRIVING METHODS

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



    //GLYPH CLAW
    public void closeClaw() {
        //glyphClawLeft.setPosition(.7);
        //glyphClawRight.setPosition(.2);
    }
    public void openClaw() {
        //glyphClawLeft.setPosition(.1);
        //glyphClawRight.setPosition(.7);
    }

    public void lowerJewelKnockerRight() {
        //lowers jewel knocker right ***exact position yet to be determined***
        jewelKnockerRight.setPosition(.5);
    }

    public void raiseJewelKnockerRight() {
        //raises jewel knocker right ***exact position yet to be determined***
        jewelKnockerRight.setPosition(0);
    }

    //VUFORIA
    public RelicRecoveryVuMark getVuMark() {
        //vuforia - decrypts the pictograph
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }
}
