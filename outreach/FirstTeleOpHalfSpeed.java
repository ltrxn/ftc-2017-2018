package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by inthe on 11/2/2017.
 */

public class FirstTeleOpHalfSpeed extends OpMode {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    //public DcMotor grabArm;

    //public Servo grabber;


    public void init() {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        //grabArm = hardwareMap.dcMotor.get("grabArm");

        //grabber = hardwareMap.servo.get("grabber");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        //makes sure that the robot stays still when initializing
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }

    @Override
    public void loop (){

        //assign variables
        //negate the y values because of joystick
        double leftYvalue = gamepad1.left_stick_y;
        double rightYvalue = gamepad1.right_stick_y;

        // clip the right and left values so that the values never exceed +/- 1
        leftYvalue = Range.clip(leftYvalue, -1, 1);
        rightYvalue = Range.clip(rightYvalue, -1, 1);

        //for staying still when the joystick is not being used
        if (leftYvalue == 0 && rightYvalue == 0) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

        //For moving directly forward
        if (leftYvalue > 0 && rightYvalue > 0){
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
        }
        //for moving backward
        else if (leftYvalue < 0 && rightYvalue < 0){
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
        }

        //for rotating on the spot to the left
        if (leftYvalue < 0 && rightYvalue > 0){
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
        }
        //for rotating on the spot to the right
        else if (leftYvalue > 0 && rightYvalue < 0){
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
        }

        //for turning left while moving forward
        /*
        Depending on how big or small the rightYvalue is, the turn can be a wide
        or short turn
         */
        if (leftYvalue < 0 && rightYvalue > 0 && Math.abs(leftYvalue) < rightYvalue){
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
        }
        //for turning right while moving forward
        else if (leftYvalue > 0 && rightYvalue < 0 && leftYvalue > Math.abs(rightYvalue)){
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
        }

        //for just moving the right drive train forward
        if (rightYvalue > 0){
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
            leftFront.setPower(0);
            leftBack.setPower(0);
        }
        else if (rightYvalue < 0){
            rightFront.setPower(rightYvalue/2);
            rightBack.setPower(rightYvalue/2);
            leftFront.setPower(0);
            leftBack.setPower(0);
        }
        else if (leftYvalue > 0){
            rightFront.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
        }
        else if (leftYvalue < 0){
            rightFront.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(leftYvalue/2);
            leftBack.setPower(leftYvalue/2);
        }

    }

    @Override
    public void stop(){

    }

    /*
    This method scales the joystick input so for low joystick values, the
    scaled value is less than linear. This is to make it easier to drive
    the robot more precisely
     */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;

    }
}
