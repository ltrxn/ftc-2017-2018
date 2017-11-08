package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Trxn on 11/7/2017.
 */

public class HardwareMichaelScott {
    //Wheel Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    private DcMotor.RunMode initialMode = null;
    HardwareMap map = null;

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
    }
}
