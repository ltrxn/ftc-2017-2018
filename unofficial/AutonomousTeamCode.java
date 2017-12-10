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

        telemetry.addData(">", "Initialization finished. Press play to start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            //Turn on color sensor
            rightColorSensor.enableLed(true);

            //Put the jewel knocker down
            jewelKnockerRight.setPosition(.5);

            Color.RGBToHSV(rightColorSensor.red() * 8, rightColorSensor.green() * 8, rightColorSensor.blue() * 8, hsvValues);

        }
    }
}
