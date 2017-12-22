package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.HardwareMichaelScott;

/**
 * Created by Trxn on 12/17/2017.
 */

public class R1State extends LinearOpMode{
    //Robot Hardware
    HardwareMichaelScott robot = new HardwareMichaelScott();
    ColorSensor sensorColor;

    //States
    private enum State {
        STATE_INITIAL,
        STATE_KNOCK_JEWEL,
        STATE_DRIVE_TO_CRYPTOBOX,
        STATE_FACE_CRYPTOBOX,
        STATE_SCORE,
        STATE_STOP,
    }

    //Loop cycle time stats variables
    public ElapsedTime runTime = new ElapsedTime(); //time into round
    public ElapsedTime stateTime = new ElapsedTime(); //time into current state
    private ElapsedTime encoderTime = new ElapsedTime();


    private State currentState; //current state machine state
    private double currentJewelKnockerDown = .57;

    private int trialCounter = 0;



    //Vumark
    RelicRecoveryVuMark vuMark; //what column to score in

    //Variables
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

    //Color Sensors
    float hsvValues[] = {0F, 0F, 0F}; //holds hue, saturation, and value information
    final float values[] = hsvValues; //reference to ^^
    final double SCALE_FACTOR = 255; //amplify the difference
    int redSensor;
    int blueSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensor");

        robot.resetEncoders(); //reset encoders
        robot.stopDriving(); //ensure motors are off

        telemetry.addData(">", "초기화가 완료되었습니다 (READY TO ROCK AND ROLL)"); //alert driver that robot is finished with initialization
        telemetry.update();

        waitForStart();
        //Start

        runTime.reset(); //zero game clock

        newState(State.STATE_INITIAL);
        robot.relicTrackables.activate();

        //Start loop
        while (opModeIsActive()) {
            //first line of telemetry, runt time and current state time
            telemetry.addData("Time", String.format("%4.f", stateTime.time()) + currentState.toString());

            switch (currentState) {
                case STATE_INITIAL:
                    if (robot.encodersAtZero() || trialCounter > 3) {
                        newState(State.STATE_KNOCK_JEWEL);
                    } else {
                        trialCounter++;
                    }
                    break;
                case STATE_KNOCK_JEWEL:
                    robot.jewelKnockerRight.setPosition(currentJewelKnockerDown);

                    redSensor = sensorColor.red();
                    blueSensor = sensorColor.blue();

                    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                            (int) (sensorColor.green() * SCALE_FACTOR),
                            (int) (sensorColor.blue() * SCALE_FACTOR),
                            hsvValues);

                    sleep(1000);
                    if (blueSensor > 20 || redSensor > 20 ) {
                        if (redSensor > blueSensor) {
                            encoderDrive(.06, -5, 5, 5);
                            encoderDrive(.06, 4, -4, 5);
                        } else {
                            encoderDrive(.1, 5, -5, 5);
                            encoderDrive(.1, -4, 4, 5);
                        }
                        newState(State.STATE_KNOCK_JEWEL);
                    } else {
                        currentJewelKnockerDown -= .02;
                        trialCounter++;
                    }
                    break;
                case STATE_DRIVE_TO_CRYPTOBOX:

            }
            telemetry.update();
        }



    }



    //Sets a new state and resets state clock
    private void newState(State newState) {
        currentState = newState;
        stateTime.reset();
        trialCounter = 0;
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.resetEncoders();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            encoderTime.reset();
            robot.driveForward(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Goal Position", "%7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Current Position", "%7d :%7d :%7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.stopDriving();

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
