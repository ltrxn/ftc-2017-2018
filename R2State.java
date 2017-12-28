package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.HardwareMichaelScott;
import org.firstinspires.ftc.teamcode.R1State;

/**
 * Created by Trxn on 12/28/2017.
 */

@Autonomous(name = "R2", group = "official")
public class R2State extends LinearOpMode{
    //hardware
    private HardwareMichaelScott robot = new HardwareMichaelScott();
    private ColorSensor sensorColor;

    //States
    private enum State {
        STATE_INITIAL,
        STATE_KNOCK_JEWEL,
        STATE_LINE_UP_WITH_WALL,
        STATE_DRIVE_TO_CRYPTOBOX,
        STATE_FACE_CRYPTOBOX,
        STATE_SCORE,
        STATE_STOP,
    }

    //time
    private ElapsedTime runTime = new ElapsedTime(); //time into round
    private ElapsedTime stateTime = new ElapsedTime(); //time into current state
    private ElapsedTime encoderTime = new ElapsedTime();

    //values
    private State currentState; //current state machine state
    private double currentJewelKnockerDown = .57; //what to set down jewel knocker position as
    private int trialCounter = 0; //# of times tried before continuing

    //final values
    private static final double TICKS_PER_INCH  = 100; //# of ticks to be rotated to drive an inch
    private static final double DRIVE_SPEED     = .4; //Speed while going to crytobox
    private static final int DISTANCE_RIGHT     = 36; //Distance from balancing stone to crytobox positions
    private static final int DISTANCE_CENTER    = 30;
    private static final int DISTANCE_LEFT      = 24;
    private static final int DISTANCE_TO_CRYPTOBOX = 6; //Distance to push block to cryptobox.
    private static final int DRIVE_TIME_OUT     = 10;

    //color
    private int redSensor;
    private int blueSensor;

    //vuforia
    private RelicRecoveryVuMark vuMark; //what column to score in


    @Override
    public void runOpMode() throws InterruptedException {
        //initialization
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensor");
        robot.resetEncoders(); //reset encoders
        robot.stopDriving(); //ensure motors are off
        telemetry.addData(">", "초기화가 완료되었습니다 (READY TO ROCK AND ROLL)"); //alert driver that robot is finished with initialization
        telemetry.update();

        //start
        waitForStart();
        runTime.reset(); //zero game clock
        newState(State.STATE_INITIAL); //set currentState to initial
        robot.relicTrackables.activate(); //activate vuforia

        //start of loop
        while (opModeIsActive()) {

            switch (currentState) {
                case STATE_INITIAL: //remember vumark and pickup glyph

                    vuMark = robot.getVuMark(); //get vumark

                    if (robot.vuMarkIsVisable() || trialCounter > 3) {
                        vuMark = robot.getVuMark();
                        robot.closeClaw();
                        sleep(500);
                        robot.pulley.setPower(.3);
                        sleep(500);
                        robot.pulley.setPower(0);

                        newState(State.STATE_KNOCK_JEWEL);

                    } else {

                        trialCounter++;

                    }
                    break;


                case STATE_KNOCK_JEWEL: //knock off the blue jewel

                    vuMark = robot.getVuMark(); //get vumark again

                    robot.jewelKnockerRight.setPosition(currentJewelKnockerDown); //lower jewel knocker
                    sleep(1000);

                    redSensor = sensorColor.red(); //remember red value
                    blueSensor = sensorColor.blue(); //remember blue value
                    sleep(500);

                    if (blueSensor > 20 || redSensor > 20 || trialCounter > 3) { //if there is a strong blue/red return...

                        if (redSensor > blueSensor) { //if ball is red...
                            encoderDrive(.06, -3, 3, 5); //turn left
                            encoderDrive(.06, 3, -3, 5); //turn right
                        } else { //if ball is blue...
                            encoderDrive(.1, 3, -3, 5); //turn right
                            encoderDrive(.1, -3, 3, 5); //turn left
                        }

                        newState(State.STATE_LINE_UP_WITH_WALL);

                    } else {

                        currentJewelKnockerDown -= .02; //raise it up a bit
                        trialCounter++;

                    }
                    break;


                case STATE_LINE_UP_WITH_WALL: //end up against the wall

                    robot.raiseJewelKnockerRight(); //lift up jewel knocker
                    encoderDrive(DRIVE_SPEED, 24,24, DRIVE_TIME_OUT); //get off balancing stone
                    encoderDrive(DRIVE_SPEED, -18, 18, DRIVE_TIME_OUT); //turn 90 counter clockwisee
                    encoderDrive(DRIVE_SPEED, -20, -20, DRIVE_TIME_OUT); //back against the wall
                    newState(State.STATE_DRIVE_TO_CRYPTOBOX);

                    break;


                case STATE_DRIVE_TO_CRYPTOBOX: //drive to correct cryptobox

                    if (vuMark == RelicRecoveryVuMark.LEFT) { //vuMark left
                        encoderDrive(DRIVE_SPEED, DISTANCE_LEFT, DISTANCE_LEFT, DRIVE_TIME_OUT);
                    } else if (vuMark == RelicRecoveryVuMark.CENTER) { //vuMark center
                        encoderDrive(DRIVE_SPEED, DISTANCE_CENTER, DISTANCE_CENTER, DRIVE_TIME_OUT);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) { //vuMark righht
                        encoderDrive(DRIVE_SPEED, DISTANCE_RIGHT, DISTANCE_RIGHT, DRIVE_TIME_OUT);
                    } else { //go center if vuMark is unknown
                        encoderDrive(DRIVE_SPEED, DISTANCE_CENTER, DISTANCE_CENTER, DRIVE_TIME_OUT);
                    }
                    newState(State.STATE_FACE_CRYPTOBOX);

                    break;


                case STATE_FACE_CRYPTOBOX: //turn clockwise 90 degrees

                    encoderDrive(DRIVE_SPEED, 18, -18, DRIVE_TIME_OUT);
                    newState(State.STATE_SCORE);

                    break;


                case STATE_SCORE: //drive forward and drop glyph
                    encoderDrive(DRIVE_SPEED, DISTANCE_TO_CRYPTOBOX, DISTANCE_TO_CRYPTOBOX, DRIVE_TIME_OUT);
                    encoderDrive(DRIVE_SPEED, DISTANCE_TO_CRYPTOBOX, DISTANCE_TO_CRYPTOBOX, DRIVE_TIME_OUT);
                    robot.pulley.setPower(-.3);
                    sleep(200);
                    robot.pulley.setPower(0);
                    sleep(1000);

                    robot.openClaw(); //open glyph claw
                    sleep(1000);
                    encoderDrive(DRIVE_SPEED, -5, -5, DRIVE_TIME_OUT * 2);
                    encoderDrive(DRIVE_SPEED, 5, 5, DRIVE_TIME_OUT);
                    encoderDrive(DRIVE_SPEED, -1, -1, DRIVE_TIME_OUT);
                    newState(State.STATE_STOP);

                    break;


                    case STATE_STOP: //do nothing
                    break;
            }


            telemetry.addData("Time", "%2f  " + currentState.toString(), stateTime.time());
            telemetry.addData("Pictograph", "%s", vuMark);
            telemetry.addData("Trial Counter", trialCounter);
            telemetry.update(); //Update Telemetry

            sleep(250);

        }
    }

    //Sets a new state, resets state clock, and reset trialCounter
    private void newState(State newState) {
        currentState = newState;
        stateTime.reset();
        trialCounter = 0;
    }

    //drive forward using encoders
    private void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // ensure that the opmode is still active
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

            // turn On RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            encoderTime.reset();
            robot.driveForward(Math.abs(speed));

            // wait for motors to not be busy
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy())) {

                // display it for the driver.
                telemetry.addData("Goal Position", "%7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Current Position", "%7d :%7d :%7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // stop all motion;
            robot.stopDriving();

            // turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }
}