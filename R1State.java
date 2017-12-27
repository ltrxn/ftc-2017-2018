package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Trxn on 12/17/2017.
 */

@Autonomous(name = "clout", group = "state")
public class R1State extends LinearOpMode {
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
    private double currentJewelKnockerDown = .57; //what to set down jewel knocker position as

    private int trialCounter = 0; //# of times tried before continuing

    //Vumark
    RelicRecoveryVuMark vuMark; //what column to score in

    //Encoders
    static final double ANDYMARK_TICKS_PER_REV = 1120; //# of ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = .5;   //Since gears go from big to small, one rotation of the gear is actually only half a rotation of the wheel
    static final double WHEEL_DIAMETER_INCHES = 4;   //Diameter of the wheel
    static final double TICKS_PER_INCH = (ANDYMARK_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); //# of ticks to be rotated to drive an inch

    static final double DRIVE_SPEED = .2; //Speed while going to crytobox
    static final double GYRO_TURN_SPEED = 0.5; //Speed while turning

    static final int THRESHOLD = 2; //tolerance when turning

    static final int DISTANCE_RIGHT = 20; //Distance from balancing stone to crytobox positions
    static final int DISTANCE_CENTER = 28;
    static final int DISTANCE_LEFT = 36;
    static final int DISTANCE_TO_CRYPTOBOX = 9; //Distance to push block to cryptobox.
    static final int DRIVE_TIME_OUT = 10;

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

        newState(State.STATE_INITIAL); //set currentState to initial
        robot.relicTrackables.activate(); //activate vuforia

        //Start loop
        while (opModeIsActive()) {
            //first line of telemetry, runt time and current state time
            telemetry.addData("Time", String.format("%4.f", stateTime.time()) + currentState.toString());
            telemetry.addData("Pictograph", vuMark);

            switch (currentState) {
                case STATE_INITIAL: //Remember VuMark and pickup glyph

                    if (robot.vuMarkIsVisable() || trialCounter > 3) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        vuMark = robot.getVuMark();

                        robot.closeClaw();
                        sleep(500);
                        robot.pulley.setPower(.3);
                        sleep(1500);
                        robot.pulley.setPower(0);

                        newState(State.STATE_KNOCK_JEWEL);

                    } else {

                        telemetry.addData("VuMark", "Is not visable");
                        trialCounter++;

                    }
                    break;


                case STATE_KNOCK_JEWEL: //knock off the blue jewel

                    robot.jewelKnockerRight.setPosition(currentJewelKnockerDown); //lower jewel knocker
                    sleep(1000);

                    redSensor = sensorColor.red(); //remember red value
                    blueSensor = sensorColor.blue(); //remember blue value
                    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR), //remember other values
                            (int) (sensorColor.green() * SCALE_FACTOR),
                            (int) (sensorColor.blue() * SCALE_FACTOR),
                            hsvValues);

                    sleep(1000);

                    if (blueSensor > 20 || redSensor > 20 || trialCounter > 3) { //if there is a strong blue/red return...

                        if (redSensor > blueSensor) { //if ball is red...
                            encoderDrive(.06, -5, 5, 5); //turn left
                            encoderDrive(.06, 4, -4, 5); //turn right
                        } else { //if ball is blue...
                            encoderDrive(.1, 5, -5, 5); //turn right
                            encoderDrive(.1, -4, 4, 5); //turn left
                        }

                        newState(State.STATE_DRIVE_TO_CRYPTOBOX);

                    } else {

                        currentJewelKnockerDown -= .02; //raise it up a bit
                        trialCounter++;

                    }
                    break;

                case STATE_DRIVE_TO_CRYPTOBOX: //drive till the cryptobox (according to vuforia)

                    robot.raiseJewelKnockerRight(); //lift up jewel knocker

                    if (robot.jewelKnockerRight.getPosition() < .3 || trialCounter > 3) { //if the jewel knocker is up...

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

                    } else {
                        robot.raiseJewelKnockerRight();
                        trialCounter++;
                    }

                    break;

                case STATE_FACE_CRYPTOBOX: //turn towards the cryptobox

                    if (robot.rightFront.getPower() == 0 || trialCounter > 3) { //make sure robot is not moving
                        encoderDrive(DRIVE_SPEED, 12, -12, DRIVE_TIME_OUT);

                        newState(State.STATE_SCORE);

                    } else {
                        trialCounter++;
                        robot.stopDriving(); //stop the driving
                        telemetry.addData("Motor", "is still driving");
                    }

                    break;

                case STATE_SCORE: //drive up to the cryptobox and release the glyph

                    if (robot.rightFront.getPower() == 0 || trialCounter > 3) { //make sure robot is not moving

                        encoderDrive(DRIVE_SPEED, DISTANCE_TO_CRYPTOBOX, DISTANCE_TO_CRYPTOBOX, DRIVE_TIME_OUT);

                        robot.pulley.setPower(-.3); //lower the pulley
                        sleep(1000);
                        robot.pulley.setPower(0);
                        robot.openClaw(); //open glyph claw

                        newState(State.STATE_STOP);

                    } else {
                        trialCounter++;
                        robot.stopDriving(); //stop the driving
                        telemetry.addData("Motor", "is still driving");
                    }

                    break;

                case STATE_STOP: //do nothing

                    if(robot.rightFront.getPower()==0 || trialCounter > 3) { //make sure robot is not moving
                        telemetry.addData("AUTONOMOUS", "COMPLETE");

                    } else {
                        trialCounter++;
                        robot.stopDriving(); //stop the driving

                        telemetry.addData("Motor", "is still driving");
                    }
                    break;
            }

            telemetry.update(); //Update Telemetry
        }
    }


    //Sets a new state and resets state clock
    private void newState(State newState) {
        currentState = newState;
        stateTime.reset();
        trialCounter = 0;
    }

    //Drives forward using encoders
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

            // wait for motors to not be busy
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

            sleep(100);   // optional pause after each move
        }
    }
}