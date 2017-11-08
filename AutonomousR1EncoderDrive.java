package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Liwen Trxn on 10/19/2017.
 */
@Autonomous(name="R1 - Encoder", group="real")
@Disabled

public class AutonomousR1EncoderDrive extends LinearOpMode {

    //robot
    HardwareIdeal robot = new HardwareIdeal();


    //values
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

        robot.init(hardwareMap);

        //grab the glyph before game starts
        robot.closeGlyphClaw();

        telemetry.addData(">", "Initialization finished. Press play to start");
        telemetry.update();



        waitForStart();

        /**********************************************************
         ****************** START OF AUTONOMOUS *******************
         **********************************************************/



        robot.relicTrackables.activate();

        while (opModeIsActive()) {

            //Knock off the BLUE jewel
            robot.turnOnColorSensorRight(); //turn on color sensor
            robot.lowerJewelKnockerRight();
            if(robot.rightColorSensor.red()> robot.rightColorSensor.blue()) { //if is to the right of the jewel
                //move the robot right
                gyroAbsoluteTurn(30);
                gyroAbsoluteTurn(0);

            } else {
                //move the robot left
                gyroAbsoluteTurn(-30);
                gyroAbsoluteTurn(0);

            }



                //if a pictograph is visible...
            if (robot.getVuMark() != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", robot.getVuMark());

                if (robot.getVuMark() == RelicRecoveryVuMark.RIGHT) {
                    //drive in front of the right column
                    scoreRight();
                } else if (robot.getVuMark() == RelicRecoveryVuMark.CENTER) {
                    //drive in front of the center column
                    scoreCenter();

                } else if (robot.getVuMark() == RelicRecoveryVuMark.LEFT) {
                    //drive in front of the left column
                    scoreLeft();
                }

            } else {
                telemetry.addData("Vumark", "is not visable");
            }

            //Release the glyph ***UNSURE HOW THIS WILL WORK***
            robot.openGlyphClaw();

            //ROBOT SHOULD BE IN SAFE ZONE, AUTONOMOUS OVER

            idle();
        }
    }


    /**********************************************************
     ************************ METHODS**************************
     **********************************************************/

    //VUFORIA
    private void scoreRight() throws InterruptedException {
        /**score into right column
         * To Do:
         * drive forward ~20" (rightCryptoboxDistance), turn 90 degrees right, drive forward ~12"
         */

        robot.encoderDrive(cryptoboxDrivePower, rightCryptoboxDistance); //Drive 20 inches at 0.5 power

        try {
            gyroTurn(90); // turn 90 degrees right
        } catch (InterruptedException e) {
            telemetry.addData("GyroTurn: ", "Failed");// if GyroTurn fails, send message to driver station.
        }

        robot.encoderDrive(cryptoboxDrivePower, 12); //Drive 12 inches at 0.5 power;

    }

    private void scoreCenter() throws InterruptedException {
        /**score into center column
         * To Do:
         * drive forward ~28"(centerCryptoboxDistance), turn 90 degrees right, drive forward ~12"
         */

        robot.encoderDrive(cryptoboxDrivePower, centerCryptoboxDistance); //Drive 28 inches at 0.5 power;

        try {
            gyroTurn(90); // turn 90 degrees right
        } catch (InterruptedException e) {
            telemetry.addData("GyroTurn: ", "Failed");// if GyroTurn fails, send message to driver station.
        }

        robot.encoderDrive(cryptoboxDrivePower, 12); //Drive 12 inches at 0.5 power;

    }

    private void scoreLeft() {
        /**score into left column
         * To Do:
         * drive forward ~36" (leftCryptoboxDistance), turn 90 degrees right, drive forward ~12"
         */

        robot.encoderDrive(cryptoboxDrivePower, leftCryptoboxDistance); //Drive 36 inches at 0.5 power;

        try {
            gyroTurn(90); // turn 90 degrees right
        } catch (InterruptedException e) {
            telemetry.addData("GyroTurn: ", "Failed"); // if GyroTurn fails, send message to driver station.
        }

        robot.encoderDrive(cryptoboxDrivePower, 12); //Drive 12 inches at 0.5 power;

    }

    //TURN
    public void gyroTurn(int target) throws InterruptedException {
        //Use this method to turn. Parameter: target = how many degrees to turn.
        gyroAbsoluteTurn(target + robot.mrGyro.getIntegratedZValue());
    }

    public void gyroAbsoluteTurn(int target) throws InterruptedException {
        //Turns robot to the target from where the robot started at
        //DO NOT USE THIS METHOD TO TURN
        zAccumulated = robot.mrGyro.getIntegratedZValue(); //set variable to gyro readings
        while (Math.abs(zAccumulated-target) > 3) { //run this loop if robot is 3 degrees or more away from target
            if (zAccumulated > target) {  //if gyro is positive, we will turn left
                robot.leftFront.setPower(-turnSpeed);
                robot.rightFront.setPower(turnSpeed);
                robot.leftBack.setPower(-turnSpeed);
                robot.rightBack.setPower(turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is negative, we will turn right
                robot.leftFront.setPower(turnSpeed);
                robot.rightFront.setPower(-turnSpeed);
                robot.leftBack.setPower(turnSpeed);
                robot.rightBack.setPower(-turnSpeed);
            }
            idle();
            zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        }
        robot.stopDriving();

        telemetry.addData("accu", String.format("%03d", zAccumulated));
        telemetry.update();

        idle();
    }
}
