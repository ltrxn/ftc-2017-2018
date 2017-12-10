package org.firstinspires.ftc.teamcode.unofficial;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.HardwareMichaelScott;

/**
 * Created by Trxn on 11/7/2017.
 */

@Autonomous(name = "Autonomous Testing", group = "testing")
public class AutonomousForTesting extends LinearOpMode {
    HardwareMichaelScott robot = new HardwareMichaelScott();

    //Values
    static final double WHEEL_DIAMETER = 4.0; //In inches
    static final int ANDYMARK_TICKS_PER_REV = 1120;
    static final int TETRIX_TICK_PER_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final double TICKS_PER_INCH = (TETRIX_TICK_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * Math.PI); //Number of ticks in each inch (# of ticks in one rotation divided by the circumference of the wheel)
    int zAccumulated; //total rotation left/right

    int rightCryptoboxDistance = 20;
    int centerCryptoboxDistance = 28;
    int leftCryptoboxDistance = 36;
    double cryptoboxDrivePower = .5; //how fast to drive when going to cryptobox
    double turnSpeed = .2;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData(">", "초기화가 완료되었습니다 (Initialization is finished)");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("RightJewelKnocker", robot.jewelKnockerRight.getPosition());

            robot.relicTrackables.activate();
            RelicRecoveryVuMark vuMark = robot.getVuMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "Is not visable");
            }
            telemetry.update();

        }
        idle();
    }
}
