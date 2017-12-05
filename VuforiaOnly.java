package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Trxn on 12/5/2017.
 */

public class VuforiaOnly extends LinearOpMode{
    public VuforiaLocalizer vuforia;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASqeker/////AAAAGXyaxZaNe0ngmcBSZn2x/aMgngJtpaXLUuvWMrmqLi768fIwOZ90EwGzH1f8bs6XPrz7WgklDoQCfX9QmfDMh6MxdyzlDpt5KFjmHrHrPgFi+qnZvD9qgfY4gTH8epM5T/tt7BJHwoNC4hjk9+Jf1Ane+XS6AXZrVf04wCynRUzE64zQDgGxflNMl73Q5qSd7BN3OvkAymgrQE4dsg1S97o7sX+obj+ubPZoJ7Hh8KriU6iOHmrVyTx6epZG2lWXDK0Iv6cQjku7Z5hvZNnK9Uvv8TRNIz5R71PBahS0nR4Xn0FH3beyMc+iu6ZNv33ZRuSh9MCPIUlzquqZtvMMPGbg3880nFmLkS9ytLn90kdX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // not necessary

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("VuMark", "Is not visable");
        }
        telemetry.update();
        idle();
    }
}
