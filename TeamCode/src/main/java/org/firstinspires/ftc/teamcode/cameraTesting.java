package org.firstinspires.ftc.teamcode;


import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;


@TeleOp(name="cameraTesting")
@Disabled

public class cameraTesting extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW
                        )
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();


        while(!isStarted() && !isStopRequested()){

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // Display the Color Sensor result.
            telemetry.addData("Best Match:", result.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            telemetry.update();

        }

        while (opModeIsActive())
        {

        }
    }
}
