package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "HHHHHHH")
public class visiontest extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private SampleDetectionVisionProcessor sampleDetectionVisionProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        sampleDetectionVisionProcessor = new SampleDetectionVisionProcessor();
        telemetry.log().setCapacity(20);
        telemetry.log().add("hi", "hi");
        telemetry.update();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "C:\\Users\\jeffx\\Downloads\\samplesss.jpg"), sampleDetectionVisionProcessor);

        for (SampleDetectionVisionProcessor.AnalyzedStone sample : sampleDetectionVisionProcessor.getDetectedStones()){
            telemetry.log().add("tvec: " + sample.color.toString() + " : ",  sample.tvec.toString());
            telemetry.log().add("1", "1");
            telemetry.update();
        }
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        if (opModeIsActive()) {
            // ...
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}
