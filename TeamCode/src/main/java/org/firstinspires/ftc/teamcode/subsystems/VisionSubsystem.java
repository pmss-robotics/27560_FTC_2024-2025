package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.SampleVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class VisionSubsystem extends SubsystemBase {

    public static int EXPOSURE_MS = 6;
    public static int GAIN = 190;
    private final AprilTagProcessor aprilTag;
    private final SampleVisionProcessor sampleVisionProcessor;
    private final VisionPortal visionPortal;
    private Telemetry telemetry;

    /**
     * All vision related logic will be done here including april tags, tensorflow, and opencv
     * @param hardwareMap
     */
    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        this.telemetry = telemetry;
        // TODO sample code!!!! re-implement in the field.
        final CameraStreamProcessor dashboard = new CameraStreamProcessor();
        sampleVisionProcessor = new SampleVisionProcessor();
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(dashboard, aprilTag, sampleVisionProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // worse compression than the default but faster
                .build();

        // some hacky logic to make sure the camera is fully ready before we put in our configs
        // otherwise it'll just crash, which is bad
        // it might need some changes maybe
        ElapsedTime elapsedTime = new ElapsedTime();
        VisionPortal.CameraState cameraState = visionPortal.getCameraState();
        while(elapsedTime.seconds() < 20 && cameraState != VisionPortal.CameraState.STREAMING){
            cameraState = visionPortal.getCameraState();
        }
        telemetry.log().add("Time from build to streaming " + elapsedTime.milliseconds() + "ms");
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(GAIN);
            sleep(20);
        }
        FtcDashboard.getInstance().startCameraStream(dashboard, 30);
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}

