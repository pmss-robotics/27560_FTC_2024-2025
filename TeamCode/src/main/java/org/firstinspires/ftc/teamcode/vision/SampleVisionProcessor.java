package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class SampleVisionProcessor implements VisionProcessor {

    public SampleVisionProcessor(){
        // constructor elements
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // we'll get someone trained in opencv and they'll write some code here that finds
        // some game feature
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // here we'll use that data to draw things on the preview
        // or do nothing and maybe try and merge to with the CameraStreamProcessor
    }
}
