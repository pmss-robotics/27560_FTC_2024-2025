package org.firstinspires.ftc.teamcode.vision;
// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;


public class duckyPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    Mat yellowmat = new Mat();

    public duckyPipeline(Telemetry tel)
    {
        telemetry=tel;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar yellowhighHSV = new Scalar(30, 255, 255);
        Scalar yellowlowHSV = new Scalar(20, 100, 100);

        Rect RECT_LEFT = new Rect(
                new Point(60, 160),
                new Point(100, 110));
        Rect RECT_MIDDLE = new Rect(
                new Point(130, 160),
                new Point(180, 110));
        Rect RECT_RIGHT = new Rect(
                new Point(220, 155),
                new Point(270, 105));
        Scalar color = new Scalar(64, 64, 64);
        Imgproc.rectangle(mat, RECT_LEFT, color, 2);
        Imgproc.rectangle(mat, RECT_MIDDLE, color, 2);
        Imgproc.rectangle(mat, RECT_RIGHT, color,2);

        Core.inRange(mat, yellowlowHSV, yellowhighHSV, yellowmat);
        Mat left = yellowmat.submat(RECT_LEFT);
        Mat middle = yellowmat.submat(RECT_MIDDLE);
        Mat right = yellowmat.submat(RECT_RIGHT);

        double leftValue = Core.sumElems(left).val[0];
        double middleValue = Core.sumElems(middle).val[0];
        double rightValue = Core.sumElems(right).val[0];

        if (leftValue > middleValue || leftValue > rightValue) {
            telemetry.addData("Duck", "Left");
        }
        if (middleValue > leftValue || middleValue > rightValue) {
            telemetry.addData("Duck", "Middle");
        }
        if (rightValue > middleValue || rightValue > leftValue) {
            telemetry.addData("Duck", "Right");
        }
        telemetry.addData("test", "test");
        telemetry.addData("leftValue", leftValue + "");
        telemetry.addData("middleValue", middleValue + "");
        telemetry.addData("rightValue", rightValue + "");
        telemetry.update();

        return mat;
    }
}
