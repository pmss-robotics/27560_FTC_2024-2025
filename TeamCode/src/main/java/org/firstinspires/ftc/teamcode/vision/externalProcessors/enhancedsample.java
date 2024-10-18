package org.firstinspires.ftc.teamcode.vision.externalProcessors;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;

public class enhancedsample extends OpenCvPipeline {

    public enum SampleColor {
        RED, BLUE, YELLOW
    }

    public static class DetectedSample {
        public final SampleColor color;
        public final RotatedRect orientedRect;
        public final double area;
        public final Mat rvec;
        public final Mat tvec;
        public final double angle;

        public DetectedSample(SampleColor color, RotatedRect orientedRect, double area, Mat rvec, Mat tvec, double angle) {
            this.color = color;
            this.orientedRect = orientedRect;
            this.area = area;
            this.rvec = rvec;
            this.tvec = tvec;
            this.angle = angle;
        }
    }

    private static final Map<SampleColor, Scalar[]> COLOR_BOUNDS = new EnumMap<>(SampleColor.class);
    static {
        COLOR_BOUNDS.put(SampleColor.RED, new Scalar[]{new Scalar(0, 100, 100), new Scalar(10, 255, 255)});
        COLOR_BOUNDS.put(SampleColor.BLUE, new Scalar[]{new Scalar(100, 100, 100), new Scalar(130, 255, 255)});
        COLOR_BOUNDS.put(SampleColor.YELLOW, new Scalar[]{new Scalar(20, 100, 100), new Scalar(30, 255, 255)});
    }

    private static final Map<SampleColor, Scalar> CONTOUR_COLORS = new EnumMap<>(SampleColor.class);
    static {
        CONTOUR_COLORS.put(SampleColor.RED, new Scalar(255, 0, 0));
        CONTOUR_COLORS.put(SampleColor.BLUE, new Scalar(0, 0, 255));
        CONTOUR_COLORS.put(SampleColor.YELLOW, new Scalar(255, 255, 0));
    }

    private final Mat hsvMat = new Mat();
    private final Mat thresholdMat = new Mat();
    private final Mat contoursOnFrame = new Mat();

    private double minArea = 1000;
    private boolean drawContours = true;

    private final List<DetectedSample> detectedSamples = new ArrayList<>();

    private final Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    private final MatOfDouble distCoeffs = new MatOfDouble();

    private static final double OBJECT_WIDTH = 10.0;
    private static final double OBJECT_HEIGHT = 5.0;

    public enhancedsample() {
        double fx = 800;
        double fy = 800;
        double cx = 320;
        double cy = 240;

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        distCoeffs.fromArray(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void setMinArea(double area) {
        this.minArea = area;
    }

    public void setDrawContours(boolean draw) {
        this.drawContours = draw;
    }

    public List<DetectedSample> getDetectedSamples() {
        return Collections.unmodifiableList(detectedSamples);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        input.copyTo(contoursOnFrame);

        detectedSamples.clear();

        for (SampleColor color : SampleColor.values()) {
            processColor(color, input);
        }

        return contoursOnFrame;
    }

    private void processColor(SampleColor color, Mat input) {
        Scalar[] bounds = COLOR_BOUNDS.get(color);
        Core.inRange(hsvMat, bounds[0], bounds[1], thresholdMat);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                MatOfPoint3f objectPoints = getObjectPoints();
                MatOfPoint2f imagePoints = getImagePoints(rotatedRect);
                Mat rvec = new Mat();
                Mat tvec = new Mat();

                Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

                double angle = calculateAngle(rotatedRect);

                detectedSamples.add(new DetectedSample(color, rotatedRect, area, rvec, tvec, angle));

                if (drawContours) {
                    drawDetectedSample(color, rotatedRect, area, angle, input);
                    drawAxis(input, rvec, tvec);
                }
            }
        }
    }

    private void drawDetectedSample(SampleColor color, RotatedRect rotatedRect, double area, double angle, Mat input) {
        drawRotatedRect(contoursOnFrame, rotatedRect, CONTOUR_COLORS.get(color));
        String label = String.format("%s %.0f %.1f°", color.name(), area, angle);
        Point labelPosition = new Point(rotatedRect.center.x, rotatedRect.center.y - 20);
        Imgproc.putText(contoursOnFrame, label, labelPosition, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, CONTOUR_COLORS.get(color), 2);
    }

    private void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
        }
    }

    private void drawAxis(Mat img, Mat rvec, Mat tvec) {
        MatOfPoint3f axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(3, 0, 0),
                new Point3(0, 3, 0),
                new Point3(0, 0, -3)
        );

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();

        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 2);
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 2);
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 2);
    }

    private MatOfPoint3f getObjectPoints() {
        return new MatOfPoint3f(
                new Point3(-OBJECT_WIDTH / 2, -OBJECT_HEIGHT / 2, 0),
                new Point3(OBJECT_WIDTH / 2, -OBJECT_HEIGHT / 2, 0),
                new Point3(OBJECT_WIDTH / 2, OBJECT_HEIGHT / 2, 0),
                new Point3(-OBJECT_WIDTH / 2, OBJECT_HEIGHT / 2, 0)
        );
    }

    private MatOfPoint2f getImagePoints(RotatedRect rect) {
        Point[] rectPoints = new Point[4];
        rect.points(rectPoints);
        return new MatOfPoint2f(orderPoints(rectPoints));
    }

    private double calculateAngle(RotatedRect rect) {
        double angle = rect.angle;
        if (rect.size.width < rect.size.height) {
            angle += 90;
        }
        return -(angle - 180);
    }

    private static Point[] orderPoints(Point[] pts) {
        Point[] orderedPts = new Point[4];
        double[] sum = new double[4];
        double[] diff = new double[4];

        for (int i = 0; i < 4; i++) {
            sum[i] = pts[i].x + pts[i].y;
            diff[i] = pts[i].y - pts[i].x;
        }

        orderedPts[0] = pts[indexOfMin(sum)];
        orderedPts[2] = pts[indexOfMax(sum)];
        orderedPts[1] = pts[indexOfMin(diff)];
        orderedPts[3] = pts[indexOfMax(diff)];

        return orderedPts;
    }

    private static int indexOfMin(double[] array) {
        return indexOfExtreme(array, true);
    }

    private static int indexOfMax(double[] array) {
        return indexOfExtreme(array, false);
    }

    private static int indexOfExtreme(double[] array, boolean findMin) {
        int index = 0;
        double extreme = array[0];

        for (int i = 1; i < array.length; i++) {
            if ((findMin && array[i] < extreme) || (!findMin && array[i] > extreme)) {
                extreme = array[i];
                index = i;
            }
        }
        return index;
    }
}