package org.firstinspires.ftc.teamcode.auton.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    RED = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(50, 100, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 74, 255),
            lower_cyan_bounds    = new Scalar(0, 50, 75, 255),
            upper_cyan_bounds    = new Scalar(75, 255, 255, 255),
            lower_red_bounds = new Scalar(50, 0, 0, 255),
            upper_red_bounds = new Scalar(255, 70, 70, 255);

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            RED = new Scalar(255, 0, 0);

    // Percent and mat definitions
    private double yelPercent, cyaPercent, redPercent;
    private Mat yelMat = new Mat(), cyaMat = new Mat(), redMat = new Mat(), blurredMat = new Mat(), kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);

        // Gets color specific values
        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        redPercent = Core.countNonZero(redMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, redPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == yelPercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == cyaPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (maxPercent == redPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        yelMat.release();
        cyaMat.release();
        redMat.release();
        kernel.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}