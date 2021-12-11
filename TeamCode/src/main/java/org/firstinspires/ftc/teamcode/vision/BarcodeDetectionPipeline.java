package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A pipeline to attempt to find the three barcode red or blue dots on the field
 * in front of the robot
 *
 * Started with the FTCLib example code for UGBasicHighCoalPipeline,
 * because that detects red and blue rectangles...
 *
 */

public class BarcodeDetectionPipeline extends OpenCvPipeline {

    // segment (vertically) of image to analyze
    private static final int VIEW_STARTING_Y_PERCENT = 70;
    private static final int VIEW_ENDING_Y_PERCENT = 100;

    private Telemetry telemetry;

    protected double centerX;
    protected double centerY;
    protected int minY, maxY;

    // public variables can be tweaked in EOCVSim
    public int minThreshold, maxThreshold, minBCArea, maxBCArea, maxBCSides = 5;
    public int minSUArea = 1000, maxSUArea = 7000, minSUSides = 6, maxSUSides = 12;

    private Mat blueThreshold;
    private Mat redThreshold;

    private Mat matYCrCb;
    private Mat redChannel;
    private Mat blueChannel;

    private List<MatOfPoint> redContours;
    private List<MatOfPoint> blueContours;

    private int duckPosition = 0;
    private boolean storageUnitVisible = false;

    public BarcodeDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        matYCrCb = new Mat();
        redChannel = new Mat();
        blueChannel = new Mat();

        blueThreshold = new Mat();
        redThreshold = new Mat();

        blueContours = new ArrayList<MatOfPoint>();
        redContours = new ArrayList<MatOfPoint>();

        minThreshold = 150;
        maxThreshold = 190;
        minBCArea = 400;
        maxBCArea = 800;

    }

    @Override
    public void init(Mat mat) {

        super.init(mat);
        int imageWidth = mat.width();
        int imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;

        minY = imageHeight * VIEW_STARTING_Y_PERCENT / 100;
        maxY = imageHeight * VIEW_ENDING_Y_PERCENT / 100;

    }

    // Return true of the contour (shape) is a 4 sided polygon and is within the required size
    public boolean filterContours(MatOfPoint contour) {
        double a = 0;

        // find the approximate polygon..
        MatOfPoint2f poly = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), poly, 3, true);

        int sideCount = poly.toArray().length;
        int area = (int)Imgproc.contourArea(contour);

        Log.i("PIPELINE", "contour area="+area+", sides=" + sideCount);

        // check if it matches for the piece of storage unit tape
        if ( (sideCount >= minSUSides)
              && (sideCount <= maxSUSides)
              && (area > minSUArea)
              && (area < maxSUArea)
        ) {
            storageUnitVisible = true;
        }

        if (sideCount > 3 && sideCount <= maxBCSides ) {

//            List<MatOfPoint> polyPoints = new ArrayList<>();
//            polyPoints.add(new MatOfPoint(poly.toArray() ) );
//            Imgproc.drawContours(input, polyPoints, -1, new Scalar(255, 0, 0), 1);

            a = Imgproc.contourArea(contour);
  //          telemetry.addData("Contour area", a);

            // accept the item if area within range
            return (minBCArea < a) && (a < maxBCArea);

        }
        return false;
    }

    @Override
    public Mat processFrame(Mat input) {

            // Convert image format to YCrCb format to make it easier
            // to extract red and blue channels


            Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Make a rectangle for the area we are not interested in, and blank it
            Rect r = new Rect();
            r.x = 0;
            r.y = 0;
            r.width = matYCrCb.width();
            r.height = minY;

            Imgproc.rectangle(matYCrCb, r, new Scalar(0, 0, 0), Imgproc.FILLED);

            r.y = maxY;
            r.height = matYCrCb.height() - maxY;
            Imgproc.rectangle(matYCrCb, r, new Scalar(0, 0, 0), Imgproc.FILLED);

            Core.extractChannel(matYCrCb, redChannel, 1);
            Core.extractChannel(matYCrCb, blueChannel, 2);

            // Filter the blue channel to only blue levels within  threshold
            Imgproc.threshold(blueChannel, blueThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
            // And same for Red..
            Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);


        // Make contour objects for red and blue
            blueContours.clear();
            redContours.clear();

            // Find the contours around red and blue areas filtered out previously

            Imgproc.findContours(blueThreshold, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

//        Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));

            // if a shape that matches a piece of the storage unit is found, the following
            // will be changed to true during the filter process
            storageUnitVisible = false;

            blueContours = blueContours.stream().filter(i -> filterContours(i)).collect(Collectors.toList());

            // same for red..
            redContours = redContours.stream().filter(i -> filterContours(i)).collect(Collectors.toList());

//            Mat x = new Mat();
//        Imgproc.cvtColor(blueThreshold, x, Imgproc.COLOR_GRAY2RGB);
//        Imgproc.drawContours(x, blueContours, -1, new Scalar(255, 0, 255), 2);
//        if (true) return x;


//        redContours = redContours.stream().filter(i -> {
//            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height > 1)
            //                   && ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height < 2);
            //           return filterContours(i) && appropriateAspect;
            //       }).collect(Collectors.toList());

            // draw the outline of the remaining contours onto the image
            Imgproc.drawContours(input, redContours, -1, new Scalar(255, 255, 0));
            Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));

            // determine which barcodes are present
            List<MatOfPoint> barcodes = (blueContours.isEmpty() ? redContours : blueContours);

            // draw the bounding rectangles on each
//            for(MatOfPoint b : barcodes) {
//                Imgproc.rectangle(input,  Imgproc.boundingRect(b), new Scalar(0,255,255) );
//            }

            // Draw where the X limits are
            int x1 = (int)(centerX * 0.6);
            int x2 = (int)(centerX * 1.2);

            Imgproc.line(input, new Point(x1, minY), new Point(x1, maxY), new Scalar(0,255,0) );
        Imgproc.line(input, new Point(x2, minY), new Point(x2, maxY), new Scalar(0,255,0) );

        boolean haveLeft = barcodes.stream().anyMatch(i -> Imgproc.boundingRect(i).x < x1);
            boolean haveRight = barcodes.stream().anyMatch(i -> Imgproc.boundingRect(i).x > x2);


            // figure out duck position 1, 2 or 3..
            // blue square nearing storage unit is level 3, furthest is level 1
            // red square nearest storage unit is level 1, furthest is level 3


            duckPosition = (isBlueVisible() || isRedVisible()) ? (haveLeft ? (haveRight ? 2 : 3) : 1) : 0;
            telemetry.addData("duckPos", duckPosition);

/*
        if (!blueContours.isEmpty()) {

            boolean haveLeft = blueContours.stream().anyMatch( i -> Imgproc.boundingRect(i).x < (centerX * 0.7) );
            boolean haveRight = blueContours.stream().anyMatch( i -> Imgproc.boundingRect(i).x > (centerX * 1.3) );

            // figure out duck position 1, 2 or 3..

            duckPosition = haveLeft ? (haveRight ? 2 : 3) : 1;
            telemetry.addData("duckPos", duckPosition);

            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
//            biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
         //       return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            // });
//            blueRect = Imgproc.boundingRect(biggestBlueContour);
   //         Imgproc.rectangle(input, blueRect, new Scalar(0, 0, 255), 3);
        } else {
            blueRect = null;
        }

        // same for red

        if (!redContours.isEmpty()) {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            redRect = Imgproc.boundingRect(biggestRedContour);
            Imgproc.rectangle(input, redRect, new Scalar(255, 0, 0), 3);
        } else {
            redRect = null;
        }
*/
            telemetry.addData("Red visible", isRedVisible());
            telemetry.addData("Blue visible", isBlueVisible());
            telemetry.addData("Storage unit visible", isStorageUnitVisible());
            telemetry.update();

        r.y = minY;
        r.height = maxY - minY;
        Imgproc.rectangle(input, r, new Scalar(0, 255, 0));

        return input; //  blueThreshold; // blueChannel; //  matYCrCb; // input;
    }

    public int duckPos() {
        return duckPosition;
    }

    public boolean isRedVisible() {
        return (redContours != null && redContours.size() >= 2);
    }

    public boolean isBlueVisible() {
        return (blueContours != null && blueContours.size() >= 2);
    }

    public boolean isStorageUnitVisible() {
        return storageUnitVisible;
    }

}
