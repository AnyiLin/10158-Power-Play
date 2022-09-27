package org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PersonTracker extends OpenCvPipeline {

    Telemetry telemetry;

    static final Scalar RED = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(255, 0, 0);

    public int redContourCount = 0;
    public int greenContourCount = 0;

    boolean topGreen = false;
    boolean bottomGreen = false;

    boolean overlapConditionOne = false;
    boolean overlapConditionTwo = false;

    boolean overlapCheckOne = false;
    boolean overlapCheckTwo = false;

    boolean distanceCheckOne = false;
    boolean distanceCheckTwo = false;

    int colorDistanceThreshold = 50;

    double maxArea;
    double previousMaxArea;
    public int maxArrayElement;

    public List<Rect> redRect;
    public List<Rect> filteredRedRect;
    public List<Rect> greenRect;

    public Rect RedRect;
    public Rect TargetRect;

    public List<MatOfPoint> redContours;
    public List<MatOfPoint> greenContours;

    public Rect biggestRedRect;

    public PersonTracker(Telemetry telemetry) {
        redContours = new ArrayList<MatOfPoint>();
        redRect = new ArrayList<Rect>();
        filteredRedRect = new ArrayList<Rect>();
        RedRect = new Rect();
        biggestRedRect = new Rect();

        greenContours = new ArrayList<MatOfPoint>();
        greenRect = new ArrayList<Rect>();

        TargetRect = new Rect();

        this.telemetry = telemetry;
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 50;
    }

    // Red masking thresholding values: TODO: re-threshold
    Scalar lowRed = new Scalar(90, 0, 0); //10, 100, 50
    Scalar highRed = new Scalar(150, 255, 255); //35, 255, 255

    // Green masking thresholding values: TODO: re-threshold
    Scalar lowGreen = new Scalar(0, 20, 10); //10, 100, 50
    Scalar highGreen = new Scalar(10, 255, 255); //35, 255, 255

    // Mat object for the red mask
    Mat maskRed = new Mat();

    // Mat object for the green mask
    Mat maskGreen = new Mat();

    // Mat object for HSV color space
    Mat HSV = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Imgproc.erode(HSV, HSV, kernel);

         // Finds the pixels within the thresholds and puts them in the mat object "maskYellow"
        inRange(HSV, lowRed, highRed, maskRed);
        inRange(HSV, lowGreen, highGreen, maskGreen);

        // Clears the arraylists
        redContours.clear();
        redRect.clear();
        greenContours.clear();
        greenRect.clear();

        // Finds the contours and draws them on the screen
        Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.findContours(maskGreen, greenContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < greenContours.size(); i++) {

            // Filters out contours with an area less than 50 (defined in the filter contours method)

                // Creates a bounding rect around each contourand the draws it
                greenRect.add(Imgproc.boundingRect(greenContours.get(i)));

        }

        // Iterates through each contour
        for (int i = 0; i < redContours.size(); i++) {

            // Filters out contours with an area less than 50 (defined in the filter contours method)

                // Creates a bounding rect around each contour and the draws it
                redRect.add(Imgproc.boundingRect(redContours.get(i)));

        }

        for (int i = 0; i < redRect.size(); i++){
            for (int j = 0; j < greenRect.size(); j++) {
                if (getRectCenterY(greenRect.get(j)) > getRectCenterY(redRect.get(i))){
                    topGreen = true;
                    overlapCheckOne = true;
                }

                if (getRectCenterY(greenRect.get(j)) < getRectCenterY(redRect.get(i))){
                    bottomGreen = true;
                    overlapCheckTwo = true;
                }

                if (greenRect.get(j).x < getRectCenterX(redRect.get(i)) && getRectCenterX(redRect.get(i)) < greenRect.get(j).x + greenRect.get(j).width && overlapCheckOne) {
                    overlapConditionOne = true;
                }

                if (greenRect.get(j).x < getRectCenterX(redRect.get(i)) && getRectCenterX(redRect.get(i)) < greenRect.get(j).x + greenRect.get(j).width && overlapCheckTwo) {
                    overlapConditionTwo = true;
                }

                if ((redRect.get(i).y - greenRect.get(j).y + greenRect.get(j).height < colorDistanceThreshold) && overlapCheckOne){
                    distanceCheckOne = true;
                }

                if ((greenRect.get(j).y + greenRect.get(j).height - redRect.get(i).y < colorDistanceThreshold) && overlapCheckTwo){
                    distanceCheckTwo = true;
                }

                overlapCheckOne = false;
                overlapCheckTwo = false;
            }

            if (topGreen && bottomGreen && overlapConditionOne && overlapConditionTwo && distanceCheckOne && distanceCheckTwo){
                filteredRedRect.add(redRect.get(i));
            }

            topGreen = false;
            bottomGreen = false;
            overlapConditionOne = false;
            overlapConditionTwo = false;
            distanceCheckOne = false;
            distanceCheckTwo = false;
        }

        maxArea = 0;
        maxArrayElement = 0;

        previousMaxArea = 0;

        for (int i = 0; i < filteredRedRect.size(); i++){
            if (filteredRedRect.get(i).area() > maxArea) maxArea = filteredRedRect.get(i).area();

            if (maxArea != previousMaxArea) maxArrayElement = i;

            previousMaxArea = maxArea;
        }


        if (filteredRedRect.size() > 0){
            TargetRect = filteredRedRect.get(maxArrayElement);

            Imgproc.rectangle(input, TargetRect, RED, 2);
        }

        telemetry.addData("Red Contour Count", redContourCount);

        // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
        for (int i = 0; i < redRect.size(); i++){
            telemetry.addData("Red Contour " + (i+1), "%7d,%7d", getRectCenterX(redRect.get(i)),getRectCenterY(redRect.get(i)));
        }

        maskRed.release();
        maskGreen.release();

        filteredRedRect.clear();

        greenContourCount = 0;
        redContourCount = 0;

        HSV.release();

        return input;
    }

    public int getRectCenterX(Rect rect){
        return rect.x + (rect.width/2);
    }

    public int getRectCenterY(Rect rect){
        return rect.y + (rect.height/2);
    }

}
