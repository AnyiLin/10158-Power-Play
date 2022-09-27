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
import java.util.Collections;
import java.util.List;

public class ConeTracker extends OpenCvPipeline {
    Telemetry telemetry;

    static final Scalar GREENSCALAR = new Scalar(0, 255, 0);

    // These are the boolean toggles for each color the code will track
    boolean ORANGE = false;
    boolean YELLOW = false;
    boolean GREEN = false;
    boolean BLUE = true;
    boolean RED = true;
    boolean PINK = false;
    boolean BLACK = false;
    boolean WHITE = false;

    // These are int variables that keep track of how many contours of a specific color there are
    public int orangeContourCount = 0;
    public int yellowContourCount = 0;
    public int greenContourCount = 0;
    public int blueContourCount = 0;
    public int redContourCount = 0;
    public int pinkContourCount = 0;
    public int whiteContourCount = 0;
    public int blackContourCount = 0;

    // Creating lists for all the bounding Rects of each color
    public List<Rect> orangeRect;
    public List<Rect> yellowRect;
    public List<Rect> greenRect;
    public List<Rect> blueRect;
    public List<Rect> redRect;
    public List<Rect> pinkRect;
    public List<Rect> whiteRect;
    public List<Rect> blackRect;

    public Rect OrangeRect;
    public Rect YellowRect;
    public Rect GreenRect;
    public Rect BlueRect;
    public Rect RedRect;
    public Rect PinkRect;
    public Rect WhiteRect;
    public Rect BlackRect;

    // Creating lists for all the MatOfPoints of each contour
    public List<MatOfPoint> redContours;
    public List<MatOfPoint> orangeContours;
    public List<MatOfPoint> yellowContours;
    public List<MatOfPoint> greenContours;
    public List<MatOfPoint> blueContours;
    public List<MatOfPoint> pinkContours;
    public List<MatOfPoint> whiteContours;
    public List<MatOfPoint> blackContours;

    public MatOfPoint biggestYellowContour;
    public MatOfPoint biggestOrangeContour;
    public MatOfPoint biggestRedContour;
    public MatOfPoint biggestBlueContour;
    public MatOfPoint biggestWhiteContour;
    public MatOfPoint biggestBlackContour;
    public MatOfPoint biggestPinkContour;
    public MatOfPoint biggestGreenContour;

    public ConeTracker(Telemetry telemetry, String color) {
        redContours = new ArrayList<MatOfPoint>();
        redRect = new ArrayList<Rect>();
        RedRect = new Rect();
        biggestRedContour = new MatOfPoint();

        blueContours = new ArrayList<MatOfPoint>();
        blueRect = new ArrayList<Rect>();
        BlueRect = new Rect();
        biggestBlueContour = new MatOfPoint();

        this.telemetry = telemetry;

        setColor(color);
    }

    public void setColor(String color)
    {
        ORANGE = false;
        RED = false;
        BLUE = false;
        GREEN = false;
        PINK = false;
        YELLOW = false;
        WHITE = false;
        BLACK = false;

        switch(color.toLowerCase())
        {
            case ("blue"):
                BLUE = true;
                break;
            case ("yellow"):
                YELLOW = true;
                break;
            case ("green"):
                GREEN = true;
                break;
            case ("pink"):
                PINK = true;
                break;
            case ("white"):
                WHITE = true;
                break;
            case ("black"):
                BLACK = true;
                break;
            case ("red"):
                RED = true;
                break;
            case ("orange"):
                ORANGE = true;
                break;
            default:
                break;
        }
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 50;
    }

    // Orange masking thresholding values:
    Scalar lowOrange = new Scalar(5, 100, 175); // 160, 170, 80 || 160, 170, 0
    Scalar highOrange = new Scalar(15, 255, 255); //180, 240, 220

    // Mat object for the orange mask
    Mat maskOrange = new Mat();

    // Yellow masking thresholding values:
    Scalar lowYellow = new Scalar(10, 100, 50); //180
    Scalar highYellow = new Scalar(35, 255, 255);

    // Mat object for the yellow mask
    Mat maskYellow = new Mat();

    // Green masking thresholding values:
    Scalar lowGreen = new Scalar(30, 0, 0); //38, 0, 0
    Scalar highGreen = new Scalar(70, 255, 255); //70, 255, 255

    // Mat object for the yellow mask
    Mat maskGreen = new Mat();

    // Blue masking thresholding values:
    Scalar lowBlue = new Scalar(90, 40, 100); // 90, 160, 20
    Scalar highBlue = new Scalar(255, 255, 255); // 130, 255, 255

    // Mat object for the blue mask
    Mat maskBlue = new Mat();

    // Red masking thresholding values:
    Scalar lowRed = new Scalar(0, 20, 10); // 160, 170, 80 || 160, 140, 10
    Scalar highRed = new Scalar(5, 255, 255); //179, 255, 220

    // Mat object for the red mask
    Mat maskRed = new Mat();

    // Pink masking thresholding values:
    Scalar lowPink = new Scalar(100, 20, 100); // 160, 10, 225
    Scalar highPink = new Scalar(179, 100, 255); // 180, 255, 255

    // Mat object for the pink mask
    Mat maskPink = new Mat();

    // White masking thresholding values:
    Scalar lowWhite = new Scalar(0, 0, 160); // 0, 0, 0
    Scalar highWhite = new Scalar(179, 15, 255); // 179, 15, 230

    // Mat object for the white mask
    Mat maskWhite = new Mat();

    // Black masking thresholding values:
    Scalar lowBlack = new Scalar(0, 0, 0);
    Scalar highBlack = new Scalar(179, 40, 100);

    // Mat object for the black mask
    Mat maskBlack = new Mat();

    // Mat object for YCrCb color space
    Mat YCrCb = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(YCrCb, YCrCb, kernel);

        if (RED) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowRed, highRed, maskRed);

            // Clears the arraylists
            redContours.clear();
            redRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < redContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(redContours.get(i))) {
                    biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    RedRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, RedRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Red Contour ", "%7d,%7d", RedRect.x + (RedRect.width/2), RedRect.y + (RedRect.height/2));

            maskRed.release();
        }

        if (BLUE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
            inRange(YCrCb, lowBlue, highBlue, maskBlue);

            // Clears the arraylists
            blueContours.clear();
            blueRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blueContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < blueContours.size(); i++) {
                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(blueContours.get(i))) {
                    biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    BlueRect = Imgproc.boundingRect(biggestBlueContour);
                    Imgproc.rectangle(input, BlueRect, GREENSCALAR      , 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Blue Contour ", "%7d,%7d", BlueRect.x + (BlueRect.width/2), BlueRect.y + (BlueRect.height/2));

            maskBlue.release();
        }

        if (PINK) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowPink, highPink, maskPink);

            // Clears the arraylists
            pinkContours.clear();
            pinkRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskPink, pinkContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, pinkContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < pinkContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(pinkContours.get(i))) {
                    biggestPinkContour = Collections.max(pinkContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    PinkRect = Imgproc.boundingRect(biggestPinkContour);

                    Imgproc.rectangle(input, PinkRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Pink Contour ", "%7d,%7d", PinkRect.x + (PinkRect.width/2), PinkRect.y + (PinkRect.height/2));

            maskPink.release();
        }

        if (GREEN) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowGreen, highGreen, maskGreen);

            // Clears the arraylists
            greenContours.clear();
            greenRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskGreen, greenContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, greenContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < greenContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(greenContours.get(i))) {
                    biggestGreenContour = Collections.max(greenContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    GreenRect = Imgproc.boundingRect(biggestGreenContour);

                    Imgproc.rectangle(input, GreenRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Green Contour ", "%7d,%7d", GreenRect.x + (GreenRect.width/2), GreenRect.y + (GreenRect.height/2));

            maskGreen.release();
        }

        if (ORANGE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowOrange, highOrange, maskOrange);

            // Clears the arraylists
            orangeContours.clear();
            orangeRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskOrange, orangeContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, orangeContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < orangeContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(orangeContours.get(i))) {
                    biggestOrangeContour = Collections.max(orangeContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    OrangeRect = Imgproc.boundingRect(biggestOrangeContour);

                    Imgproc.rectangle(input, OrangeRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Orange Contour ", "%7d,%7d", OrangeRect.x + (OrangeRect.width/2), OrangeRect.y + (OrangeRect.height/2));

            maskOrange.release();
        }

        if (YELLOW) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowYellow, highYellow, maskYellow);

            // Clears the arraylists
            yellowContours.clear();
            yellowRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskYellow, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, yellowContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < yellowContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(yellowContours.get(i))) {
                    biggestYellowContour = Collections.max(yellowContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    YellowRect = Imgproc.boundingRect(biggestYellowContour);

                    Imgproc.rectangle(input, YellowRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Yellow Contour ", "%7d,%7d", YellowRect.x + (YellowRect.width/2), YellowRect.y + (YellowRect.height/2));

            maskYellow.release();
        }

        if (WHITE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowWhite, highWhite, maskWhite);

            // Clears the arraylists
            whiteContours.clear();
            whiteRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskWhite, whiteContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, whiteContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < whiteContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(whiteContours.get(i))) {
                    biggestWhiteContour = Collections.max(whiteContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    WhiteRect = Imgproc.boundingRect(biggestWhiteContour);

                    Imgproc.rectangle(input, WhiteRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("White Contour ", "%7d,%7d", WhiteRect.x + (WhiteRect.width/2), WhiteRect.y + (WhiteRect.height/2));

            maskWhite.release();
        }

        if (BLACK) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowBlack, highBlack, maskBlack);

            // Clears the arraylists
            blackContours.clear();
            blackRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskBlack, blackContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blackContours, -1, GREENSCALAR); //input

            // Iterates through each contour
            for (int i = 0; i < blackContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(blackContours.get(i))) {
                    biggestBlackContour = Collections.max(blackContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contour and the draws it
                    BlackRect = Imgproc.boundingRect(biggestBlackContour);

                    Imgproc.rectangle(input, BlackRect, GREENSCALAR, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Black Contour ", "%7d,%7d", BlackRect.x + (BlackRect.width/2), BlackRect.y + (BlackRect.height/2));

            maskBlack.release();
        }


        orangeContourCount = 0;
        yellowContourCount = 0;
        greenContourCount = 0;
        blueContourCount = 0;
        redContourCount = 0;
        pinkContourCount = 0;
        whiteContourCount = 0;
        blackContourCount = 0;

        YCrCb.release();

        //TODO: move this when actually using code (this is just for EasyOpenCV sim)
        //telemetry.update();

        return input;
    }
}
