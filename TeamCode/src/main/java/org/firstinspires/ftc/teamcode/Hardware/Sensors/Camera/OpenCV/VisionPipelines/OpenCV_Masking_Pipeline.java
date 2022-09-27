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

public class OpenCV_Masking_Pipeline extends OpenCvPipeline {

    // Creating a Telemetry object to allow us to use the "Telemetry" class without extending to LinearOpMode
    Telemetry telemetry;

    /** Most important section of the code: Colors **/
    // These are just color constants used to define the color for each bounding box we create for the contours
//    static final Scalar BLAZE =  new Scalar(255, 102, 0);
//    static final Scalar GOLD = new Scalar(255, 215, 0);
//    static final Scalar PARAKEET = new Scalar(3, 192, 74);
//    static final Scalar CYAN = new Scalar(0, 139, 139);
//    static final Scalar CRIMSON = new Scalar(220, 20, 60);
//    static final Scalar HOT_PINK = new Scalar(196, 23, 112);
//    static final Scalar LIGHT_GRAY = new Scalar(255, 255, 240);
//    static final Scalar DARK_GRAY = new Scalar(120,120,120);

    static final Scalar BLAZE =  new Scalar(255, 255, 240);
    static final Scalar GOLD = new Scalar(100, 100, 200);
    static final Scalar PARAKEET = new Scalar(220, 20, 60);
    static final Scalar CYAN = new Scalar(100, 200, 112);
    static final Scalar CRIMSON = new Scalar(120,120,120);
    static final Scalar HOT_PINK = new Scalar(3, 200, 255);
    static final Scalar LIGHT_GRAY = new Scalar(255, 215, 0);
    static final Scalar DARK_GRAY = new Scalar(255, 102, 0);

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

    // Creating lists for all the MatOfPoints of each contour
    public List<MatOfPoint> redContours;
    public List<MatOfPoint> orangeContours;
    public List<MatOfPoint> yellowContours;
    public List<MatOfPoint> greenContours;
    public List<MatOfPoint> blueContours;
    public List<MatOfPoint> pinkContours;
    public List<MatOfPoint> whiteContours;
    public List<MatOfPoint> blackContours;

    // Make a Constructor
    public OpenCV_Masking_Pipeline(Telemetry telemetry) {

        // Creates an arraylist for each contour list
        redContours = new ArrayList<MatOfPoint>();
        orangeContours = new ArrayList<MatOfPoint>();
        yellowContours = new ArrayList<MatOfPoint>();
        greenContours = new ArrayList<MatOfPoint>();
        blueContours = new ArrayList<MatOfPoint>();
        pinkContours = new ArrayList<MatOfPoint>();
        whiteContours = new ArrayList<MatOfPoint>();
        blackContours = new ArrayList<MatOfPoint>();

        // Creates an arraylist for bounding Rect list
        orangeRect = new ArrayList<Rect>();
        yellowRect = new ArrayList<Rect>();
        greenRect = new ArrayList<Rect>();
        blueRect = new ArrayList<Rect>();
        redRect = new ArrayList<Rect>();
        pinkRect = new ArrayList<Rect>();
        whiteRect = new ArrayList<Rect>();
        blackRect = new ArrayList<Rect>();

        this.telemetry = telemetry;
    }

    public List<Rect> getYellowRect(){

        return yellowRect;
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 10;
    }

    /** Note: These thresholding values are approximates and you should try to finetune them to your specific necessities **/

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

    // Mat object for HSV color space
    Mat HSV = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size((2*2) + 1, (2*2)+1));

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

//        Imgproc.morphologyEx(HSV, HSV, Imgproc.MORPH_OPEN, new Mat());
//        Imgproc.morphologyEx(HSV, HSV, Imgproc.MORPH_CLOSE, new Mat());

//        Imgproc.GaussianBlur(HSV, HSV, kSize,0);
//        Imgproc.blur(HSV, HSV, kSize);
        Imgproc.erode(HSV, HSV, kernel);

        if (ORANGE){
            inRange(HSV, lowOrange, highOrange, maskOrange);
            orangeContours.clear();
            orangeRect.clear();

            Imgproc.findContours(maskOrange, orangeContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, orangeContours, -1, BLAZE); //input

            for (int i = 0; i < orangeContours.size(); i++){
                if (filterContours(orangeContours.get(i))){
                    orangeRect.add(Imgproc.boundingRect(orangeContours.get(i)));
                    Imgproc.rectangle(input, orangeRect.get(orangeContourCount), BLAZE, 2);
                    orangeContourCount++;
                }
            }

            telemetry.addData("Orange Contour Count", orangeContourCount);

            for (int i = 0; i < orangeRect.size(); i++){
                telemetry.addData("Orange Contour " + (i+1), "%7d,%7d", orangeRect.get(i).x + (orangeRect.get(i).width/2), orangeRect.get(i).y + (orangeRect.get(i).height/2));
            }

            maskOrange.release();
        }

        if (YELLOW){
            // Finds the pixels within the thresholds and puts them in the mat object "maskYellow"
            inRange(HSV, lowYellow, highYellow, maskYellow);

            // Clears the arraylists
            yellowContours.clear();
            yellowRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskYellow, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, yellowContours, -1, GOLD); //input

            // Iterates through each contour
            for (int i = 0; i < yellowContours.size(); i++){

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(yellowContours.get(i))){
                    // Creates a bounding rect around each contourand the draws it
                    yellowRect.add(Imgproc.boundingRect(yellowContours.get(i)));
                    Imgproc.rectangle(input, yellowRect.get(yellowContourCount), GOLD, 2);

                    // Creates a count for the amount of yellow contours on the the screen
                    yellowContourCount++;
                }
            }

            telemetry.addData("Yellow Contour Count", yellowContourCount);

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            for (int i = 0; i < yellowRect.size(); i++){
                telemetry.addData("Yellow Contour " + (i+1), "%7d,%7d", yellowRect.get(i).x + (yellowRect.get(i).width/2), yellowRect.get(i).y + (yellowRect.get(i).height/2));
            }

            maskYellow.release();
        }

        if (GREEN){
            inRange(HSV, lowGreen, highGreen, maskGreen);

            greenContours.clear();
            greenRect.clear();

            Imgproc.findContours(maskGreen, greenContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, greenContours, -1, PARAKEET); //input

            for (int i = 0; i < greenContours.size(); i++){
                if (filterContours(greenContours.get(i))){
                    greenRect.add(Imgproc.boundingRect(greenContours.get(i)));
                    Imgproc.rectangle(input, greenRect.get(greenContourCount), PARAKEET, 2);
                    greenContourCount++;
                }
            }

            telemetry.addData("Green Contour Count", greenContourCount);

            for (int i = 0; i < greenRect.size(); i++){
                telemetry.addData("Green Contour " + (i+1), "%7d,%7d", greenRect.get(i).x + (greenRect.get(i).width/2), greenRect.get(i).y + (greenRect.get(i).height/2));
            }

            maskGreen.release();
        }

        if (BLUE){
            inRange(HSV, lowBlue, highBlue, maskBlue);

            Imgproc.morphologyEx(maskBlue, maskBlue, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(maskBlue, maskBlue, Imgproc.MORPH_CLOSE, new Mat());

            Imgproc.GaussianBlur(maskBlue, maskBlue, kSize, 0);

            blueContours.clear();
            blueRect.clear();

            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blueContours, -1, CYAN); //input

            for (int i = 0; i < blueContours.size(); i++){
                if (filterContours(blueContours.get(i))){
                    blueRect.add(Imgproc.boundingRect(blueContours.get(i)));
                    Imgproc.rectangle(input, blueRect.get(blueContourCount), CYAN, 2);
                    blueContourCount++;
                }
            }

            telemetry.addData("Blue Contour Count", blueContourCount);

            for (int i = 0; i < blueRect.size(); i++){
                telemetry.addData("Blue Contour " + (i+1), "%7d,%7d", blueRect.get(i).x + (blueRect.get(i).width/2), blueRect.get(i).y + (blueRect.get(i).height/2));
            }

//            maskBlue.release();
        }

        if (RED){
            inRange(HSV, lowRed, highRed, maskRed);
            redContours.clear();
            redRect.clear();

            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, CRIMSON); //input

            for (int i = 0; i < redContours.size(); i++){
                if (filterContours(redContours.get(i))){
                    redRect.add(Imgproc.boundingRect(redContours.get(i)));
                    Imgproc.rectangle(input, redRect.get(redContourCount), CRIMSON, 2);
                    redContourCount++;
                }
            }

            telemetry.addData("Red Contour Count", redContourCount);

            for (int i = 0; i < redRect.size(); i++){
                telemetry.addData("Red Contour " + (i+1), "%7d,%7d", redRect.get(i).x + (redRect.get(i).width/2), redRect.get(i).y + (redRect.get(i).height/2));
            }

            maskRed.release();
        }


        if (PINK){
            inRange(HSV, lowPink, highPink, maskPink);

            pinkContours.clear();
            pinkRect.clear();

            Imgproc.findContours(maskPink, pinkContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, pinkContours, -1, HOT_PINK); //input

            for (int i = 0; i < pinkContours.size(); i++){
                if (filterContours(pinkContours.get(i))){
                    pinkRect.add(Imgproc.boundingRect(pinkContours.get(i)));
                    Imgproc.rectangle(input, pinkRect.get(pinkContourCount), HOT_PINK, 2);
                    pinkContourCount++;
                }
            }

            telemetry.addData("Pink Contour Count", pinkContourCount);

            for (int i = 0; i < pinkRect.size(); i++){
                telemetry.addData("Pink Contour " + (i+1), "%7d,%7d", pinkRect.get(i).x + (pinkRect.get(i).width/2), pinkRect.get(i).y + (pinkRect.get(i).height/2));
            }

            maskPink.release();
        }

        if (WHITE){
            inRange(HSV, lowWhite, highWhite, maskWhite);

            Imgproc.morphologyEx(maskWhite, maskWhite, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(maskWhite, maskWhite, Imgproc.MORPH_CLOSE, new Mat());

            Imgproc.blur(maskWhite, maskWhite, kSize);

            whiteContours.clear();
            whiteRect.clear();

            Imgproc.findContours(maskWhite, whiteContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, whiteContours, -1, LIGHT_GRAY); //input

            for (int i = 0; i < whiteContours.size(); i++){
                if (filterContours(whiteContours.get(i))){
                    whiteRect.add(Imgproc.boundingRect(whiteContours.get(i)));
                    Imgproc.rectangle(input, whiteRect.get(whiteContourCount), LIGHT_GRAY, 2);
                    whiteContourCount++;
                }
            }

            telemetry.addData("White Contour Count", whiteContourCount);

            for (int i = 0; i < whiteRect.size(); i++){
                telemetry.addData("White Contour " + (i+1), "%7d,%7d", whiteRect.get(i).x + (whiteRect.get(i).width/2), whiteRect.get(i).y + (whiteRect.get(i).height/2));
            }

            maskWhite.release();
        }

        if (BLACK){
            inRange(HSV, lowBlack, highBlack, maskBlack);
            blackContours.clear();
            blackRect.clear();

            Imgproc.findContours(maskBlack, blackContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blackContours, -1, DARK_GRAY); //input

            for (int i = 0; i < blackContours.size(); i++){
                if (filterContours(blackContours.get(i))){
                    blackRect.add(Imgproc.boundingRect(blackContours.get(i)));
                    Imgproc.rectangle(input, blackRect.get(blackContourCount), DARK_GRAY, 2);
                    blackContourCount++;
                }
            }

            telemetry.addData("Black Contour Count", blackContourCount);

            for (int i = 0; i < blackRect.size(); i++){
                telemetry.addData("Black Contour " + (i+1), "%7d,%7d", blackRect.get(i).x + (blackRect.get(i).width/2), blackRect.get(i).y + (blackRect.get(i).height/2));
            }

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

        HSV.release();

        // TODO: Remove
        telemetry.update();
        return input;
    }
}
