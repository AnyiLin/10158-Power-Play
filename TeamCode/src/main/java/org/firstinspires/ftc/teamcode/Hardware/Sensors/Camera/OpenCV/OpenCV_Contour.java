package org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class OpenCV_Contour extends LinearOpMode {

    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static OpenCV_Pipeline pipeline;

    @Override
    public void runOpMode() {

        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new OpenCV_Pipeline();
        webcam.setPipeline(pipeline);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            telemetry.update();
        }
    }


    public static class OpenCV_Pipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        // Create a Mat object that will hold the color data

        Mat Blur = new Mat();

        Mat Hierarchy = new Mat();
        Mat valueThreshold = new Mat();
        Mat circles = new Mat();

        Rect circleRect;

        public int minValueThreshold;
        public int maxValueThreshold;

        List<MatOfPoint> Contours;
        MatOfPoint biggestContour;

        // Make a Constructor
        public OpenCV_Pipeline() {
            minValueThreshold = 20;
            maxValueThreshold = 150;

            Contours = new ArrayList<MatOfPoint>();
        }

        public double rectCheckBottom(MatOfPoint contour) {
            Rect rectInfo = Imgproc.boundingRect(contour).clone();

            double ratio = rectInfo.width/rectInfo.height;

            return ratio / 11;
        }
        public double rectCheckMiddle(MatOfPoint contour) {
            Rect rectInfo = Imgproc.boundingRect(contour).clone();

            double ratio = rectInfo.width/rectInfo.height;

            return ratio / 8;
        }

        public double shippingHubCheck(MatOfPoint contour){
            Rect rectInfo = Imgproc.boundingRect(contour).clone();
            double contourArea = Imgproc.contourArea(contour);

            double value = rectInfo.area()/contourArea;

            return value/5;
        }

        public double rectCheckTop(MatOfPoint contour) {
            Rect rectInfo = Imgproc.boundingRect(contour).clone();

            double ratio = rectInfo.width/rectInfo.height;

            return ratio / 15;
        }

        public double circleCheck(MatOfPoint contour) {

            Rect circleInfo = Imgproc.boundingRect(contour).clone();

            double radius = circleInfo.width/2;
            double area = Math.PI * (radius*radius);

            double idealArea = circleInfo.area()/1.273239545;

            return area / idealArea;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat gray = new Mat(input.rows(), input.cols(), input.type());

            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            Mat edges = new Mat(input.rows(), input.cols(), input.type());

            Size kSize = new Size(3, 3);

            Imgproc.blur(gray, edges, kSize);
//
            Imgproc.Canny(edges, edges, minValueThreshold, maxValueThreshold);

            Contours.clear();

            Imgproc.findContours(edges, Contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.drawContours(input, Contours, -1, AQUA); //input

//            MatOfPoint2f thisContour2f = new MatOfPoint2f();
//            MatOfPoint approxContour = new MatOfPoint();
//            MatOfPoint2f approxContour2f = new MatOfPoint2f();
//
//            Rect ret = null;

//            edges.convertTo(thisContour2f, CvType.channels(0));
//
//            Imgproc.approxPolyDP(thisContour2f, approxContour2f, 2, true);
//
//            approxContour2f.convertTo(approxContour, CvType.channels(0));

//            if (approxContour.size().height == 4) {
//                Imgproc.boundingRect(approxContour);
//            }

//            long count = approxContour.total();

            for (int i = 0; i < Contours.size(); i++){
                if ((shippingHubCheck(Contours.get(i)) > 0.8) && (shippingHubCheck(Contours.get(i)) < 1.2)){
                    circleRect = Imgproc.boundingRect(Contours.get(i));
                    Imgproc.rectangle(input, circleRect, PARAKEET, 2); // input
                }

//                if (count == 3) {
//                    circleRect = Imgproc.boundingRect(Contours.get(i));
//                    Imgproc.rectangle(input, circleRect, PARAKEET, 2);
//                }
            }

            /*
            Imgproc.HoughCircles(edges, circles, Imgproc.HOUGH_GRADIENT, Math.PI/180, 50, 1, 50);
            for (int i = 0; i < circles.cols(); i++){
                double[] data = circles.get(0,i);
                Point center = new Point(Math.round(data[0]), Math.round(data[1]));
                Imgproc.circle(HSV, center, 1, new Scalar(0,0,255), 2, 8, 0);

            }             */
//
//            Imgproc.HoughCircles(edges, circles, Imgproc.HOUGH_GRADIENT,  1, 20, 75, 40);
//
//            for (int i = 0; i < circles.cols(); i++ ) {
//                double[] data = circles.get(0, i);
//                Point center = new Point(Math.round(data[0]), Math.round(data[1]));
//                // circle center
//                Imgproc.circle(edges, center, 1, CRIMSON, 4, 8, 0 );
//                // circle outline
//                int radius = (int) Math.round(data[2]);
//                Imgproc.circle(edges, center, radius, CRIMSON, 3, 8, 0 );
//            }

            return input;
        }
    }
}
