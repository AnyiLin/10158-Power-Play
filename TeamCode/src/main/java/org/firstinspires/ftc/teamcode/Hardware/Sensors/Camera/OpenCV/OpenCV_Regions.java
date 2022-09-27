package org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class OpenCV_Regions extends LinearOpMode {

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

            // Telemetry readings for the HSV values in each region
            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_1[0], pipeline.HSV_Value_1[1], pipeline.HSV_Value_1[2]);
            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
            telemetry.addData("Region 3", "%7d, %7d, %7d", pipeline.HSV_Value_3[0], pipeline.HSV_Value_3[1], pipeline.HSV_Value_3[2]);

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

        // Create a Mat object that will hold the color data
        Mat HSV = new Mat();

        // Make a Constructor
        public OpenCV_Pipeline() {

        }

        // Define the dimensions and location of each region
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1, 70);
        static final int REGION1_WIDTH = 105;
        static final int REGION1_HEIGHT = 105;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(107,70);
        static final int REGION2_WIDTH = 105;
        static final int REGION2_HEIGHT = 105;
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(213,70);
        static final int REGION3_WIDTH = 105;
        static final int REGION3_HEIGHT = 105;

        // Create the points that will be used to make the rectangles for the region
        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

        Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        // Creates a field of type "Mat"
        Mat region1, region2, region3;

        // Creating an array for each region which have an element for each channel of interest
        public int[] HSV_Value_1 = new int[3];
        public int[] HSV_Value_2 = new int[3];
        public int[] HSV_Value_3 = new int[3];

        @Override
        public Mat processFrame(Mat input) {

            // Converts the RGB colors from the video to HSV, which is more useful for image analysis
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2YCrCb);

            // Creates the regions and finds the HSV values for each of the regions
            region1 = HSV.submat(new Rect(region1_pointA, region1_pointB));
            region2 = HSV.submat(new Rect(region2_pointA, region2_pointB));
            region3 = HSV.submat(new Rect(region3_pointA, region3_pointB));

            // Loops through each channel of interest
            for (int i = 0; i < 3; i++){
                // Finds the average HSV value for each channel of interest (The "i" representing the channel of interest)
                HSV_Value_1[i] = (int) Core.mean(region1).val[i];
                HSV_Value_2[i] = (int) Core.mean(region2).val[i];
                HSV_Value_3[i] = (int) Core.mean(region3).val[i];
            }

            // Draws rectangles representing the regions in the camera stream
            Imgproc.rectangle(HSV, region1_pointA, region1_pointB, GOLD,1);
            Imgproc.rectangle(HSV, region2_pointA, region2_pointB, CRIMSON,1);
            Imgproc.rectangle(HSV, region3_pointA, region3_pointB, GOLD,1);

            return HSV;
        }
    }

    public double colorDist3D(double hueIdeal, double saturationIdeal, double valueIdeal, double hueActual, double saturationActual, double valueActual) {

        double dist = Math.sqrt(Math.pow(hueActual - hueIdeal, 2) + Math.pow(saturationActual - saturationIdeal, 2) + Math.pow(valueActual - valueIdeal, 2));

        return dist;
    }
}
