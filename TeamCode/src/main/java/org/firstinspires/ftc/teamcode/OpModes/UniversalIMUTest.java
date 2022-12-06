package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.HeadingAdjustmentUniversalIMU;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Universal IMU Test", group = "Other")
public class UniversalIMUTest extends LinearOpMode {

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private double fx = 578.272, fy = 578.272, cx = 402.145, cy = 221.506;

    // UNITS ARE METERS
    private double tagsize = 0.039;

    private int ID_TAG_OF_INTEREST = 1; // Tag ID [variable] from the 36h11 family
    private int positionToGo;

    private AprilTagDetection tagOfInterest = null;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, strafeEncoder, leftLift, rightLift, arm, liftMotor;

    private final String setLiftMotor = "leftLift";

    private Servo rotate, claw;

    private boolean liftInMotion;

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0.65, CLAW_CLOSE = 0;

    private final int TALL = 3100, MEDIUM = 250, LOW = 3100, CONE_STACK = 1700,
            ARM_FLIPPED = 1000, ARM_SHORT = 150;

    private final double FORWARD = 90, BACKWARD = 267, RIGHT = 0, LEFT = 180;

    private HeadingAdjustmentUniversalIMU tallPoleHeading, originalHeading, coneStackHeading;

    public void autonomous() {
        originalHeading.setHeadingGoal(); // sets the heading goal to the original heading so we can return to it later
        tallPoleHeading.setHeadingGoal(-135); // sets the heading goal to the tall pole
        coneStackHeading.setHeadingGoal(-90); // sets heading goal to the cone stack

        while (!isStopRequested()) {
            zeroDrive();
            telemetry.addData("heading", originalHeading.headingDegrees());
            telemetry.addData("time", System.currentTimeMillis());
            telemetry.update();
        }

    }

    /**
     * @param angle remember this is in degrees!`
     */
    public void strafe(double angle, double power, long millis) {
        angle = Math.toRadians(angle);
        double y =  -Math.sin(angle); // Remember, this is reversed!
        double x = -Math.cos(angle) * 1; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double leftFrontPower = (y + x) / denominator;
        double leftRearPower = (y - x) / denominator;
        double rightFrontPower = (y - x) / denominator;
        double rightRearPower = (y + x) / denominator;

        leftFront.setPower(leftFrontPower * power);
        leftRear.setPower(leftRearPower * power);
        rightFront.setPower(rightFrontPower * power);
        rightRear.setPower(rightRearPower * power);

        sleep(millis);

        zeroDrive();
    }

    public void liftToPositionAndFlip(int liftPosition, int armPosition, double rotatePosition) {
        liftInMotion = true;
        int liftVelocity = 1440*2;
        int armVelocity = (int)(1440 * 0.65);
        long startTime = System.currentTimeMillis();
        long timeOut = 2500;
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVelocity);
        rotate.setPosition(rotatePosition);
        while(arm.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            if (!leftLift.isBusy()) {
                leftLift.setPower(0);
            }
        }
        arm.setPower(0);
        while(leftLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
        }
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
    }

    public void zeroDrive() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void waitUntilLiftStopped() {
        while (liftInMotion) {}
    }

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "strafeEncoder");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (setLiftMotor) {
            case "leftLift":
                liftMotor = leftLift;
                break;
            case "rightLift":
                liftMotor = rightLift;
                break;
        }

        rotate = hardwareMap.get(Servo.class, "rotate");
        claw = hardwareMap.get(Servo.class, "claw");
        rotate.setPosition(ROTATE_UPSIDE);
        claw.setPosition(CLAW_OPEN);

        tallPoleHeading = new HeadingAdjustmentUniversalIMU(hardwareMap);
        tallPoleHeading.setTelemetry(telemetry);
        originalHeading = new HeadingAdjustmentUniversalIMU(hardwareMap);
        originalHeading.setTelemetry(telemetry);
        coneStackHeading = new HeadingAdjustmentUniversalIMU(hardwareMap);
        coneStackHeading.setTelemetry(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                onOpened();
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            detectTag(0);
            detectTag(1);
            detectTag(2);
        }

        camera.stopStreaming();

        parkingPosition();

        if (!isStopRequested()) autonomous();
    }

    public void parkingPosition() {
        if(tagOfInterest == null) {
            /*
             * tag was never sighted during INIT
             */
            positionToGo = 2;

        } else {
            /*
             * tag sighted
             */
            switch (tagOfInterest.id) {
                case 0:
                    positionToGo = 2;
                    break;
                case 1:
                    positionToGo = 1;
                    break;
                case 2:
                    positionToGo = 3;
                    break;
            }
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void detectTag(int tagID) {
        ID_TAG_OF_INTEREST = tagID;
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == ID_TAG_OF_INTEREST) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);
    }

    private float useless = (float)2.7777777777777777777;
}

/**
 * 8==D
 */
