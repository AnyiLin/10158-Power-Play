package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.HeadingAdjustment;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "IMU Right Tall Pole Then Park", group = "Autonomous")
public class TallPoleParkRightIMU extends LinearOpMode {

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

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0.65, CLAW_CLOSE = 0;

    private final int TALL = 3100, MEDIUM = 250, LOW = 3100, CONE_STACK = 1500,
            ARM_FLIPPED = 1000, ARM_SHORT = 150;

    private final double FORWARD = 90, BACKWARD = 267, RIGHT = 0, LEFT = 180;

    private HeadingAdjustment tallPoleHeading, originalHeading, coneStackHeading;

    public void autonomous() {
        originalHeading.setHeadingGoal(); // sets the heading goal to the original heading so we can return to it later
        tallPoleHeading.setHeadingGoal(-45); // sets the heading goal to the tall pole
        coneStackHeading.setHeadingGoal(-90); // sets heading goal to the cone stack

        leftLift.setPower(1); // lifts lift slightly to grab cone better
        sleep(200);
        leftLift.setPower(0);
        claw.setPosition(CLAW_CLOSE); // grabs cone
        sleep(500);
        arm.setPower(0.5); // raises arm slightly to avoid running it over
        sleep(50);
        arm.setPower(0);
        strafe(BACKWARD, 0.8, 2750); // runs forward about 2 tiles
        tallPoleHeading.correctError(1,2000); // turns to pole

        liftToPositionAndFlip(TALL, ARM_FLIPPED-200, ROTATE_DOWNSIDE); // flips up
        strafe(BACKWARD,0.3,100); // goes back to pole
        arm.setPower(0.3); // lowers arm on pole
        sleep(400);
        arm.setPower(0);
        claw.setPosition(CLAW_OPEN); // releases cone
        arm.setPower(-0.4);
        strafe(FORWARD,0.3,100); // goes back to center of tile
        arm.setPower(0);
        claw.setPosition(CLAW_CLOSE); // closes claw to avoid any wire issues
        liftToPositionAndFlip(50, 50, ROTATE_UPSIDE); // returns lift to lowered position

        strafe(FORWARD,0.4,300); // goes forward a bit to adjust for cone stack
        coneStackHeading.correctError(1,3000); // face towards cone stack
        claw.setPosition(CLAW_OPEN); // open claw
        strafe(FORWARD,0.7,1200); // move forward to stack
        liftToPositionAndFlip(CONE_STACK,50,ROTATE_UPSIDE); // lift lift to cone stack height
        strafe(FORWARD,0.3,500); // slowly run to cone stack
        claw.setPosition(CLAW_CLOSE); // grab cone
        sleep(500);
        liftToPositionAndFlip(CONE_STACK+1000,50,ROTATE_UPSIDE); // lift cone from stack
        strafe(BACKWARD,0.6,1800); // move backward to center tile
        tallPoleHeading.correctError(1,3000); // turns to pole
        strafe(BACKWARD,0.4,300);

        liftToPositionAndFlip(TALL, ARM_FLIPPED-200, ROTATE_DOWNSIDE); // flips up
        strafe(BACKWARD,0.3,100); // goes back to pole
        arm.setPower(0.3); // lowers arm on pole
        sleep(400);
        arm.setPower(0);
        claw.setPosition(CLAW_OPEN); // releases cone
        arm.setPower(-0.4);
        strafe(FORWARD,0.3,250); // goes back to center of tile
        arm.setPower(0);
        claw.setPosition(CLAW_CLOSE); // closes claw to avoid any wire issues
        liftToPositionAndFlip(50, 50, ROTATE_UPSIDE); // returns lift to lowered position

        originalHeading.correctError(1,2500); // goes back to original heading
        claw.setPosition(CLAW_OPEN); // opens claw again to avoid hitting any junctions
        sleep(500);
        switch(positionToGo) // determine where to go
        {
            case 1:
                //run right to the left space
                strafe(RIGHT, 0.8,1600);
                break;
            case 2:
                //stay in place
                break;
            case 3:
                //run left to the right space
                strafe(LEFT,0.8,1600);
                break;
        }
        strafe(FORWARD, 0.8,1000); // move forward slightly to be completely in the space
        arm.setPower(-0.5); // puts arm at zero position for driver mode
        sleep(300);
        arm.setPower(0);

    }

    /**
     * @param angle remember this is in degrees!
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
    }

    public void zeroDrive() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
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

        tallPoleHeading = new HeadingAdjustment(hardwareMap);
        tallPoleHeading.setTelemetry(telemetry);
        originalHeading = new HeadingAdjustment(hardwareMap);
        originalHeading.setTelemetry(telemetry);
        coneStackHeading = new HeadingAdjustment(hardwareMap);
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

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            detectTag(0);
            detectTag(1);
            detectTag(2);
        }

        parkingPosition();

        autonomous();
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
}

/**
 * 8==D
 */