package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.HeadingAdjustmentUniversalIMU;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Test Road Runner Right Auto", group = "Other")
public class TestRoadRunnerAuto extends LinearOpMode {

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

    private SampleMecanumDrive drive;

    private Trajectory initialDrive, turnToTallPole, turnToConeStack, driveToConeStack, driveToTallPole, turnToInitialHeading, parking1, parking2, parking3;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, strafeEncoder, leftLift, rightLift, arm, liftMotor;

    private final String setLiftMotor = "leftLift";

    private Servo rotate, claw;

    private boolean liftInMotion;

    private int conesInStack = 5;

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0.65, CLAW_CLOSE = 0;

    private final int TALL = 3100, MEDIUM = 250, LOW = 3100, CONE_STACK = 1700,
            ARM_FLIPPED = 1000, ARM_SHORT = 150;

    public void autonomous() {
        leftLift.setPower(1); // lifts lift slightly to grab cone better
        sleep(200);
        leftLift.setPower(0);
        claw.setPosition(CLAW_CLOSE); // grabs cone
        sleep(300);
        drive.followTrajectory(initialDrive); // drives to the tall pole
        new Thread(new Runnable() { public void run() {
            liftToPositionAndFlip(TALL, ARM_FLIPPED-200, ROTATE_DOWNSIDE); // flips up in a new thread
        }}).start();
        for (int counter = 0; counter < 3; counter++) {
            drive.followTrajectory(turnToTallPole); // turns to the tall pole
            waitUntilLiftStopped();
            arm.setPower(0.3); // lowers arm on pole
            sleep(500);
            arm.setPower(0);
            sleep(500);
            claw.setPosition(CLAW_OPEN); // releases cone
            arm.setPower(-0.4); // lifts arm off pole
            sleep(500);
            arm.setPower(0);
            claw.setPosition(CLAW_CLOSE); // closes claw to avoid any wire issues
            new Thread(new Runnable() {
                public void run() {
                    liftToPositionAndFlip(getConeStackHeight(), 50, ROTATE_UPSIDE); // lift lift to cone stack height in a new thread
                }
            }).start();
            drive.followTrajectory(turnToConeStack); // turns to cone stack
            drive.followTrajectory(driveToConeStack); // drives to cone stack
            waitUntilLiftStopped();
            claw.setPosition(CLAW_CLOSE); // grab cone
            sleep(300);
            leftLift.setPower(1); // lifts cone from stack
            sleep(800);
            leftLift.setPower(0);
            conesInStack--;
            new Thread(new Runnable() {
                public void run() {
                    liftToPositionAndFlip(TALL, ARM_FLIPPED - 200, ROTATE_DOWNSIDE); // flips up in a new thread
                }
            }).start();
            drive.followTrajectory(driveToTallPole); // drives to tall pole
        }
        drive.followTrajectory(turnToTallPole); // turns to the tall pole
        waitUntilLiftStopped();
        arm.setPower(0.3); // lowers arm on pole
        sleep(500);
        arm.setPower(0);
        sleep(500);
        claw.setPosition(CLAW_OPEN); // releases cone
        arm.setPower(-0.4); // lifts arm off pole
        sleep(500);
        arm.setPower(0);
        claw.setPosition(CLAW_CLOSE); // closes claw to avoid any wire issues
        new Thread(new Runnable() {
            public void run() {
                liftToPositionAndFlip(0, 50, ROTATE_UPSIDE); // returns lift to lowered position in a new thread
            }
        }).start();
        drive.followTrajectory(turnToInitialHeading); // turns back to initial heading
        waitUntilLiftStopped();
        claw.setPosition(CLAW_OPEN); // opens claw to avoid hitting any junctions
        sleep(300);
        switch (positionToGo) {
            case 1:
                drive.followTrajectory(parking1);
                break;
            case 2:
                drive.followTrajectory(parking2);
                break;
            case 3:
                drive.followTrajectory(parking3);
                break;
        }
        arm.setPower(-0.5); // puts arm at zero position for driver mode
        sleep(300);
        arm.setPower(0);
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

    public int getConeStackHeight() {
        return CONE_STACK - 275 * (5 - conesInStack);
    }

    public void waitUntilLiftStopped() {
        while (liftInMotion) {}
    }

    public void buildTrajectories() {
        drive = new SampleMecanumDrive(hardwareMap);

        /**
         * maybe use spline paths instead of straight trajectories?
         *
         * strafe forward 60 inches
         * turn to the left Math.toRadians(45); (to the tall pole)
         * score cone, perhaps have the lift asynchronously as we turn
         * turn to the left Math.toRadians(45); (to the cone stack)
         * have the lift go down to cone stack height, perhaps as we turn
         * strafe backward 24 or so inches to the cone stack
         * grab the cone and lift the lift
         * strafe backwards 24 or so inches to the middle tile
         */
        initialDrive = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();
        turnToTallPole = drive.trajectoryBuilder(initialDrive.end())
                .splineToSplineHeading(new Pose2d(60, 0, Math.toRadians(45)), Math.toRadians(45))
                .build();
        turnToConeStack = drive.trajectoryBuilder(turnToTallPole.end())
                .splineToSplineHeading(new Pose2d(60, 0, Math.toRadians(90)), Math.toRadians(90))
                .build();
        driveToConeStack= drive.trajectoryBuilder(turnToConeStack.end())
                .splineToSplineHeading(new Pose2d(60, -24, Math.toRadians(90)), Math.toRadians(90))
                .build();
        driveToTallPole = drive.trajectoryBuilder(driveToConeStack.end())
                .splineToSplineHeading(new Pose2d(60, 0, Math.toRadians(90)), Math.toRadians(90))
                .build();
        turnToInitialHeading = drive.trajectoryBuilder(driveToTallPole.end())
                .splineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();
        parking1 = drive.trajectoryBuilder(turnToInitialHeading.end())
                .splineToSplineHeading(new Pose2d(60, -24, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(44,-24, Math.toRadians(0)), Math.toRadians(0))
                .build();
        parking2 = drive.trajectoryBuilder(turnToInitialHeading.end())
                .splineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(44,0, Math.toRadians(0)), Math.toRadians(0))
                .build();
        parking3 = drive.trajectoryBuilder(turnToInitialHeading.end())
                .splineToSplineHeading(new Pose2d(60, 24, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(44,24, Math.toRadians(0)), Math.toRadians(0))
                .build();
    }

    @Override
    public void runOpMode() {
        buildTrajectories();

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

        if (isStopRequested()) return;

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
