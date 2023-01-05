package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Test Road Runner Right Auto Sequence", group = "Other")
public class TestRoadRunnerAutoSequence extends LinearOpMode {

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

    private Trajectory initialDrive1, initialDrive2, turnToTallPole, turnToConeStack, driveToConeStack1, driveToConeStack2, driveToTallPole1, driveToTallPole2,turnToStartingWall, parking1, parking2, parking3;

    private TrajectorySequence initialDrive, getCone, driveToTallPole;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, strafeEncoder, leftLift, rightLift, arm, liftMotor;

    private final String setLiftMotor = "leftLift";

    private Servo rotate, claw;

    private boolean liftInMotion;

    private int conesInStack = 5;


    private final double ROTATE_UPSIDE = RobotConstants.ROTATE_UPSIDE, ROTATE_DOWNSIDE = RobotConstants.ROTATE_DOWNSIDE, CLAW_OPEN = RobotConstants.CLAW_OPEN, CLAW_CLOSE = RobotConstants.CLAW_CLOSE, LIFT_ERROR = RobotConstants.LIFT_ERROR;

    private final int TALL = RobotConstants.TALL, MEDIUM = RobotConstants.MEDIUM, LOW = RobotConstants.LOW, CONE_STACK = RobotConstants.CONE_STACK, ARM_FLIPPED = RobotConstants.ARM_FLIPPED, ARM_SHORT = RobotConstants.ARM_SHORT, LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY, ARM_VELOCITY = RobotConstants.ARM_VELOCITY, LIFT_MAXIMUM = RobotConstants.LIFT_MAXIMUM, LIFT_MINIMUM = RobotConstants.LIFT_MINIMUM;

    private Pose2d tallPolePose = new Pose2d(-2.5, 54, Math.toRadians(-45));

    public void autonomous() {
        leftLift.setPower(1); // lifts lift slightly to grab cone better
        rightLift.setPower(1);
        sleep(100);
        leftLift.setPower(0);
        rightLift.setPower(0);
        claw.setPosition(CLAW_CLOSE); // grabs cone
        sleep(300);
        drive.followTrajectorySequence(initialDrive);
        for (int counter = 0; counter < 1; counter++) {
            sleep(500);
            claw.setPosition(CLAW_OPEN); // releases cone
            sleep(500);
            drive.followTrajectorySequence(getCone);
            claw.setPosition(CLAW_CLOSE); // grab cone
            sleep(300);
            liftToPositionAndFlip(getConeStackHeight()+800, 0, ROTATE_UPSIDE); // lifts from stack
            conesInStack--;
            drive.followTrajectorySequence(driveToTallPole);
        }
        sleep(500);
        claw.setPosition(CLAW_OPEN); // releases cone
        sleep(500);
        liftToPositionAndFlip(0, 100, ROTATE_UPSIDE, turnToStartingWall); // returns lift to lowered position and turns to initial heading
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
    }

    public void initialDrive() {
        liftToPositionAndFlip(400, 0, ROTATE_UPSIDE, initialDrive1); //lifts lift a bit to avoid ground junction
        drive.followTrajectory(initialDrive2);
        liftToPositionAndFlip(TALL, ARM_FLIPPED-30, ROTATE_DOWNSIDE, turnToTallPole); // flips up and drives to tall pole
    }

    public void driveToConeStack() {
        liftToPositionAndFlip(getConeStackHeight(), 0, ROTATE_UPSIDE, driveToConeStack1);
        drive.followTrajectory(driveToConeStack2);
    }

    public void liftToPositionAndFlip(int liftPosition, int armPosition, double rotatePosition) {
        liftInMotion = true;
        int liftVelocity = LIFT_VELOCITY;
        int armVelocity = ARM_VELOCITY;
        long startTime = System.currentTimeMillis();
        long timeOut = 2500;
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(liftVelocity);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVelocity);
        rotate.setPosition(rotatePosition);
        while(arm.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
        }
        while(leftLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
        }
        while(rightLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
        }
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
    }

    public void liftToPositionAndFlip(int liftPosition, int armPosition, double rotatePosition, Trajectory trajectory) {
        liftInMotion = true;
        int liftVelocity = LIFT_VELOCITY;
        int armVelocity = ARM_VELOCITY;
        long startTime = System.currentTimeMillis();
        long timeOut = 2500;
        drive.followTrajectoryAsync(trajectory);
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(liftVelocity);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVelocity);
        rotate.setPosition(rotatePosition);
        while(arm.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            drive.update();
        }
        while(leftLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            drive.update();
        }
        while(rightLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            drive.update();
        }
        while (drive.isBusy()) {
            drive.update();
        }
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
    }

    public void liftToPositionAndFlip(int liftPosition, int armPosition, double rotatePosition, TrajectorySequence trajectory) {
        liftInMotion = true;
        int liftVelocity = LIFT_VELOCITY;
        int armVelocity = ARM_VELOCITY;
        long startTime = System.currentTimeMillis();
        long timeOut = 2500;
        drive.followTrajectorySequenceAsync(trajectory);
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(liftVelocity);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVelocity);
        rotate.setPosition(rotatePosition);
        while(arm.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            drive.update();
        }
        while(leftLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            drive.update();
        }
        while(rightLift.isBusy()) {
            if (System.currentTimeMillis()-startTime>timeOut) {
                break;
            }
            drive.update();
        }
        while (drive.isBusy()) {
            drive.update();
        }
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
    }

    public void startLift(int liftPosition, int armPosition, double rotatePosition) {
        liftInMotion = true;
        int liftVelocity = LIFT_VELOCITY;
        int armVelocity = ARM_VELOCITY;
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(liftVelocity);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVelocity);
        rotate.setPosition(rotatePosition);
    }

    public void stopLift() {
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
    }

    public int getConeStackHeight() {
        return CONE_STACK - 180 * (5 - conesInStack);
    }

    public void waitUntilLiftStopped() {
        while (liftInMotion) {}
    }

    public void buildTrajectories() {
        drive = new SampleMecanumDrive(hardwareMap);

        initialDrive = drive.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(400, 0, ROTATE_UPSIDE))
                .lineTo(new Vector2d(-3,20))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift())
                .lineTo(new Vector2d(-2,40))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED-30, ROTATE_DOWNSIDE))
                .lineToSplineHeading(tallPolePose)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift())
                .build();
        getCone = drive.trajectorySequenceBuilder(initialDrive.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight(), 0, ROTATE_UPSIDE))
                .lineToSplineHeading(new Pose2d(0, 51, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(24.5, 51, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()-> stopLift())
                .build();
        driveToTallPole = drive.trajectorySequenceBuilder(getCone.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED-30, ROTATE_DOWNSIDE))
                .lineToSplineHeading(new Pose2d(8, 51, Math.toRadians(0)))
                .lineToSplineHeading(tallPolePose)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift())
                .build();
        initialDrive1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-3,20))
                .build();
        initialDrive2 = drive.trajectoryBuilder(initialDrive1.end())
                .lineTo(new Vector2d(-2,40))
                .build();
        turnToTallPole = drive.trajectoryBuilder(initialDrive2.end())
                .lineToSplineHeading(tallPolePose)
                .build();
        turnToConeStack = drive.trajectoryBuilder(turnToTallPole.end())
                .lineToSplineHeading(new Pose2d(0, 51, Math.toRadians(0)))
                .build();
        driveToConeStack1 = drive.trajectoryBuilder(turnToConeStack.end())
                .lineToSplineHeading(new Pose2d(20, 51, Math.toRadians(0)))
                .build();
        driveToConeStack2 = drive.trajectoryBuilder(driveToConeStack1.end())
                .lineToSplineHeading(new Pose2d(24.5, 51, Math.toRadians(0)))
                .build();
        driveToTallPole1 = drive.trajectoryBuilder(driveToConeStack2.end())
                .lineToSplineHeading(new Pose2d(8, 51, Math.toRadians(0)))
                //.splineToLinearHeading(tallPolePose, tallPolePose.getHeading())
                //.splineTo(vectorFromPose(tallPolePose), tallPolePose.getHeading()+Math.PI)
                .build();
        driveToTallPole2 = drive.trajectoryBuilder(driveToTallPole1.end())
                .lineToSplineHeading(tallPolePose)
                .build();
        /*
        turnToStartingWall = drive.trajectoryBuilder(driveToTallPole2.end())
                .lineToSplineHeading(new Pose2d(0, 50, Math.toRadians(-90)))
                .build();
         */
        turnToStartingWall = drive.trajectoryBuilder(driveToTallPole.end())
                .lineToSplineHeading(new Pose2d(0, 50, Math.toRadians(-90)))
                .build();
        parking1 = drive.trajectoryBuilder(turnToStartingWall.end())
                .lineToSplineHeading(new Pose2d(-20, 50, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(-20,30), Math.toRadians(-90))
                .build();
        parking2 = drive.trajectoryBuilder(turnToStartingWall.end())
                .lineToSplineHeading(new Pose2d(0,30, Math.toRadians(-90)))
                .build();
        parking3 = drive.trajectoryBuilder(turnToStartingWall.end())
                .lineToSplineHeading(new Pose2d(20, 50, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(20,30), Math.toRadians(-90))
                .build();
    }

    public Vector2d vectorFromPose(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
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
        /*
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        */
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

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "leftCamera"), cameraMonitorViewId);
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
