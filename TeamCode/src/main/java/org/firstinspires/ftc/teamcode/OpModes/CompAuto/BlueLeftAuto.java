package org.firstinspires.ftc.teamcode.OpModes.CompAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpModes.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Blue Left 1+3 Autonomous", group = "Autonomous")
public class BlueLeftAuto extends LinearOpMode {

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

    private TrajectorySequence initialDrive, getConeOne, getConeTwo, getConeThree, parkingOne, parkingTwo, parkingThree;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, strafeEncoder, leftLift, rightLift, arm, liftMotor;

    private final String setLiftMotor = "leftLift";

    private Servo rotate, claw;

    private boolean liftInMotion;

    private int conesInStack = 5;

    private double armVelocityCoefficient = 0.8;

    private final double ROTATE_UPSIDE = RobotConstants.ROTATE_UPSIDE, ROTATE_DOWNSIDE = RobotConstants.ROTATE_DOWNSIDE, CLAW_OPEN = RobotConstants.CLAW_OPEN, CLAW_CLOSE = RobotConstants.CLAW_CLOSE, LIFT_ERROR = RobotConstants.LIFT_ERROR;

    private final int TALL = RobotConstants.TALL, MEDIUM = RobotConstants.MEDIUM, LOW = RobotConstants.LOW, CONE_STACK = RobotConstants.CONE_STACK, CONE_HEIGHT_CHANGE = RobotConstants.CONE_HEIGHT_CHANGE, ARM_FLIPPED = RobotConstants.ARM_FLIPPED, ARM_SHORT = RobotConstants.ARM_SHORT, LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY, ARM_VELOCITY = RobotConstants.ARM_VELOCITY, LIFT_MAXIMUM = RobotConstants.LIFT_MAXIMUM, LIFT_MINIMUM = RobotConstants.LIFT_MINIMUM;

    private Pose2d tallPolePose = new Pose2d(6.25, -52.25, Math.toRadians(49.5));
    private Pose2d tallPolePose2 = new Pose2d(2.25, -51.75, Math.toRadians(42));
    private Pose2d tallPolePose3 = new Pose2d(2.25, -51.5, Math.toRadians(42));
    private Pose2d tallPolePose4 = new Pose2d(2.25, -51.5, Math.toRadians(39));
    private Pose2d coneStack = new Pose2d(29.5, -49, Math.toRadians(0));
    private Pose2d coneStack2 = new Pose2d(29, -47.5, Math.toRadians(0));
    private Pose2d coneStack3 = new Pose2d(29, -47.5, Math.toRadians(0));

    public void autonomous() {
        // this should be pretty self explanatory. For questions on what the trajectory sequences do, see a bit below
        drive.followTrajectorySequence(initialDrive);

        drive.followTrajectorySequence(getConeOne);

        drive.followTrajectorySequence(getConeTwo);

        drive.followTrajectorySequence(getConeThree);

        switch (positionToGo) {
            case 1:
                drive.followTrajectorySequence(parkingOne);
                break;
            case 2:
                drive.followTrajectorySequence(parkingTwo);
                break;
            case 3:
                drive.followTrajectorySequence(parkingThree);
                break;
        }
    }

    public void startLift(int liftPosition, int armPosition, double rotatePosition) {
        liftInMotion = true;
        int liftVelocity = LIFT_VELOCITY;
        int armVelocity = (int)(ARM_VELOCITY*armVelocityCoefficient);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(liftVelocity);
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVelocity);
        rotate.setPosition(rotatePosition);
    }

    public void startLift(int liftPosition, int armPosition, int setArmVelocity, double rotatePosition) {
        liftInMotion = true;
        int liftVelocity = LIFT_VELOCITY;
        int armVelocity = setArmVelocity;
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(liftVelocity);
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(liftVelocity);
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
        return CONE_STACK - CONE_HEIGHT_CHANGE * (5 - conesInStack);
    }

    public void buildTrajectories() {
        drive = new SampleMecanumDrive(hardwareMap);

        initialDrive = drive.trajectorySequenceBuilder(new Pose2d()) // this does the starting drive to the tall pole and scores the cone
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(150, 0, ROTATE_UPSIDE)) // lift arm up to grab cone
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_CLOSE)) // close claw on cone
                .waitSeconds(0.3) // give the claw time to close
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED-100, ROTATE_DOWNSIDE)) // raise lift up and flit arm. gets cone out of the way of the ground junction and poles, and this needs to be done anyways, so easier sooner rather than later
                .lineToSplineHeading(new Pose2d(1,-44, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL+40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL+40))
                .splineToLinearHeading(tallPolePose, tallPolePose.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED+300, ROTATE_DOWNSIDE)) // dunks the cone on the pole
                .waitSeconds(0.2) // gives the arm time to lower
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lift before the trajectory sequence ends. May not be entirely necessary but its here
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_OPEN)) // drops the cone
                .build();
        getConeOne = drive.trajectorySequenceBuilder(initialDrive.end()) // this drives from the tall pole to the cone stack, grabs a cone, and then drives back to the tall pole and scores it
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight(), 0, (int)(ARM_VELOCITY*2), ROTATE_DOWNSIDE)) // starts moving the arm and lift to cone stack height, but not rotating the claw so that it doesn't hit the cone or pole
                .UNSTABLE_addTemporalMarkerOffset(0.3,()-> startLift(getConeStackHeight(), 0, (int)(ARM_VELOCITY*2), ROTATE_UPSIDE)) // starts rotating the claw after a delay, avoiding hitting anything with the claw
                .splineTo(vectorFromPose(coneStack), coneStack.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()-> stopLift()) // half a second before reaching the cone stack, stops the lift so that the motors don't push against the cone stack and cause issues
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_CLOSE)) // grabs a cone
                .waitSeconds(0.3) // gives the claw time to close
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight()+800, 0, ROTATE_UPSIDE)) // lifts the cone off of the stack
                .UNSTABLE_addTemporalMarkerOffset(0,()-> conesInStack--) // decrements the variable keeping track of the number of cones in the stack
                .waitSeconds(0.4) // gives the lift time to lift the cone off of the stack
                .UNSTABLE_addTemporalMarkerOffset(0.5,()-> startLift(TALL, ARM_FLIPPED-100, ROTATE_DOWNSIDE)) // raises the lift and flips the arm after a short delay to avoid hitting the wall with the arm
                .setReversed(true).splineToLinearHeading(tallPolePose2, tallPolePose2.getHeading()+Math.PI).setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED+300, ROTATE_DOWNSIDE)) // dunks the cone on the pole
                .waitSeconds(0.3) // gives the arm time to lower
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lift before the end of the trajectory sequence. Again, probably not strictly necessary
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_OPEN)) // drops the cone on the pole
                .build();
        getConeTwo = drive.trajectorySequenceBuilder(getConeOne.end()) // this drives from the tall pole to the cone stack, grabs a second cone, and then drives back to the tall pole and scores it
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight(), 0, (int)(ARM_VELOCITY*2), ROTATE_DOWNSIDE)) // starts moving the arm and lift to cone stack height, but not rotating the claw so that it doesn't hit the cone or pole
                .UNSTABLE_addTemporalMarkerOffset(0.3,()-> startLift(getConeStackHeight(), 0, (int)(ARM_VELOCITY*2), ROTATE_UPSIDE)) // starts rotating the claw after a delay, avoiding hitting anything with the claw
                .splineTo(vectorFromPose(coneStack2), coneStack2.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()-> stopLift()) // half a second before reaching the cone stack, stops the lift so that the motors don't push against the cone stack and cause issues
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_CLOSE)) // grabs a cone
                .waitSeconds(0.3) // gives the claw time to close
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight()+800, 0, ROTATE_UPSIDE)) // lifts the cone off of the stack
                .UNSTABLE_addTemporalMarkerOffset(0,()-> conesInStack--) // decrements the variable keeping track of the number of cones in the stack
                .waitSeconds(0.4) // gives the lift time to lift the cone off of the stack
                .UNSTABLE_addTemporalMarkerOffset(0.5,()-> startLift(TALL, ARM_FLIPPED-100, ROTATE_DOWNSIDE)) // raises the lift and flips the arm after a short delay to avoid hitting the wall with the arm
                .setReversed(true).splineToLinearHeading(tallPolePose3, tallPolePose3.getHeading()+Math.PI).setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED+300, ROTATE_DOWNSIDE)) // dunks the cone on the pole
                .waitSeconds(0.3) // gives the arm time to lower
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lift before the end of the trajectory sequence. Again, probably not strictly necessary
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_OPEN)) // drops the cone on the pole
                .build();
        getConeThree = drive.trajectorySequenceBuilder(getConeTwo.end()) // this drives from the tall pole to the cone stack, grabs a second cone, and then drives back to the tall pole and scores it
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight(), 0, (int)(ARM_VELOCITY*2), ROTATE_DOWNSIDE)) // starts moving the arm and lift to cone stack height, but not rotating the claw so that it doesn't hit the cone or pole
                .UNSTABLE_addTemporalMarkerOffset(0.3,()-> startLift(getConeStackHeight(), 0, (int)(ARM_VELOCITY*2), ROTATE_UPSIDE)) // starts rotating the claw after a delay, avoiding hitting anything with the claw
                .splineTo(vectorFromPose(coneStack3), coneStack3.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()-> stopLift()) // half a second before reaching the cone stack, stops the lift so that the motors don't push against the cone stack and cause issues
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_CLOSE)) // grabs a cone
                .waitSeconds(0.3) // gives the claw time to close
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(getConeStackHeight()+800, 0, ROTATE_UPSIDE)) // lifts the cone off of the stack
                .UNSTABLE_addTemporalMarkerOffset(0,()-> conesInStack--) // decrements the variable keeping track of the number of cones in the stack
                .waitSeconds(0.4) // gives the lift time to lift the cone off of the stack
                .UNSTABLE_addTemporalMarkerOffset(0.5,()-> startLift(TALL, ARM_FLIPPED-100, ROTATE_DOWNSIDE)) // raises the lift and flips the arm after a short delay to avoid hitting the wall with the arm
                .setReversed(true).splineToLinearHeading(tallPolePose4, tallPolePose4.getHeading()+Math.PI).setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(TALL, ARM_FLIPPED+300, ROTATE_DOWNSIDE)) // dunks the cone on the pole
                .waitSeconds(0.3) // gives the arm time to lower
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lift before the end of the trajectory sequence. Again, probably not strictly necessary
                .UNSTABLE_addTemporalMarkerOffset(0,()-> claw.setPosition(CLAW_OPEN)) // drops the cone on the pole
                .build();
        parkingOne = drive.trajectorySequenceBuilder(getConeThree.end()) // this parks in parking space one from the tall pole and resets the arm and lift
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(10, 400, ROTATE_DOWNSIDE))
                .UNSTABLE_addTemporalMarkerOffset(0.3,()-> startLift(10, 400, ROTATE_UPSIDE))
                .lineToSplineHeading(new Pose2d(0, -50, Math.toRadians(90))) // turns to face the starting wall while moving to the center of the tall pole
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(10, 50, ROTATE_UPSIDE))
                .lineToSplineHeading(new Pose2d(26, -50, Math.toRadians(90))) // moves to the third parking zone
                .splineToConstantHeading(new Vector2d(24.5,-33), Math.toRadians(90)) // runs forward a little bit
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lifts
                .build();
        parkingTwo = drive.trajectorySequenceBuilder(getConeThree.end()) // this parks in parking space two from the tall pole and resets the arm and lift
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(10, 400, ROTATE_DOWNSIDE)) // starts resetting the arm and lift, but not rotating the claw so that it doesn't hit the cone or pole
                .UNSTABLE_addTemporalMarkerOffset(0.3,()-> startLift(10, 400, ROTATE_UPSIDE)) // starts rotating the claw after a delay, avoiding hitting anything with the claw
                .lineToSplineHeading(new Pose2d(1, -50, Math.toRadians(90))) // turns to face the starting wall while moving to the center of the tall pole. We don't need to move after this in this trajectory sequence because we're already in parking zone two
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(10, 50, ROTATE_UPSIDE))
                .splineToConstantHeading(new Vector2d(1,-33), Math.toRadians(90)) // runs forward a little bit
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lifts
                .build();
        parkingThree = drive.trajectorySequenceBuilder(getConeThree.end()) // this parks in parking space three from the tall pole and resets the arm and lift
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(10, 400, ROTATE_DOWNSIDE)) // starts resetting the arm and lift, but not rotating the claw so that it doesn't hit the cone or pole
                .UNSTABLE_addTemporalMarkerOffset(0.3,()-> startLift(10, 400, ROTATE_UPSIDE)) // starts rotating the claw after a delay, avoiding hitting anything with the claw
                .lineToSplineHeading(new Pose2d(0, -50, Math.toRadians(90))) // turns to face the starting wall while moving to the center of the tall pole
                .UNSTABLE_addTemporalMarkerOffset(0,()-> startLift(10, 50, ROTATE_UPSIDE))
                .lineToSplineHeading(new Pose2d(-23, -51, Math.toRadians(90))) // moves to the first parking zone
                .splineToConstantHeading(new Vector2d(-23,-33), Math.toRadians(90)) // runs forward a little bit
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> stopLift()) // stops the lifts
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
        //leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
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

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "rightCamera"), cameraMonitorViewId);
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
