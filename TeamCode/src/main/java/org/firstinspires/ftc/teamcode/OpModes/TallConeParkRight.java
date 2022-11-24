package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Right Tall Pole Then Park", group = "Deprecated")
public class TallConeParkRight extends LinearOpMode {

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

    private DcMotor leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, arm;

    private Servo rotate, claw;

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0.7, CLAW_CLOSE = 0;

    private final int LEFT = 270, RIGHT = 90, FORWARD = 0, BACKWARD = 180, LIFT_MAXIMUM = 7000,
    TALL = 5000, MEDIUM = 2000, LOW = 0, ARM_FLIPPED = 900, ARM_VERTICAL = 600;

    private boolean loop = true;

    public void autonomous()
    {
        claw.setPosition(CLAW_CLOSE);//grabs cone
        sleep(500);
        arm.setPower(0.5);//raises arm slightly to avoid running it over
        sleep(50);
        arm.setPower(0);
        rotate.setPosition(ROTATE_UPSIDE);//adjusts rotate position from starting position
        //run forward for like 1 second
        //strafe(1, FORWARD, 1275); this is about a tile plus a tiny bit
        strafe(1, FORWARD, 2250);//run forward 2 tiles
        sleep(200);
        turn(0.7,"right",2375);//turn right to tall pole backwards
        liftToPositionAndFlip(TALL+500, ARM_FLIPPED, ROTATE_DOWNSIDE);//flip over arm and raise lifts
        strafe(0.5, BACKWARD, 300);//moves towards pole
        sleep(1000);
        leftLift.setPower(-0.5);//lowers lift
        sleep(1200);
        leftLift.setPower(0);
        claw.setPosition(CLAW_OPEN);//releases cone
        sleep(500);
        liftToPositionAndFlip(0 , 150, ROTATE_UPSIDE);//put arm and lifts back
        strafe(0.5, FORWARD, 420);//go back a bit
        turn(0.7,"left",2350);//turn left to center `
        strafe(0.7, BACKWARD, 1500); //run backwards to position
        strafe(1,LEFT,50);//adjustment
        switch(positionToGo)
        {
            case 1:
                //run left
                strafe(1,LEFT,1225);
                break;
            case 2:
                //stay in place
                break;
            case 3:
                //run right
                strafe(1,RIGHT,1675);
                break;
        }
        arm.setPower(-0.5);
        sleep(300);
        arm.setPower(0);
    }

    public void liftToPositionAndFlip(int liftPosition, int armPosition, double rotatePosition)
    {
        if (liftPosition>leftLift.getCurrentPosition())
        {
            leftLift.setPower(1);
        }
        else
        {
            leftLift.setPower(-1);
        }
        if (armPosition>arm.getCurrentPosition())
        {
            arm.setPower(0.5);
        }
        else
        {
            arm.setPower(-0.5);
        }
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setPosition(rotatePosition);
        while(arm.isBusy())
        {
            if (!leftLift.isBusy())
            {
                leftLift.setPower(0);
            }
        }
        arm.setPower(0);
        while(leftLift.isBusy()){}
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void strafe(double power, double direction, long millis)
    {
        //direction will be between 0 and 359
        direction += 90;
        while (direction>360) direction -= 360;

        double robotY = Math.sin(Math.toRadians(direction)); // Remember, this is reversed!
        double robotX = Math.cos(Math.toRadians(direction)) * 1; // Counteract imperfect strafing // TODO: If strafing is imperfect, then change 1 to 1.1 or something

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double robotDenominator = Math.max(Math.abs(robotY) + Math.abs(robotX), 1);
        double frontLeftPower = ((robotY + robotX) / robotDenominator)*power;
        double backLeftPower = ((robotY - robotX) / robotDenominator)*power;
        double frontRightPower = ((robotY - robotX) / robotDenominator)*power;
        double backRightPower = ((robotY + robotX) / robotDenominator)*power;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        sleep(millis);

        setDriveMotorsZero();
    }

    public void turn(double power, String direction, long millis)
    {
        switch (direction)
        {
            case "left":
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-1*power);
                rightRear.setPower(-1*power);
                break;
            case "right":
                leftFront.setPower(-1*power);
                leftRear.setPower(-1*power);
                rightFront.setPower(power);
                rightRear.setPower(power);
                break;
        }

        sleep(millis);

        setDriveMotorsZero();
    }

    public void setDriveMotorsZero()
    {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public void runOpMode()
    {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
        arm = hardwareMap.dcMotor.get("arm");
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
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate = hardwareMap.get(Servo.class, "rotate");
        claw = hardwareMap.get(Servo.class, "claw");
        rotate.setPosition(0.65);
        claw.setPosition(CLAW_OPEN);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            detectTag(0);
            detectTag(1);
            detectTag(2);
        }

        parkingPosition();

        autonomous();
    }

    public void parkingPosition()
    {
        if(tagOfInterest == null)
        {
            /*
             * tag was never sighted during INIT
             */
            positionToGo = 2;

        }
        else
        {
            /*
             * tag sighted
             */
            switch (tagOfInterest.id)
            {
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

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void detectTag(int tagID)
    {
        ID_TAG_OF_INTEREST = tagID;
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == ID_TAG_OF_INTEREST)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);
    }
}