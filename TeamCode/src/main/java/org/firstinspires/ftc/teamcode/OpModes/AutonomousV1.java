package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous (name = "Also Park", group = "Deprecated")
public class AutonomousV1 extends LinearOpMode {

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

    private AprilTagDetection tagOfInterest = null;

    private final String liftEncoderMotor = "leftLift";

    private boolean asyncLiftRunning, clawMoving, rotateMoving;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private DcMotor leftLift, rightLift, arm, liftMotor;

    private Servo rotate, claw;

    private BNO055IMU imu;

    private final double ROTATE_UPSIDE = -1, ROTATE_DOWNSIDE = 1, CLAW_OPEN = 0, CLAW_CLOSE = 0;

    private final long FLIP_WAIT_TIME = 700;

    private final int ARM_FLIP_HEIGHT = 0, MAXIMUM_LIFT = 0, MAXIMUM_ARM = 0, LOW = 0, MEDIUM = 0, HIGH = 0, LEFT = 90, RIGHT = 270, FORWARD = 0, BACKWARD = 180;

    private int liftTargetPosition, positionToGo;

    public void autonomous()
    {
        //TODO: write the whole autonomous program for the school round
        //run forward for like 1 second
        strafe(1, FORWARD, 1000);
        switch(positionToGo)
        {
            case 1:
                //run left
                strafe(1,LEFT,400);
                break;
            case 2:
                //stay in place
                break;
            case 3:
                //run right
                strafe(1,RIGHT,400);
                break;
        }
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
                leftFront.setPower(-1*power);
                leftRear.setPower(-1*power);
                rightFront.setPower(power);
                rightRear.setPower(power);
                break;
            case "right":
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-1*power);
                rightRear.setPower(-1*power);
                break;
        }

        sleep(millis);

        setDriveMotorsZero();
    }

    @Override
    public void runOpMode()
    {
        initialize();

        while (!isStarted() && !isStopRequested())
        {
            for (int counter = 0; counter < 100; counter++)
                detectTag(0);
            for (int counter = 0; counter < 100; counter++)
                detectTag(1);
            for (int counter = 0; counter < 100; counter++)
                detectTag(2);
        }
        parkingPosition();
        displayInfo();
        autonomous();
    }

    public void initialize()
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
        arm = hardwareMap.dcMotor.get("arm");
        switch (liftEncoderMotor)
        {
            case "leftLift":
                liftMotor = leftLift;
                break;
            case "rightLift":
                liftMotor = rightLift;
                break;
        }
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate = hardwareMap.get(Servo.class, "rotate");
        claw = hardwareMap.get(Servo.class, "claw");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        rotate.setPosition(ROTATE_UPSIDE);
        claw.setPosition(CLAW_OPEN);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        // set axes
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);
        //TODO: implement roadrunner here

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

    public void layout2J(Gamepad gamepad)
    {
        double robotY = gamepad.right_stick_y; // Remember, this is reversed!
        double robotX = -gamepad.right_stick_x * 1.1; // Counteract imperfect strafing // TODO: If strafing is imperfect, then change 1 to 1.1 or something
        double fieldY = -gamepad.left_stick_y; // Remember, this is reversed!
        double fieldX = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing // TODO: If strafing is imperfect, then change 1 to 1.1 or something
        double rx = 0; if (gamepad.right_bumper) {rx = -1;} if (gamepad.left_bumper) {rx = 1;}

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = fieldX * Math.cos(botHeading) - fieldY * Math.sin(botHeading);
        double rotY = fieldX * Math.sin(botHeading) + fieldY * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double robotDenominator = Math.max(Math.abs(robotY) + Math.abs(robotX) + Math.abs(rx), 1);
        double fieldDenominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rx), 1);
        double frontLeftPower = (robotY + robotX + rx) / robotDenominator + (rotY + rotX + rx) / fieldDenominator;
        double backLeftPower = (robotY - robotX + rx) / robotDenominator + (rotY - rotX + rx) / fieldDenominator;
        double frontRightPower = (robotY - robotX - rx) / robotDenominator + (rotY - rotX - rx) / fieldDenominator;
        double backRightPower = (robotY + robotX - rx) / robotDenominator + (rotY + rotX - rx) / fieldDenominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        setLiftPower(0);
        lift(1);
        lift(-1);
        lift(0.5);
        lift(-0.5);

        setArmPower(0);
        arm(gamepad.right_trigger);
        arm(-1*gamepad.left_trigger);
        arm(0.1);
        arm(-0.1);

        toggleClaw();
        toggleRotate();
        returnLiftAndFlip();
    }

    public void displayInfo() {
        new Thread(new Runnable() {
            public void run() {
                while (opModeIsActive())
                {
                    asyncDisplayInfo();
                }
            }
        }).start();
    }

    public void asyncDisplayInfo()
    {
        telemetry.addData("tag detected ", tagOfInterest.id);
        telemetry.addData("left lift position ", leftLift.getCurrentPosition());
        telemetry.addData("right lift position ", rightLift.getCurrentPosition());
        telemetry.addData("lift position ", getLiftPosition());
        telemetry.addData("arm position ", arm.getCurrentPosition());
        telemetry.update();
    }

    public void lift(int position)
    {
        new Thread(new Runnable() {
            public void run(){
                liftToPosition(position);
            }
        }).start();
    }

    public void setLiftPower(double power)
    {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public int getLiftPosition()
    {
        return liftMotor.getCurrentPosition();
    }

    public void lift(double power)
    {
        runLift(power);
    }

    public void runLift(double power)
    {
        setLiftPower(power);
        if (power>0&&getLiftPosition()>MAXIMUM_LIFT)
        {
            setLiftPower(0);
            liftToPosition(MAXIMUM_LIFT);
            //TODO: make it go back down to the max height it if exceeds it
        }
        if (power<0&&getLiftPosition()<0)
        {
            setLiftPower(0);
            liftToPosition(0);
            //TODO: make it go back up to the lowest height if it goes down too far
        }
    }

    public void liftToPosition(int position)
    {
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setLiftPower(1);
        liftMotor.setTargetPosition(position);
        while (liftMotor.isBusy())
        {
        }
        setLiftPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void liftAndFlip(int position)
    {
        new Thread(new Runnable() {
            public void run()
            {
                asyncLiftAndFlip(position);
            }
        }).start();
    }

    public void waitForArmFlip()
    {
        sleep(FLIP_WAIT_TIME);
    }

    public void asyncLiftAndFlip(int position)
    {
        asyncLiftRunning = true;
        liftToPosition(ARM_FLIP_HEIGHT);
        flipArmAndClaw();
        waitForArmFlip();
        liftToPosition(position);
        asyncLiftRunning = false;
    }

    public void flipArmAndClaw()
    {
        new Thread(new Runnable() {
            public void run() {
                asyncFlipArmAndClaw();
            }
        }).start();
    }

    public void asyncFlipArmAndClaw()
    {
        flipClaw();
        armToPosition(MAXIMUM_ARM);
    }

    public void flipClaw()
    {
        new Thread(new Runnable() {
            public void run() {
                asyncFlipClaw();
            }
        }).start();
    }

    public void asyncFlipClaw()
    {
        rotate.setPosition(ROTATE_DOWNSIDE);
    }

    public void returnLiftAndFlip()
    {
        new Thread(new Runnable() {
            public void run()
            {
                asyncReturnLiftAndFlip();
            }
        }).start();
    }

    public void asyncReturnLiftAndFlip()
    {
        asyncLiftRunning = true;
        liftToPosition(ARM_FLIP_HEIGHT);
        returnFlipArmAndClaw();
        waitForArmFlip();
        liftToPosition(0);
        asyncLiftRunning = false;
    }

    public void returnFlipArmAndClaw()
    {
        new Thread(new Runnable() {
            public void run() {
                asyncReturnFlipArmAndClaw();
            }
        }).start();
    }

    public void asyncReturnFlipArmAndClaw()
    {
        returnFlipClaw();
        armToPosition(0);
    }

    public void returnFlipClaw()
    {
        new Thread(new Runnable() {
            public void run() {
                returnFlipClaw();
            }
        }).start();
    }

    public void asyncReturnFlipClaw()
    {
        rotate.setPosition(ROTATE_UPSIDE);
    }

    public int getArmPosition()
    {
        return arm.getCurrentPosition();
    }

    public void arm(int position)
    {
        new Thread(new Runnable() {
            public void run(){
                armToPosition(position);
            }
        }).start();
    }

    public void setArmPower(double power)
    {
        arm.setPower(power);
    }

    public void arm(double power)
    {
        runArm(power);
    }

    public void runArm(double power)
    {
        setArmPower(power);
        if (power>0&&getArmPosition()>MAXIMUM_ARM)
        {
            setArmPower(0);
            armToPosition(MAXIMUM_ARM);
            //TODO: make it go back down to the max height it if exceeds it
        }
        if (power<0&&getArmPosition()<0)
        {
            setArmPower(0);
            armToPosition(0);
            //TODO: make it go back up to the lowest height if it goes down too far
        }
    }

    public void armToPosition(int position)
    {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmPower(1);
        leftLift.setTargetPosition(position);
        while (leftLift.isBusy())
        {
        }
        setArmPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void toggleClaw()
    {
        if (!clawMoving) {
            asyncToggleClaw();
        }
    }

    public void asyncToggleClaw()
    {
        clawMoving = true;
        if (claw.getPosition()==CLAW_OPEN)
        {
            claw.setPosition(CLAW_CLOSE);
            while (claw.getPosition()!=CLAW_CLOSE)
            {
            }
        }
        else if (claw.getPosition()==CLAW_CLOSE)
        {
            claw.setPosition(CLAW_OPEN);
            while (claw.getPosition()!=CLAW_OPEN)
            {
            }
        }
        clawMoving = false;
    }

    public void toggleRotate()
    {
        if (!rotateMoving) {
            asyncToggleRotate();
        }
    }

    public void asyncToggleRotate()
    {
        rotateMoving = true;
        if (rotate.getPosition()==ROTATE_UPSIDE)
        {
            rotate.setPosition(ROTATE_DOWNSIDE);
            while (rotate.getPosition()!=ROTATE_DOWNSIDE)
            {
            }
        }
        else if (rotate.getPosition()==ROTATE_DOWNSIDE)
        {
            rotate.setPosition(ROTATE_UPSIDE);
            while (rotate.getPosition()!=ROTATE_UPSIDE)
            {
            }
        }
        rotateMoving = false;
    }

    public void setDriveMotorsZero()
    {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
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