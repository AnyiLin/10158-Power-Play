package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.util.Arrays;
import java.util.List;

@TeleOp (name = "Funky Drive", group = "Deprecated")
public class Drive extends LinearOpMode {

    private StringBuilder jackLayout = new StringBuilder("1J"), yoyiLayout = new StringBuilder("1Y");

    private final String liftEncoderMotor = "leftLift";

    private boolean jackSwitching, yoyiSwitching, asyncLiftRunning, clawMoving, rotateMoving;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private DcMotor leftLift, rightLift, arm, liftMotor;

    private Servo rotate, claw;

    private BNO055IMU imu;

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0, CLAW_CLOSE = 0;

    private final long FLIP_WAIT_TIME = 700;

    private final int ARM_FLIP_HEIGHT = 0, MAXIMUM_LIFT = 3000, MAXIMUM_ARM = 0, LOW = 0, MEDIUM = 0, HIGH = 0;

    private int liftTargetPosition;

    /**
     * IMPORTANT NOTES:
     *
     * For the Double Shock controllers:
     * triangle - top
     * circle - right
     * cross (X-shaped) - bottom
     * square - left
     *
     * For the older Logitech controllers:
     * Y - top
     * B - right
     * A - bottom
     * X - left
     */

    @Override
    public void runOpMode()
    {
        initialize();

        while (!isStarted() && !isStopRequested())
        {
        }
        displayInfo();
        while (opModeIsActive())
        {
            switch (jackLayout.toString())
            {
                case "1J":
                    layout1J(gamepad1);
                    break;
                case "2J":
                    layout2J(gamepad1);
                    break;
                case "1Y":
                    layout1Y(gamepad1);
                    break;
            }
            switch (yoyiLayout.toString())
            {
                case "1J":
                    layout1J(gamepad2);
                    break;
                case "2J":
                    layout2J(gamepad2);
                    break;
                case "1Y":
                    layout1Y(gamepad2);
                    break;
            }
        }
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
        }

    public void layout1J(Gamepad gamepad)
    {
        if (gamepad == gamepad1&&!jackSwitching)
        {
            if(gamepad.left_stick_button) {
                detectButtonRelease(gamepad, "left_stick_buttonSwitch");jackLayout = new StringBuilder("2J");jackSwitching = true;}
            if(gamepad.right_stick_button) {
                detectButtonRelease(gamepad, "right_stick_buttonSwitch");jackLayout = new StringBuilder("1Y");jackSwitching = true;}
        }
        if (gamepad == gamepad2&&!yoyiSwitching)
        {
            if(gamepad.left_stick_button) {
                detectButtonRelease(gamepad, "left_stick_buttonSwitch");yoyiLayout = new StringBuilder("2J");yoyiSwitching  = true;}
            if(gamepad.right_stick_button) {
                detectButtonRelease(gamepad, "right_stick_buttonSwitch");yoyiLayout = new StringBuilder("1Y");yoyiSwitching  = true;}
        }

        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public void layout2J(Gamepad gamepad)
    {
        if (gamepad == gamepad1&&!jackSwitching)
        {
            if(gamepad.triangle||gamepad.y) {
                detectButtonRelease(gamepad, "topButtonSwitch");jackLayout = new StringBuilder("1J");jackSwitching = true;}
        }
        if (gamepad == gamepad2&&!yoyiSwitching)
        {
            if(gamepad.triangle||gamepad.y) {
                detectButtonRelease(gamepad, "topButtonSwitch");yoyiLayout = new StringBuilder("1J");yoyiSwitching  = true;}
        }

        double robotY = -gamepad.right_stick_y; // Remember, this is reversed!
        double robotX = -gamepad.right_stick_x * 1; // Counteract imperfect strafing // TODO: If strafing is imperfect, then change 1 to 1.1 or something
        double fieldY = -gamepad.left_stick_y; // Remember, this is reversed!
        double fieldX = gamepad.left_stick_x * 1; // Counteract imperfect strafing // TODO: If strafing is imperfect, then change 1 to 1.1 or something
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
        lift(1, gamepad, "dpad_up");
        lift(-1, gamepad, "dpad_down");
        lift(0.5, gamepad, "right_stick_button");
        lift(-0.5, gamepad, "left_stick_button");

        setArmPower(0);
        arm(gamepad.right_trigger, gamepad, "right_trigger");
        arm(-1*gamepad.left_trigger, gamepad, "left_trigger");
        arm(0.1, gamepad, "dpad_left");
        arm(-0.1, gamepad, "dpad_right");

        toggleClaw(gamepad, "bottomButton");
        toggleRotate(gamepad, "rightButton");
        returnLifts(gamepad, "leftButton");
    }

    public void layout1Y(Gamepad gamepad)
    {
        if (gamepad == gamepad1&&!jackSwitching)
        {
            if(gamepad.triangle||gamepad.y) {
                detectButtonRelease(gamepad, "topButtonSwitch");jackLayout = new StringBuilder("1J");jackSwitching = true;}
        }
        if (gamepad == gamepad2&&!yoyiSwitching)
        {
            if(gamepad.triangle||gamepad.y) {
                detectButtonRelease(gamepad, "topButtonSwitch");yoyiLayout = new StringBuilder("1J");yoyiSwitching  = true;}
        }

        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
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
        //shows layouts
        telemetry.addData("Jack's Layout ", jackLayout);
        telemetry.addData("Yoyi's Layout ", yoyiLayout);
        telemetry.addData("Jack switching ", jackSwitching);
        telemetry.addData("Yoyi switching ", yoyiSwitching);
        telemetry.addData("left lift position ", leftLift.getCurrentPosition());
        telemetry.addData("right lift position ", rightLift.getCurrentPosition());
        telemetry.addData("lift position ", getLiftPosition());
        telemetry.addData("arm position ", arm.getCurrentPosition());
        telemetry.addData("lift running", asyncLiftRunning);
        telemetry.addData("rotate moving ", rotateMoving);
        telemetry.addData("claw moving ", clawMoving);
        telemetry.update();
    }

    public void detectButtonRelease(Gamepad gamepad, String button)
    {
        new Thread(new Runnable() {
            public void run(){
                detectButtonReleaseAsync(gamepad, button);
            }
        }).start();
    }

    public void detectButtonReleaseAsync(Gamepad gamepad, String button)
    {
        switch (button)
        {
            case "left_stick_buttonSwitch":
                while(gamepad.left_stick_button) {}
                if (gamepad == gamepad1) {jackSwitching = false;} else {yoyiSwitching  = false;}
                break;
            case "right_stick_buttonSwitch":
                while(gamepad.right_stick_button) {}
                if (gamepad == gamepad1) {jackSwitching = false;} else {yoyiSwitching  = false;}
                break;
            case "topButtonSwitch":
                while(gamepad.triangle||gamepad.y) {}
                if (gamepad == gamepad1) {jackSwitching = false;} else {yoyiSwitching  = false;}
                break;
        }
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
        liftMotor.setPower(power);
    }

    public int getLiftPosition()
    {
        return liftMotor.getCurrentPosition();
    }

    public void lift(double power, Gamepad gamepad, String button)
    {
        switch (button)
        {
            case "dpad_up":
                if (gamepad.dpad_up)
                {
                    runLift(power);
                }
                break;
            case "dpad_down":
                if (gamepad.dpad_down)
                {
                    runLift(power);
                }
                break;
            case "right_stick_button":
                if (gamepad.right_stick_button)
                {
                    runLift(power);
                }
                break;
            case "left_stick_button":
                if (gamepad.left_stick_button)
                {
                    runLift(power);
                }
                break;
        }
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
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setLiftPower(1);
        while (liftMotor.isBusy())
        {
        }
        setLiftPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runLifts(Gamepad gamepad, String button)
    {
        if (!asyncLiftRunning) {
            switch (button) {
                case "leftButton":
                    if (gamepad.square || gamepad.x) {
                        new Thread(new Runnable() {
                            public void run() {
                                liftAndFlip(HIGH);
                            }
                        }).start();
                    }
                    break;
                case "bottomButton":
                    if (gamepad.cross || gamepad.a) {
                        new Thread(new Runnable() {
                            public void run() {
                                liftAndFlip(MEDIUM);
                            }
                        }).start();
                    }
                    break;
                case "rightButton":
                    if (gamepad.circle || gamepad.b) {
                        new Thread(new Runnable() {
                            public void run() {
                                liftAndFlip(LOW);
                            }
                        }).start();
                    }
                    break;
            }
        }
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

    public void returnLifts(Gamepad gamepad, String button)
    {
        if (!asyncLiftRunning) {
            switch (button) {
                case "leftButton":
                    if (gamepad.square || gamepad.x) {
                        new Thread(new Runnable() {
                            public void run() {
                                returnLiftAndFlip();
                            }
                        }).start();
                    }
                    break;
            }
        }
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

    public void arm(double power, Gamepad gamepad, String button)
    {
        switch (button)
        {
            case "right_trigger":
                if (gamepad.right_trigger>0)
                {
                    new Thread(new Runnable() {
                        public void run() {
                            runArm(power);
                        }
                    }).start();
                }
                break;
            case "left_trigger":
                if (gamepad.left_trigger>0)
                {
                    new Thread(new Runnable() {
                        public void run() {
                            runArm(power);
                        }
                    }).start();
                }
                break;
            case "dpad_left":
                if (gamepad.dpad_left)
                {
                    new Thread(new Runnable() {
                        public void run() {
                            runArm(power);
                        }
                    }).start();
                }
                break;
            case "dpad_right":
                if (gamepad.dpad_right)
                {
                    new Thread(new Runnable() {
                        public void run() {
                            runArm(power);
                        }
                    }).start();
                }
                break;
        }
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
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmPower(1);
        while (leftLift.isBusy())
        {
        }
        setArmPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void toggleClaw(Gamepad gamepad, String button)
    {
        if (!clawMoving) {
            switch (button) {
                case "bottomButton":
                    if (gamepad.cross || gamepad.a) {
                        new Thread(new Runnable() {
                            public void run() {
                                asyncToggleClaw();
                            }
                        }).start();
                    }
                    break;
            }
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

    public void toggleRotate(Gamepad gamepad, String button)
    {
        if (!rotateMoving) {
            switch (button) {
                case "rightButton":
                    if (gamepad.circle || gamepad.b) {
                        new Thread(new Runnable() {
                            public void run() {
                                asyncToggleRotate();
                            }
                        }).start();
                    }
                    break;
            }
        }
    }

    public void asyncToggleRotate()
    {
        rotateMoving = true;
        if (rotate.getPosition()>0)
        {
            rotate.setPosition(ROTATE_DOWNSIDE);
            while (rotate.getPosition()!=ROTATE_DOWNSIDE)
            {
            }
        }
        else if (rotate.getPosition()<0)
        {
            rotate.setPosition(ROTATE_UPSIDE);
            while (rotate.getPosition()!=ROTATE_UPSIDE)
            {
            }
        }
        rotateMoving = false;
    }
}