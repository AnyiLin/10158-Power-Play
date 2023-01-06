package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp (name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, strafeEncoder, leftLift, rightLift, arm, liftMotor;

    private final String setLiftMotor = "leftLift";

    private boolean clawButtonPressed, rotateButtonPressed, liftInMotion;

    private Servo rotate, claw;

    private int lastLiftPosition, lastArmPosition;

    private long startTime;

    private final long LIFT_TIME_OUT = 1500;
    private final double ROTATE_UPSIDE = RobotConstants.ROTATE_UPSIDE, ROTATE_DOWNSIDE = RobotConstants.ROTATE_DOWNSIDE, CLAW_OPEN = RobotConstants.CLAW_OPEN, CLAW_CLOSE = RobotConstants.CLAW_CLOSE, LIFT_ERROR = RobotConstants.LIFT_ERROR;

    private final int TALL = RobotConstants.TALL, MEDIUM = RobotConstants.MEDIUM, LOW = RobotConstants.LOW, CONE_STACK = RobotConstants.CONE_STACK, ARM_FLIPPED = RobotConstants.ARM_FLIPPED, ARM_SHORT = RobotConstants.ARM_SHORT, LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY, ARM_VELOCITY = RobotConstants.ARM_VELOCITY, LIFT_MAXIMUM = RobotConstants.LIFT_MAXIMUM, LIFT_MINIMUM = RobotConstants.LIFT_MINIMUM;

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
        //leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (setLiftMotor)
        {
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

        waitForStart();

        /**
         * left stick forward/backward and left/right turning
         * right will be field centric
         * triggers strafe robot
         */
        while (opModeIsActive())
        {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.right_trigger+gamepad1.left_trigger;
            double rx = -gamepad1.left_stick_x * 1; // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            //TODO: add field centric drive once odometry and stuff works

            doClaw();

            doRotate();

            /*
            arm presets:
            dpad_up - tall pole
            dpad_left - medium pole
            dpad_down - short pole
            dpad_right - reset
             */

            if (gamepad2.x) {
                liftInMotion = false;
            }

            if (!liftInMotion) {
                if (gamepad2.right_stick_button) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastArmPosition = arm.getCurrentPosition();
                }
                if (gamepad2.left_stick_button) {
                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastLiftPosition = liftMotor.getCurrentPosition();
                }

                //y stick inputs are -1 for top and 1 for bottom
                if (gamepad2.right_stick_y!=0) {
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (gamepad2.right_bumper) { //fine adjustment
                        arm.setPower(gamepad2.right_stick_y / 4);
                    } else { //normal
                        arm.setPower(gamepad2.right_stick_y / 2);
                    }
                    lastArmPosition = arm.getCurrentPosition();
                } else {
                    arm.setTargetPosition(lastArmPosition);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(ARM_VELOCITY/4);
                }

                if ((liftMotor.getPower()>0&&liftMotor.getCurrentPosition()>LIFT_MAXIMUM)||(liftMotor.getPower()<0&&liftMotor.getCurrentPosition()<LIFT_MINIMUM)) {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                } else {
                    if (gamepad2.left_stick_y!=0) {
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        if (gamepad2.right_bumper) { // fine adjustment
                            leftLift.setPower(-gamepad2.left_stick_y/2);
                            rightLift.setPower(-gamepad2.left_stick_y/2);
                        } else { // normal
                            leftLift.setPower(-gamepad2.left_stick_y);
                            rightLift.setPower(-gamepad2.left_stick_y);
                        }
                        if (leftLift.getCurrentPosition()-rightLift.getCurrentPosition()>LIFT_ERROR) {
                            dampenLiftMotor(leftLift);
                        }
                        if (rightLift.getCurrentPosition()-leftLift.getCurrentPosition()>LIFT_ERROR) {
                            dampenLiftMotor(rightLift);
                        }
                        lastLiftPosition = liftMotor.getCurrentPosition();
                    } else {
                        leftLift.setTargetPosition(lastLiftPosition);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setVelocity(LIFT_VELOCITY);
                        rightLift.setTargetPosition(lastLiftPosition);
                        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightLift.setVelocity(LIFT_VELOCITY);
                    }
                }
                if (gamepad2.dpad_up)
                {
                    startPreset(TALL,ARM_FLIPPED,ROTATE_DOWNSIDE);
                }
                if (gamepad2.dpad_left)
                {
                    startPreset(MEDIUM,ARM_FLIPPED,ROTATE_DOWNSIDE);
                }
                if (gamepad2.dpad_down)
                {
                    startPreset(LOW,ARM_SHORT,ROTATE_UPSIDE);
                }
                if (gamepad2.dpad_right)
                {
                    startPreset(0,0,ROTATE_UPSIDE);
                }
            } else {
                if (!arm.isBusy()&&!leftLift.isBusy()&&!rightLift.isBusy()) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLift.setPower(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setPower(0);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastLiftPosition = liftMotor.getCurrentPosition();
                    lastArmPosition = arm.getCurrentPosition();
                    liftInMotion = false;
                }
                if (System.currentTimeMillis()-startTime>LIFT_TIME_OUT) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLift.setPower(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setPower(0);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastLiftPosition = liftMotor.getCurrentPosition();
                    lastArmPosition = arm.getCurrentPosition();
                    liftInMotion = false;
                }
            }
            telemetry.addData("claw button pressed", clawButtonPressed);
            telemetry.addData("left lift position", leftLift.getCurrentPosition());
            telemetry.addData("left lift power", leftLift.getPower());
            telemetry.addData("right lift position", rightLift.getCurrentPosition());
            telemetry.addData("right lift power", rightLift.getPower());
            telemetry.addData("lift position", liftMotor.getCurrentPosition());
            telemetry.addData("last lift position", lastLiftPosition);
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("arm power", arm.getPower());
            telemetry.addData("lift in motion", liftInMotion);
            telemetry.addData("left front position", leftFront.getCurrentPosition());
            telemetry.addData("right front position", rightFront.getCurrentPosition());
            telemetry.addData("left rear position", leftRear.getCurrentPosition());
            telemetry.addData("right rear position", rightRear.getCurrentPosition());
            telemetry.addData("left encoder", -leftFront.getCurrentPosition());//reversed
            telemetry.addData("right encoder", -rightRear.getCurrentPosition());//reversed
            telemetry.addData("strafe encoder", strafeEncoder.getCurrentPosition());
            telemetry.update();
        }
    }

    public void dampenLiftMotor(DcMotorEx lift) {
        lift.setPower(lift.getPower()*(0.9+0.1* LIFT_ERROR/liftError()));
    }

    public int liftError() {
        return Math.abs(leftLift.getCurrentPosition()-rightLift.getCurrentPosition());
    }

    public void startPreset(int liftPosition, int armPosition, double rotatePosition) {
        liftInMotion = true;
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(LIFT_VELOCITY);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY);
        rotate.setPosition(rotatePosition);
        startTime = System.currentTimeMillis();
    }

    public void doClaw() {
        if (gamepad2.a) {
            if (!clawButtonPressed) {
                clawButtonPressed = true;
                if (claw.getPosition()==CLAW_CLOSE) {
                    claw.setPosition(CLAW_OPEN);
                } else {
                    claw.setPosition(CLAW_CLOSE);
                }
            }
        } else {
            if (clawButtonPressed) {
                clawButtonPressed = false;
            }
        }
    }

    public void doRotate() {
        if (gamepad2.b) {
            if (!rotateButtonPressed) {
                rotateButtonPressed = true;
                if (rotate.getPosition()==ROTATE_UPSIDE) {
                    rotate.setPosition(ROTATE_DOWNSIDE);
                } else {
                    rotate.setPosition(ROTATE_UPSIDE);
                }
            }
        } else {
            if (rotateButtonPressed) {
                rotateButtonPressed = false;
            }
        }
    }
}

/**
 * 8==D
 */