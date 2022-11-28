package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Single Person Drive", group = "Drive")
public class OnePersonDrive extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, strafeEncoder, leftLift, rightLift, arm, liftMotor;

    private final String setLiftMotor = "leftLift";

    private boolean clawButtonPressed, rotateButtonPressed, liftInMotion;

    private Servo rotate, claw;

    private int lastLiftPosition;

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0.65, CLAW_CLOSE = 0;

    private final int TALL = 3100, MEDIUM = 250, LOW = 3100,
            ARM_FLIPPED = 1000, ARM_SHORT = 150;

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
         * left stick forward/backward and turn
         * bumpers strafe
         *
         */
        while (opModeIsActive())
        {
            double y =  gamepad1.left_stick_y; // Remember, this is reversed!
            double rx = -gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double x = 0; if (gamepad1.right_bumper) {x = -1;} if (gamepad1.left_bumper) {x = 1;}

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

            doClaw();

            doRotate();

            /*
            arm presets:
            dpad_up - tall pole
            dpad_left - medium pole
            dpad_down - short pole
            dpad_right - reset
             */

            if (gamepad1.x)
            {
                liftInMotion = false;
            }

            if (!liftInMotion)
            {
                if (gamepad1.right_trigger>0) {
                    arm.setPower(gamepad1.right_trigger*0.5);
                } else if (gamepad1.left_trigger>0) {
                    arm.setPower(-gamepad1.left_trigger*0.5);
                } else {
                    arm.setPower(0);
                }

                if ((leftLift.getPower()>0&&liftMotor.getCurrentPosition()>3100)||(leftLift.getPower()<0&&liftMotor.getCurrentPosition()<0)) {
                    leftLift.setPower(0);
                } else {
                    if (gamepad1.right_stick_y!=0) {
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftLift.setPower(-gamepad1.right_stick_y);
                        lastLiftPosition = leftLift.getCurrentPosition();
                    } else {
                        leftLift.setTargetPosition(lastLiftPosition);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setVelocity(1440 * 2);
                    }
                }

                if (gamepad1.dpad_up)
                {
                    liftInMotion = true;
                    new Thread(new Runnable() {
                        public void run() {
                                liftToPositionAndFlip(TALL, ARM_FLIPPED, ROTATE_DOWNSIDE);
                        }
                    }).start();
                }
                if (gamepad1.dpad_left)
                {
                    liftInMotion = true;
                    new Thread(new Runnable() {
                        public void run() {
                            liftToPositionAndFlip(MEDIUM, ARM_FLIPPED, ROTATE_DOWNSIDE);
                        }
                    }).start();
                }
                if (gamepad1.dpad_down)
                {
                    liftInMotion = true;
                    new Thread(new Runnable() {
                        public void run() {
                            liftToPositionAndFlip(LOW, ARM_SHORT, ROTATE_UPSIDE);
                        }
                    }).start();
                }
                if (gamepad1.dpad_right)
                {
                    liftInMotion = true;
                    new Thread(new Runnable() {
                        public void run() {
                            liftToPositionAndFlip(50, 50, ROTATE_UPSIDE);
                        }
                    }).start();
                }
            }

            telemetry.addData("claw button pressed", clawButtonPressed);
            telemetry.addData("left lift position", leftLift.getCurrentPosition());
            telemetry.addData("right lift position", rightLift.getCurrentPosition());
            telemetry.addData("lift position", liftMotor.getCurrentPosition());
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("arm power", arm.getPower());
            telemetry.addData("lift in motion", liftInMotion);
            telemetry.addData("left encoder", leftFront.getCurrentPosition());
            telemetry.addData("right encoder", rightRear.getCurrentPosition());
            telemetry.addData("strafe encoder", strafeEncoder.getCurrentPosition());
            telemetry.update();
        }
    }

    public void liftToPositionAndFlip(int liftPosition, int armPosition, double rotatePosition) {
        /*
        double liftPower = 1;
        double armPower = 0.4;
        long startTime = System.currentTimeMillis();
        long timeOut = 2500;
        int totalArmDistance = Math.abs(armPosition-arm.getCurrentPosition());

        if (liftPosition>leftLift.getCurrentPosition())
        {
            leftLift.setPower(liftPower);
        }
        else
        {
            leftLift.setPower(-liftPower);
        }
        if (armPosition>arm.getCurrentPosition())
        {
            arm.setPower(armPower);
        }
        else
        {
            arm.setPower(-armPower);
        }
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setPosition(rotatePosition);
        while(arm.isBusy())
        {
            if (System.currentTimeMillis()-startTime>timeOut)
            {
                break;
            }
            if (!leftLift.isBusy())
            {
                leftLift.setPower(0);
            }
            int currentArmDistance = Math.abs(armPosition-arm.getCurrentPosition());
            if((double)currentArmDistance/totalArmDistance < 1.0/5)
            {
                if (armPosition>arm.getCurrentPosition())
                {
                    arm.setPower(armPower*-0.1);
                }
                else
                {
                    arm.setPower(-armPower*-0.1);
                }
            }
            if((double)currentArmDistance/totalArmDistance > 1.0/5)
            {
                if (armPosition>arm.getCurrentPosition())
                {
                    arm.setPower(armPower);
                }
                else
                {
                    arm.setPower(-armPower);
                }
            }
        }
        arm.setPower(0);
        while(leftLift.isBusy())
        {
            if (System.currentTimeMillis()-startTime>timeOut)
            {
                break;
            }
        }
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
         */
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
        while(arm.isBusy())
        {
            if (System.currentTimeMillis()-startTime>timeOut)
            {
                break;
            }
            if (!leftLift.isBusy())
            {
                leftLift.setPower(0);
            }
        }
        arm.setPower(0);
        while(leftLift.isBusy())
        {
            if (System.currentTimeMillis()-startTime>timeOut)
            {
                break;
            }
        }
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftInMotion = false;
        lastLiftPosition = leftLift.getCurrentPosition();
    }

    public void doClaw() {
        if (gamepad1.a)
        {
            if (clawButtonPressed)
            {
            }
            else
            {
                clawButtonPressed = true;
                if (claw.getPosition()==CLAW_CLOSE)
                {
                    claw.setPosition(CLAW_OPEN);
                }
                else
                {
                    claw.setPosition(CLAW_CLOSE);
                }
            }
        }
        else
        {
            if (clawButtonPressed)
            {
                clawButtonPressed = false;
            }
        }
    }

    public void doRotate() {
        if (gamepad1.b)
        {
            if (rotateButtonPressed)
            {
            }
            else
            {
                rotateButtonPressed = true;
                if (rotate.getPosition()==ROTATE_UPSIDE)
                {
                    rotate.setPosition(ROTATE_DOWNSIDE);
                }
                else
                {
                    rotate.setPosition(ROTATE_UPSIDE);
                }
            }
        }
        else
        {
            if (rotateButtonPressed)
            {
                rotateButtonPressed = false;
            }
        }
    }
}

/**
 * 8==D
 */