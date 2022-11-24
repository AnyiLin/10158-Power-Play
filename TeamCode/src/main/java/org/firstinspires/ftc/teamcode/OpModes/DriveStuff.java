package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp (name = "Single Person Drive Without Encoders", group = "Deprecated")
public class DriveStuff extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, arm;

    private boolean clawButtonPressed, rotateButtonPressed;

    private Servo rotate, claw;

    private final double ROTATE_UPSIDE = 1, ROTATE_DOWNSIDE = -1, CLAW_OPEN = 0.7, CLAW_CLOSE = 0;

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
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate = hardwareMap.get(Servo.class, "rotate");
        claw = hardwareMap.get(Servo.class, "claw");
        rotate.setPosition(ROTATE_UPSIDE);
        claw.setPosition(CLAW_OPEN);

        waitForStart();

        /**
         * left stick strafes
         * right stick turns
         *
         */
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double rx = 0; if (gamepad1.right_bumper) {rx = -1;} if (gamepad1.left_bumper) {rx = 1;}

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

            if (gamepad1.right_trigger>0) {
                arm.setPower(-gamepad1.right_trigger*0.5);
            } else if (gamepad1.left_trigger>0) {
                arm.setPower(gamepad1.left_trigger*0.5);
            } else {
                arm.setPower(0);
            }

            leftLift.setPower(-gamepad1.right_stick_y);
            rightLift.setPower(-gamepad1.right_stick_y);

            doClaw();

            doRotate();

            telemetry.addData("claw button pressed", clawButtonPressed);
            telemetry.addData("left lift position", leftLift.getCurrentPosition());
            telemetry.update();
        }
    }

    public void doClaw() {
        if (gamepad1.b)
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
        if (gamepad1.a)
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