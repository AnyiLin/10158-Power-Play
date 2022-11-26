package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CustomOdometry {

    public Telemetry telemetry;

    public HardwareMap hardwareMap;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, leftEncoder, rightEncoder, strafeEncoder;

    private final int TICKS_PER_REV = 4096;
    private final double WHEEL_RADIUS = 1.8898; // this is in inches
    private final double END_RADIUS = 3; // in inches as well, basically this is when the robot begins its final error correction and the time limit starts
    private final double STRAFE_ERROR = 0.5; // also in inches
    private final double ANGULAR_ERROR = 5; // this is in degrees
    private final float CORRECTION_TIME = (float) 1; // this is in seconds, it's the time allowed for correction at the end of the course.

    public CustomOdometry(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "strafeEncoder");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // +y direction is robot forward
    // +x direction is robot right
    public void strafe(double xInches, double yInches) {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angleToEnd = findAngle(xInches,yInches); // this is in radians
        while (distanceToEndInches(xInches, yInches)>END_RADIUS) {
            if (strafeErrorInches(xInches, yInches)>STRAFE_ERROR) {

            } else if (false) {
                // TODO: Add in angular error correction. Do we prioritise it over strafe error?
            } else { // if there is no excessive error, we drive as planned.
                drive(currentAngleToEnd(xInches, yInches),0.3);
            }
            // for now we test if we need error correction in the middle if we constantly update the angle to end
            drive(currentAngleToEnd(xInches, yInches),0.3);
            if (!telemetry.equals(null)) {
                telemetry.addData("distance to end",distanceToEndInches(xInches, yInches));
                telemetry.update();
            }
        }

    }

    /**
     * @param angle remember this is in radians!
     */
    public void drive(double angle, double power) {
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
    }

    public double distanceToEndInches(double xInches, double yInches) {
        return distance(ticksToInches(strafeEncoder.getCurrentPosition()),ticksToInches(averagePosition()),xInches,yInches);
    }

    public double strafeErrorInches(double xInches, double yInches) {
        return pDistance(ticksToInches(strafeEncoder.getCurrentPosition()),ticksToInches(averagePosition()),0,0,xInches,yInches);
    }

    public double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x1-x2, y1-y2);
    }

    public double pDistance(double x, double y, double x1, double y1, double x2, double y2) {

        double A = x - x1; // position of point rel one end of line
        double B = y - y1;
        double C = x2 - x1; // vector along line
        double D = y2 - y1;
        double E = -D; // orthogonal vector
        double F = C;

        double dot = A * E + B * F;
        double len_sq = E * E + F * F;

        return Math.abs(dot)/Math.sqrt(len_sq);
    }

    public int averagePosition() {
        return -(leftEncoder.getCurrentPosition()+rightEncoder.getCurrentPosition())/2;
    }

    public double currentAngleToEnd(double xInches, double yInches) {
        return findAngle(xInches - ticksToInches(strafeEncoder.getCurrentPosition()),yInches - ticksToInches(averagePosition()));
    }

    public double findAngle(double x, double y) {
        if (x<0) {
            return Math.atan(y/x)+Math.PI;
        } else if (x>0) {
            return Math.atan(y/x);
        } else {
            if (y>0) {
                return Math.PI/2;
            } else {
                return -Math.PI/2;
            }
        }
    }

    public double ticksToInches(int ticks) {
        return ((double)ticks/TICKS_PER_REV)*(2*Math.PI*WHEEL_RADIUS);
    }

    public int inchesToTicks(double inches) {
        return (int)(TICKS_PER_REV*(inches/(2*Math.PI*WHEEL_RADIUS)));
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}
