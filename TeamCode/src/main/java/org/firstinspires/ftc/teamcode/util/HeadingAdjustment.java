package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class HeadingAdjustment {

    public Telemetry telemetry;

    public BNO055IMU imu;

    public HardwareMap hardwareMap;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private double headingGoal, integral, error, previousError;

    private final double MIN_PID = 0.2, MAX_PID = 0.8, P = 0.01, I = 0, D = 0;

    private long previousLoopTime = 0;

    /**
     * IMPORTANT: right is negative, left is positive for the IMU
     */

    public HeadingAdjustment(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
        BNO055IMUUtil.swapThenFlipAxes(imu,AxesOrder.YZX,AxesSigns.PPP);
    }

    public double PID() {
        double deltaTime = (System.nanoTime() - previousLoopTime)/Math.pow(10,9);
        previousLoopTime = System.nanoTime();
        previousError = error;
        error = Math.min(leftError(),rightError());
        integral += (error*deltaTime); // Integral is increased by the error*time
        double derivative = (error - previousError) / deltaTime;
        telemetry.addData("integral", integral);
        return P*error + I*integral + D*derivative;

    }

    private double clamp(double num, double min, double max) {
        if (num < min) {
            return min;
        } else if (num > max) {
            return max;
        }
        return num;
    }

    public void correctError(double angularError, float timeOut) {
        double initialHeading = headingDegrees();
        long startTime = System.nanoTime();
        previousLoopTime = System.nanoTime();
        error = Math.min(leftError(),rightError());
        integral = 0;
        while (Math.min(leftError(),rightError())>angularError) {
            // positive power runs backwards
            if (leftError() < rightError()) { // this means we turn right
                leftFront.setPower(-clamp(PID(),MIN_PID,MAX_PID));
                leftRear.setPower(-clamp(PID(),MIN_PID,MAX_PID));
                rightFront.setPower(clamp(PID(),MIN_PID,MAX_PID));
                rightRear.setPower(clamp(PID(),MIN_PID,MAX_PID));
            } else { // this means we turn left
                leftFront.setPower(clamp(PID(),MIN_PID,MAX_PID));
                leftRear.setPower(clamp(PID(),MIN_PID,MAX_PID));
                rightFront.setPower(-clamp(PID(),MIN_PID,MAX_PID));
                rightRear.setPower(-clamp(PID(),MIN_PID,MAX_PID));
            }
            if (System.nanoTime()-startTime>timeOut*Math.pow(10,6)) {
                break;
            }
            telemetry.addData("initial heading", initialHeading);
            telemetry.addData("heading goal", headingGoal);
            telemetry.addData("heading", headingDegrees());
            telemetry.addData("left error", leftError());
            telemetry.addData("right error", rightError());
            telemetry.addData("PID", PID());
            telemetry.update();
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public double leftError() {
        if (headingDegrees()-headingGoal<0) {
            return headingDegrees()-headingGoal+360;
        } else {
            return headingDegrees()-headingGoal;
        }
    }

    public double rightError() {
        if (headingGoal-headingDegrees()<0) {
            return headingGoal-headingDegrees()+360;
        } else {
            return headingGoal-headingDegrees();
        }
    }

    /**
     * @param offset this is in degrees! Also, offset being positive means to the right and vice versa. However, this is reversed for the IMU.
     */
    public void setHeadingGoal(double offset) {
        headingGoal = headingDegrees() - offset;
        if (headingGoal > 180) {
            headingGoal = -180+(headingGoal-180);
        }
        if (headingGoal < -180) {
            headingGoal = 180 + (headingGoal + 180);
        }
    }

    public void setHeadingGoal() {
        headingGoal = headingDegrees();
    }

    public void resetHeadingGoal(double setTo) {
        headingGoal = setTo;
    }

    public double getHeadingGoal() {
        return headingGoal;
    }

    public double headingDegrees() {
        return imu.getAngularOrientation().firstAngle;
    }

    /*
     * I don't use this but it's here for funsies
     */
    public double headingRadians() {
        return Math.toRadians(headingDegrees());
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}

/**
 * 8==D
 */
