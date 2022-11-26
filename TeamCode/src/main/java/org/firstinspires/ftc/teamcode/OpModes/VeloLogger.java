package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Random;

@TeleOp
public class VeloLogger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        NanoClock clock = NanoClock.system();
        String filename = Misc.formatInvariant("EncoderVelocityData-%d.csv", System.currentTimeMillis());
        File file = new File(AppUtil.ROOT_FOLDER, filename);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try (PrintWriter pw = new PrintWriter(file)) {
            pw.println("pos,vel,time");
            while (!isStopRequested()) {
                leftFront.setPower(1);
                leftRear.setPower(1);
                rightFront.setPower(-1);
                rightRear.setPower(-1);
                imu.getAngularOrientation();
                pw.printf("%d,%f,%f%n", leftFront.getCurrentPosition(), leftFront.getVelocity(), clock.seconds());
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
