package org.firstinspires.ftc.team10309.autoOp;

// CHANGE WHILE LOOP LOGIC; MAKE SURE ROBOT GOES BACKWARDS IF IT GOES PAST THE TARGET.
//
import android.widget.ArrayAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

@Autonomous(name="Test Odo Move")
public class TestOdoMove extends LinearOpMode {
    
    private Robot robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);
        
        waitForStart();
        hardwareMap.get(DcMotor.class, "front left").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
        hardwareMap.get(DcMotor.class, "front right").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwareMap.get(DcMotor.class, "back left").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwareMap.get(DcMotor.class, "back right").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = 40*360/1.2;
        double err = -target;
        double speed = 0.8;
        double multiplier = 0.00017;
        double precision = 500;
        double Kp = 0.6;
        double Ki = 0.1;
        double Kd = 0.12;
        DcMotorEx enc = hardwareMap.get(DcMotorEx.class, "front left");
        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);
        while (Math.abs(enc.getCurrentPosition()-target) > precision && opModeIsActive()) {
            telemetry.addLine("In loop");
            telemetry.addData("Odo", enc.getCurrentPosition());
            telemetry.addLine(enc.getMode().name());
            err = enc.getCurrentPosition()-target;
            double sum = trend.stream().reduce(0d, Double::sum);
            double corr =
                    Kp * err + Ki * sum + Kd * (-trend.get(trend.size()-1)+err);
            telemetry.addData("Correction Raw", corr);
            
            corr *= multiplier;
            telemetry.addData("Correction", corr);
            telemetry.addData("D", Kd * (trend.get(trend.size()-1)-err));
            if (corr < 0 && corr > -0.1) {
                corr = -0.1;
            } else if (corr > 0 && corr < 0.1) {
                corr = 0.1;
            } else if (corr < 0 && corr < -speed) {
                corr = -speed;
            } else if (corr > 0 && corr > speed) {
                corr = speed;
            }
            telemetry.addLine("before speed");
            hardwareMap.get(DcMotor.class, "front left").setPower(corr);
            hardwareMap.get(DcMotor.class, "front right").setPower(-corr);
            hardwareMap.get(DcMotor.class, "back left").setPower(-corr);
            hardwareMap.get(DcMotor.class, "back right").setPower(corr);
            
            if (trend.size() <= 5) {
                trend.add(err);
            } else {
                trend.remove(0);
                trend.add(err);
            }
            telemetry.addLine("after trend");
            telemetry.update();
        }
        telemetry.addLine("end");
        telemetry.update();
        hardwareMap.get(DcMotor.class, "front left").setPower(0);
        hardwareMap.get(DcMotor.class, "front right").setPower(0);
        hardwareMap.get(DcMotor.class, "back left").setPower(0);
        hardwareMap.get(DcMotor.class, "back right").setPower(0);
        Thread.sleep(1000);
    }
}
