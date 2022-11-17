package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.Robot;

import java.util.ArrayList;
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
        double err = 0;
        ArrayList<Double> trend = new ArrayList<>();
        while (hardwareMap.get(DcMotorEx.class, "front left").getCurrentPosition() <= 10*360/1.2 && opModeIsActive()) {
            telemetry.addLine("In loop");
            telemetry.addData("Odo", hardwareMap.get(DcMotorEx.class, "front left").getCurrentPosition());
            telemetry.addLine(hardwareMap.get(DcMotor.class, "front left").getMode().name());
            telemetry.update();
            
            double precision = 500;
            double Kp = 0.5;
            double Ki = 1/200;
            double Kd = 1/50;
            double corr =
                    Kp * err + Ki * trend.stream().reduce(0d,
                            (Double a, Double b) -> {return a+b;}) + Kd * (trend.get(trend.size()-1)-err);
            hardwareMap.get(DcMotor.class, "front left").setPower(-0.2);
            hardwareMap.get(DcMotor.class, "front right").setPower(0.2);
            hardwareMap.get(DcMotor.class, "back left").setPower(0.2);
            hardwareMap.get(DcMotor.class, "back right").setPower(-0.2);
            
    
            double[] trendI = new double[trend.size() + 1];
            for (int i = 0; i < trend.size(); i++) trendI[i] = trend.get(i);
            trendI[trendI.length - 1] = err;
        }
        hardwareMap.get(DcMotor.class, "front left").setPower(0);
        hardwareMap.get(DcMotor.class, "front right").setPower(0);
        hardwareMap.get(DcMotor.class, "back left").setPower(0);
        hardwareMap.get(DcMotor.class, "back right").setPower(0);
    }
}
