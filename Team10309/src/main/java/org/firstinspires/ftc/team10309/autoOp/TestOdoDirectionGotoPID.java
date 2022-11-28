package org.firstinspires.ftc.team10309.autoOp;

// CHANGE WHILE LOOP LOGIC; MAKE SURE ROBOT GOES BACKWARDS IF IT GOES PAST THE TARGET.
//

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team10309.API.Robot;

import java.util.ArrayList;

@Autonomous(name="Test Odo Correction Goto Move PID")
public class TestOdoDirectionGotoPID extends LinearOpMode {
    
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
    
        double speed = 0.8;
        double target = 40*360/1.2;
        
        double err = -target;
        double multiplier = 0.00017;
        double precision = 500;
        double Kp = 0.6;
        double Ki = 0.1;
        double Kd = 0.12;
        DcMotorEx enc = hardwareMap.get(DcMotorEx.class, "front left");
        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);
        
        Orientation angles =
                this.hardwareMap.get(BNO055IMU.class, "imu").getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        
        double angle = angles.thirdAngle;
        double a_err = 0 - angle;
        double a_multiplier = 0.00005;
        double a_precision = 1;
        double a_Kp = 0.5;
        double a_Ki = 0.1;
        double a_Kd = 0.12;
        double a_speed = 0;
        
        ArrayList<Double> a_trend = new ArrayList<>();
        a_trend.add(a_err);
        while (Math.abs(enc.getCurrentPosition()-target) > precision && opModeIsActive()) {
            angles =
                    this.hardwareMap.get(BNO055IMU.class, "imu").getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.XYZ,
                            AngleUnit.DEGREES);
            err = enc.getCurrentPosition()-target;
            a_err = 0-angles.thirdAngle;
            telemetry.addData("Angle Error", a_err);
            double sum = trend.stream().reduce(0d, Double::sum);
            double a_sum = a_trend.stream().reduce(0d, Double::sum);
            double corr =
                    Kp * err + Ki * sum + Kd * (-trend.get(trend.size()-1)+err);
            a_speed = 0.4 * corr;
            double turnpid =
                    a_Kp * a_err + a_Ki * a_sum + a_Kd * (-a_trend.get(a_trend.size()-1)+a_err);
            
            
            corr *= multiplier;
            turnpid *= a_multiplier;
            if (corr < 0 && corr > -0.1) {
                corr = -0.1;
            } else if (corr > 0 && corr < 0.1) {
                corr = 0.1;
            } else if (corr < 0 && corr < -speed) {
                corr = -speed;
            } else if (corr > 0 && corr > speed) {
                corr = speed;
            }
            
            // lower limit: Math.abs(corr*0.2)
            // upper limti: math.abs(corr*0.4)
            if (turnpid < 0 && turnpid > -0.1) {
                turnpid = -Math.abs(corr*0.2);
            } else if (turnpid > 0 && turnpid < 0.1) {
                turnpid = Math.abs(corr*0.2);
            } else if (turnpid < 0 && turnpid < -a_speed) {
                turnpid = -a_speed;
            } else if (turnpid > 0 && turnpid > a_speed) {
                turnpid = a_speed;
            }
            telemetry.addData("Angle Correction", turnpid);
            if (Math.abs(a_err) < a_precision) {
                hardwareMap.get(DcMotor.class, "front left").setPower(corr);
                hardwareMap.get(DcMotor.class, "front right").setPower(-corr);
                hardwareMap.get(DcMotor.class, "back left").setPower(-corr);
                hardwareMap.get(DcMotor.class, "back right").setPower(corr);
            } else {
                if (err > 0) {
                    hardwareMap.get(DcMotor.class, "front left").setPower(corr-turnpid);
                    hardwareMap.get(DcMotor.class, "front right").setPower(-corr-turnpid);
                    hardwareMap.get(DcMotor.class, "back left").setPower(-corr+turnpid);
                    hardwareMap.get(DcMotor.class, "back right").setPower(corr+turnpid);
                }
                hardwareMap.get(DcMotor.class, "front left").setPower(corr+turnpid);
                hardwareMap.get(DcMotor.class, "front right").setPower(-corr+turnpid);
                hardwareMap.get(DcMotor.class, "back left").setPower(-corr-turnpid);
                hardwareMap.get(DcMotor.class, "back right").setPower(corr-turnpid);
            }
            
            if (trend.size() <= 5) {
                trend.add(err);
            } else {
                trend.remove(0);
                trend.add(err);
            }
            if (a_trend.size() <= 5) {
                trend.add(a_err);
            } else {
                trend.remove(0);
                trend.add(a_err);
            }
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
