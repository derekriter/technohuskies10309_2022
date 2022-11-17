package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Test Odo Move1")
public class TestOdoMovePID extends LinearOpMode {
    
    private Robot robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);
        
        waitForStart();
        
        robot.driveOdo(10, 0.008, new double[]{-10, -10});
//        hardwareMap.get(DcMotor.class, "front left").setPower(0);
//        hardwareMap.get(DcMotor.class, "front right").setPower(0);
//        hardwareMap.get(DcMotor.class, "back left").setPower(0);
//        hardwareMap.get(DcMotor.class, "back right").setPower(0);
    }
}
