package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Test Odo Move Strafe")
public class TestOdoMovePIDStrafe extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);

        telemetry.addLine("Init Completed");
        telemetry.update();
        waitForStart();
    
        robot.strafeOdo(36f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(-36f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(24f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(-24f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(20f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(-20f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(6f, 0.6f);
        Thread.sleep(2000);
        robot.strafeOdo(-6f, 0.6f);
        Thread.sleep(2000);
    }
}
