package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Test Odo Move Drive")
public class TestOdoMovePIDDrive extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);

        telemetry.addLine("Init Completed");
        telemetry.update();
        waitForStart();

        robot.driveOdo(6.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(-6.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(12.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(-12.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(24.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(-24.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(36.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(-36.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(72.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(-72.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(80.0f, 0.6f);
        Thread.sleep(5000);
        robot.driveOdo(-80.0f, 0.6f);
        Thread.sleep(5000);
    }
}
