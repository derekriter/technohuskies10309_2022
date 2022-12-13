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

        robot.strafeOdo(60, 0.6f*1.25f);
    }
}
