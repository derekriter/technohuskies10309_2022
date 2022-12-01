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

        telemetry.addLine("Init Completed");
        waitForStart();

        robot.driveOdo(10, 0.8f);
    }
}
