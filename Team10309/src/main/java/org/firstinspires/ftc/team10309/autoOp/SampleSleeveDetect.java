package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.Robot;

@TeleOp
public class SampleSleeveDetect extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false);
        robot.initDetect();
        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();
        telemetry.addLine(robot.scanSleeve().name());
        telemetry.update();
        Thread.sleep(1000);
    }
}
