package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Test turn PID")
public class TestTurnPID extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);

        telemetry.addLine("Init Completed");
        telemetry.update();
        waitForStart();
//        robot.turn(10f, 0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("10 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(-10f, 0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("-10 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
        robot.turn(45f,  0.2);
        Thread.sleep(1000);
        telemetry.addLine("45 Completed");
        telemetry.update();
        Thread.sleep(1000);
        robot.turn(-45f,  0.2);
        Thread.sleep(1000);
        telemetry.addLine("-45 Completed");
        telemetry.update();
        Thread.sleep(1000);
        robot.turn(60f,  0.2);
        Thread.sleep(1000);
        telemetry.addLine("60 Completed");
        telemetry.update();
        Thread.sleep(1000);
        robot.turn(-60f,  0.2);
        Thread.sleep(1000);
        telemetry.addLine("-60 Completed");
        telemetry.update();
        Thread.sleep(1000);
//        robot.turn(75f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("75 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(-75f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("-75 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(80f, 0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("80 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(-80f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("-80 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(85f, 0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("85 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(-85f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("-85 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
        robot.turn(89.8f, 0.2);
        Thread.sleep(1000);
        telemetry.addLine("95 Completed");
        telemetry.update();
        Thread.sleep(1000);
        robot.turn(-89.8f,  0.2);
        Thread.sleep(1000);
        telemetry.addLine("-95 Completed");
        telemetry.update();
        Thread.sleep(1000);
//        robot.turn(120f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("120 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(-120f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("-120 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(150f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("150 Completed");
//        telemetry.update();
//        Thread.sleep(1000);
//        robot.turn(-150f,  0.2);
//        Thread.sleep(1000);
//        telemetry.addLine("-150 Completed");
//        telemetry.update();
        Thread.sleep(1000);
    }
}
