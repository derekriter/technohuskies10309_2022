package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Test turn original PID")
public class TestOriginalTurnPID extends LinearOpMode {
    
    private Robot robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);
        
        telemetry.addLine("Init Completed");
        telemetry.update();
        waitForStart();
        
        robot.turn(10f, 0.2f);
        Thread.sleep(2000);
        telemetry.addLine("20 Completed");
        telemetry.update();
        Thread.sleep(2000);
        robot.turn(-10f, 0.2f);
        Thread.sleep(2000);
        telemetry.addLine("-20 Completed");
        telemetry.update();
        Thread.sleep(2000);
//            robot.turn(40f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("40 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(-40f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("-40 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(60f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("60 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(-60f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("-60 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(75f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("75 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(-75f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("-75 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
        robot.turn(91f, 0.2f);
        Thread.sleep(2000);
        telemetry.addLine("90 Completed");
        telemetry.update();
        Thread.sleep(2000);
        robot.turn(-91f,  0.2f);
        Thread.sleep(2000);
        telemetry.addLine("-90 Completed");
        telemetry.update();
        Thread.sleep(2000);
//            robot.turn(120f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("120 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(-120f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("-120 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(150f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("150 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
//            robot.turn(-150f,  0.2);
//            Thread.sleep(2000);
//            telemetry.addLine("-150 Completed");
//            telemetry.update();
//            Thread.sleep(2000);
    }
}
