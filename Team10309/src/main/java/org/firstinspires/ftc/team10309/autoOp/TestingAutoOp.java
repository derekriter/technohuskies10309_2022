package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Testing AutoOp")
public class TestingAutoOp extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this);

        waitForStart();
        this.robot.startUpdateLoop();

        //main program
        this.robot.strafe(12, 0.5f);

        //exiting
        this.robot.exitUpdateLoop();
    }
}
