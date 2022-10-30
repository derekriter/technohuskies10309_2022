package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Testing AutoOp")
public class TestingAutoOp extends LinearOpMode {

    private Robot robot;
    private ClawController clawController;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, false);
        this.clawController = new ClawController(this.robot.getHardware(), this);

        waitForStart();

        //main program

    }
}
