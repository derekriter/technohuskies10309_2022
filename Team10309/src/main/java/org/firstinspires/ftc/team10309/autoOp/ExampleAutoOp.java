package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Example AutoOp")
public class ExampleAutoOp extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, true);

        waitForStart();

        //running
        //>> PUT AUTO OP CODE HERE <<
//        robot.driveTiles(-1.25f, 0.5f);
        robot.strafeTiles(-2f, 0.5f);
//        robot.driveTiles(1.25f, 0.5f);
    }
}
