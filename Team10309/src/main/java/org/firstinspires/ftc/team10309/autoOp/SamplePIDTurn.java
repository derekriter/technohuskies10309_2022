package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.Robot;

@TeleOp(name="Sample PID Turn")
public class SamplePIDTurn extends LinearOpMode {
    
    private Robot robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this, false);
        
        waitForStart();
        
        robot.turn(90, 1f);
    }
}
