package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Example AutoOp", group="Examples")
public class ExampleAutoOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        this.robot = new Robot(this);
    }

    @Override
    public void start() {
        this.robot.drive(3);
    }
    @Override
    public void loop() {
        this.robot.update();
    }
}
