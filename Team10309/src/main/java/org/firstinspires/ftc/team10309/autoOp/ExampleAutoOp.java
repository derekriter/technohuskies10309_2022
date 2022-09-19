package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Example AutoOp", group="Examples")
public class ExampleAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Successfully Initialized", "%s");
        telemetry.update();

        waitForStart();

        telemetry.addData("Successfully Ran", "%s");
        telemetry.update();

        sleep(1000);
    }
}
