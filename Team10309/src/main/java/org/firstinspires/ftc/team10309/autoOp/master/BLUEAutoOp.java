    package org.firstinspires.ftc.team10309.autoOp.master;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    import org.firstinspires.ftc.team10309.API.ClawController;
    import org.firstinspires.ftc.team10309.API.Robot;

    @Autonomous(name="BLUE Auto Op | FINAL", group="Examples")
    public class BLUEAutoOp extends LinearOpMode {

        private Robot robot;
        private ClawController clawController;

        @Override
        public void runOpMode() throws InterruptedException {
            //init
            this.robot = new Robot(this, true);
            this.clawController = new ClawController(this.robot.getHardware(), this);

            waitForStart();

            robot.initDetect();
            telemetry.addLine("Press Start");
            telemetry.update();

            waitForStart();

            telemetry.addLine(robot.scanSleeve().name());
            telemetry.update();
            Thread.sleep(1000);
            robot.strafe(4,.7f);
            robot.turn(90,.05f,1f);
            robot.strafe(-21,.7f);
            robot.drive(-32,.7f);
            robot .drive(-2,.7f);
            clawController.setClawState(ClawController.ClawState.OPEN);
            robot.strafe(-26,.7f);
        }
    }
