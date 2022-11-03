    package org.firstinspires.ftc.team10309.autoOp.master;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    import org.firstinspires.ftc.team10309.API.ClawController;
    import org.firstinspires.ftc.team10309.API.Robot;
    import org.firstinspires.ftc.team10309.API.SleeveDetect;

    @Autonomous(name="RED CONE Auto Op | FINAL", group="Examples")
    public class REDAutoOpCone extends LinearOpMode {

        private Robot robot;
        private ClawController clawController;

        @Override
        public void runOpMode() throws InterruptedException {
            //init
            this.robot = new Robot(this, true);
            this.clawController = new ClawController(this.robot.getHardware(), this);

            robot.initDetect();
            SleeveDetect.SignalState state = robot.scanSleeve();
            telemetry.addLine("Press Start");
            telemetry.update();
            telemetry.addData("Signal state", state.name());
            telemetry.update();
    
            waitForStart();
    
            final float liftSpeed = 0.9f;
            final float armPosLeft = 0.05f;
            final float armPosCenter = 0.4f;
            final float armPosRight = 0.75f;
            final float clawOpenPos = 0.4f;
            final float clawClosePos = 0.15f;
            // constants, copied from TeleOpMain.
    
//
//            SleeveDetect.SignalState state = robot.scanSleeve();
//            telemetry.addData("Signal state", state.name());
//            telemetry.update();
//
//            Thread.sleep(1000);
//            robot.turn(-90,.05f,1f); // FIx,  Plus move out before....
//            robot.strafe(21,.7f);
//            robot.drive(32,.7f); // a bit more?
//            robot.drive(2,.7f);
//            clawController.setClawState(ClawController.ClawState.OPEN);
//            robot.strafe(26,.7f);
            robot.strafeTiles(1, 0.5f, 3f);
            robot.strafe(-3, 0.5f);
            robot.driveTiles(1, 0.5f);
            robot.strafeTiles(1, 0.5f);
            robot.getHardware().getLift().setTargetPosition(-1100);// Lowest elevator height for
            // moving claw rotator
            robot.getHardware().getLift().setPower(liftSpeed);
            clawController.setClawState(ClawController.ClawState.OPEN);
            robot.getHardware().getClawRotator().setPosition(armPosLeft);
            
        }
    }