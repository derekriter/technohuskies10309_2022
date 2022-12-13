    package org.firstinspires.ftc.team10309.autoOp;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    import org.firstinspires.ftc.team10309.API.ManipulatorController;
    import org.firstinspires.ftc.team10309.API.Robot;
    import org.firstinspires.ftc.team10309.API.SleeveDetect;

    @Autonomous(name="Purple PARK Auto Op", group="Examples")
    public class PURPLEAutoOpPark extends LinearOpMode {

        private Robot robot;
        private ManipulatorController manipulatorController;

        @Override
        public void runOpMode() throws InterruptedException {
            //init
            this.robot = new Robot(this, true);
            this.manipulatorController = new ManipulatorController(this.robot.getHardware(), this);

            robot.initDetect();
            telemetry.addLine("Press Start");
            telemetry.update();
    
            waitForStart();
            SleeveDetect.SignalState state = robot.scanSleeve();
            telemetry.addData("Signal state", state.name());
            telemetry.update();
    
//
//            SleeveDetect.SignalState state = robot.scanSleeve();
//            telemetry.addData("Signal state", state.name());
//            telemetry.update();
//
//            Thread.sleep(1000);
//            robot.turn(-90,1f); // FIx,  Plus move out before....
//            robot.strafe(21,.7f);
//            robot.drive(32,.7f); // a bit more?
//            robot.drive(2,.7f);
//            clawController.setClawState(ClawController.ClawState.OPEN);
//            robot.strafe(26,.7f);
            
            
            if (state == SleeveDetect.SignalState.RED_SQUARE) {
                robot.strafeTiles(1, 0.5f, 6.5f);
                robot.strafe(-3f, 0.5f);
                robot.driveTiles(1, 0.5f, 3f);
            } else if (state == SleeveDetect.SignalState.GREEN_CIRCLE) {
                robot.strafeTiles(1, 0.5f);
                // done
            } else {
                robot.strafeTiles(1, 0.5f, 6.5f);
                robot.strafe(-3f, 0.5f);
                robot.driveTiles(-1, 0.5f, 4f);
            }
        }
    }