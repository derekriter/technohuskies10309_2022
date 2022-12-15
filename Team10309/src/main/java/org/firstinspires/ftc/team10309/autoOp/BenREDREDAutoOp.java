package org.firstinspires.ftc.team10309.autoOp;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.team10309.API.ManipulatorController;
        import org.firstinspires.ftc.team10309.API.Robot;
        import org.firstinspires.ftc.team10309.API.SleeveDetect;

// Algin with a jenga block 1 jenga block from the right side edge of mat.
@Autonomous(name = "BenRedRed")
public class BenREDREDAutoOp extends LinearOpMode {

    private Robot robot;
    private ManipulatorController manipulatorController;

    @Override
    public void runOpMode() throws InterruptedException {

        final float liftSpeed = 0.9f;
        final float armPosLeft = 0.05f;
        final float armPosCenter = 0.4f;
        final float armPosRight = 0.75f;
        final float clawOpenPos = 0.4f;
        final float clawClosePos = 0.15f;
        //init
        this.robot = new Robot(this, true);
        this.manipulatorController = new ManipulatorController(this.robot.getHardware(), this);

        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        robot.getHardware().getClaw().setPosition(clawOpenPos);
        Thread.sleep(3000);
        robot.getHardware().getClaw().setPosition(clawClosePos);
        robot.initDetect();

        telemetry.addLine("Press Start");

        telemetry.update();
        waitForStart();
        SleeveDetect.SignalState state = robot.scanSleeve();
        telemetry.addData("Signal state", state.name());
        telemetry.update();

        //go to low junction
        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.LOW, false);
        this.robot.strafeTilesOdo(0.7f, 0.5f, -0.5f);
        this.robot.driveOdo(-1.5f, 0.5f);
        Thread.sleep(200);

        //drop cone on low junction
        manipulatorController.setLiftPosition(this.robot.getHardware().getLift().getCurrentPosition() + 500);
        manipulatorController.setClaw(ManipulatorController.ClawPosition.OPEN);
        manipulatorController.setLiftPosition(this.robot.getHardware().getLift().getCurrentPosition() - 500);

        //go to cone stack
        this.robot.driveOdo(1.5f, 0.5f);
        this.robot.strafeTilesOdo(1.5f, 0.5f, -3.5f);
        manipulatorController.setArmPosition(ManipulatorController.ArmRotation.FRONT);

        //pick up cone
        manipulatorController.setLiftPosition(-1256);
        this.robot.driveTilesOdo(1f, 0.5f);
        manipulatorController.setClaw(ManipulatorController.ClawPosition.CLOSED);
        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.LOW);
        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.HIGH, false);

        //go to tall junction
        this.robot.driveOdo(-6, 0.2f);
        this.robot.turn(45f, .2 );
        this.robot.turn(45f, .2);
        this.robot.strafeTilesOdo(1.5f, 0.5f);
        this.robot.driveOdo(3f, 0.5f);

        // drop onto high junction
        manipulatorController.setClaw(ManipulatorController.ClawPosition.OPEN);
        this.robot.driveOdo(-3f, 0.5f);

        // detection stuff
        if (state == SleeveDetect.SignalState.RED_SQUARE) {
            this.robot.strafeTilesOdo(-1.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        } else if (state == SleeveDetect.SignalState.GREEN_CIRCLE) {
            this.robot.strafeTilesOdo(-0.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        } else if (state == SleeveDetect.SignalState.BLUE_TRIANGLE) {
            this.robot.strafeTilesOdo(0.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        } else {
            this.robot.strafeTilesOdo(-0.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);

        }
    }
}