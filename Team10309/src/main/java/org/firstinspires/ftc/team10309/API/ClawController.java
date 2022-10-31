package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.info.RobotInfo;

public class ClawController {

    public enum LiftPosition {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }
    public enum ClawRotation {
        FRONT,
        SIDE,
        BACK
    }
    public enum ClawState {
        OPEN,
        CLOSED
    }

    private RobotHardware hardware;
    private LinearOpMode opMode;

    public ClawController(RobotHardware hardware, LinearOpMode opMode) {
        this.hardware = hardware;
        this.opMode = opMode;
        this.hardware.resetLift();
    }

    public void setLiftPosition(LiftPosition position) {
        if(position == LiftPosition.GROUND) this.hardware.getLift().setTargetPosition(0);
        else if(position == LiftPosition.LOW) this.hardware.getLift().setTargetPosition(RobotInfo.liftTop / 3);
        else if(position == LiftPosition.MIDDLE) this.hardware.getLift().setTargetPosition(RobotInfo.liftTop / 3 * 2);
        else this.hardware.getLift().setTargetPosition(RobotInfo.liftTop);

        this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getLift().setPower(0.45);

        while(this.hardware.getLift().isBusy() && this.opMode.opModeIsActive()) {}
    }
    public void setClawRotation(ClawRotation rotation) {
        if(rotation == ClawRotation.FRONT) this.hardware.getClawRotator().setPosition(0.05);
        if(rotation == ClawRotation.SIDE) this.hardware.getClawRotator().setPosition(0.4);
        if(rotation == ClawRotation.BACK) this.hardware.getClawRotator().setPosition(0.75);
    }
    public void setClawState(ClawState state) {
        if(state == ClawState.OPEN) this.hardware.getClaw().setPosition(0.4);
        if(state == ClawState.CLOSED) this.hardware.getClaw().setPosition(0.15);
    }
}
