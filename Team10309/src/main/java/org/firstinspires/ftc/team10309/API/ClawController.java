package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ClawController {

    public enum LiftPosition {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }
    public enum ClawRotation {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public enum ClawState {
        OPEN,
        CLOSED
    }

    private RobotHardware hardware;

    public ClawController(RobotHardware hardware) {
        this.hardware = hardware;
        this.hardware.resetLift();
    }

    public void setLiftPosition(LiftPosition position) {
        if(position == LiftPosition.GROUND) this.hardware.getLift().setTargetPosition(0);
        else if(position == LiftPosition.LOW) this.hardware.getLift().setTargetPosition(-4333);
        else if(position == LiftPosition.MIDDLE) this.hardware.getLift().setTargetPosition(-8666);
        else this.hardware.getLift().setTargetPosition(-13000);

        this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getLift().setPower(1);
    }
    public void setClawRotation(ClawRotation rotation) {
        if(rotation == ClawRotation.LEFT) this.hardware.getClawRotater().setPosition(0);
        if(rotation == ClawRotation.MIDDLE) this.hardware.getClawRotater().setPosition(0.5);
        if(rotation == ClawRotation.RIGHT) this.hardware.getClawRotater().setPosition(1);
    }
    public void setClawState(ClawState state) {
        if(state == ClawState.OPEN) this.hardware.getClaw().setPosition(0.4);
        if(state == ClawState.CLOSED) this.hardware.getClaw().setPosition(0.15);
    }
}
