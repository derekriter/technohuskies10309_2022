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
        else if(position == LiftPosition.LOW) this.hardware.getLift().setTargetPosition(-6200);
        else if(position == LiftPosition.MIDDLE) this.hardware.getLift().setTargetPosition(-10300);
        else this.hardware.getLift().setTargetPosition(RobotInfo.liftTop);

        this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getLift().setPower(0.45);

        while(this.hardware.getLift().isBusy() && this.opMode.opModeIsActive()) {}
    }
    public void setClawRotation(ClawRotation rotation) {
        float targetPosition;
        if(rotation == ClawRotation.FRONT) targetPosition = 0.05f;
        else if(rotation == ClawRotation.SIDE) targetPosition = 0.4f;
        else targetPosition = 0.75f;

        while(this.hardware.getClawRotator().getPosition() != targetPosition) {
            this.hardware.getClawRotator().setPosition(targetPosition);
        }
    }
    public void setClawState(ClawState state) {
        float targetPosition;
        if(state == ClawState.OPEN) targetPosition = 0.4f;
        else targetPosition = 0.15f;

        while(this.hardware.getClawRotator().getPosition() != targetPosition) {
            this.hardware.getClawRotator().setPosition(targetPosition);
        }
    }
}
