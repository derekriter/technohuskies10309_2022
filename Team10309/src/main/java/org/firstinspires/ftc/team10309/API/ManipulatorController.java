package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.info.RobotInfo;

public class ManipulatorController {

    public enum LiftPosition {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }
    public enum ArmRotation {
        FRONT,
        SIDE,
        BACK
    }
    public enum ClawPosition {
        OPEN,
        CLOSED
    }

    private final RobotHardware hardware;
    private final LinearOpMode opMode;

    private final int liftGoundPos = 0;
    private final int liftLowPos = -4133;
    private final int liftMiddlePos = -6866;
    private final int liftTopPos = RobotInfo.liftTop;

    private final float clawRotatorFrontPos = 0.05f;
    private final float clawRotatorSidePos = 0.4f;
    private final float clawRotatorBackPos = 0.75f;

    private final float clawOpenPos = 0.4f;
    private final float clawClosedPos = 0.15f;

    public ManipulatorController(RobotHardware pHardware, LinearOpMode opMode) {
        this.hardware = pHardware;
        this.opMode = opMode;
        this.hardware.resetLift();
    }

    public void setLiftPosition(LiftPosition position) {
        setLiftPosition(position, true);
    }
    public void setLiftPosition(LiftPosition position, boolean waitForLift) {
        if(position == LiftPosition.GROUND) this.hardware.getLift().setTargetPosition(liftGoundPos);
        else if(position == LiftPosition.LOW) this.hardware.getLift().setTargetPosition(liftLowPos);
        else if(position == LiftPosition.MIDDLE) this.hardware.getLift().setTargetPosition(liftMiddlePos);
        else this.hardware.getLift().setTargetPosition(liftTopPos);

        this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getLift().setPower(1);

        if(waitForLift) waitForLift();
    }
    public void setLiftPosition(int position) {
        setLiftPosition(position, true);
    }
    public void setLiftPosition(int position, boolean waitForLift) {
        int clampedPos = Math.max(Math.min(position, 0), liftTopPos);
        this.hardware.getLift().setTargetPosition(clampedPos);

        this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getLift().setPower(1);

        if(waitForLift) waitForLift();
    }

    public void setArmPosition(ArmRotation rotation) {
        float targetPosition;
        if(rotation == ArmRotation.FRONT) targetPosition = clawRotatorFrontPos;
        else if(rotation == ArmRotation.SIDE) targetPosition = clawRotatorSidePos;
        else targetPosition = clawRotatorBackPos;

        this.hardware.getClawRotator().setPosition(targetPosition);
    }
    public void setArmPosition(float rotation) {
        float clampedRotation = Math.max(Math.min(rotation, clawRotatorBackPos),
                clawRotatorFrontPos);

        this.hardware.getClawRotator().setPosition(clampedRotation);
    }

    public void setClaw(ClawPosition state) {
        float targetPosition;
        if(state == ClawPosition.OPEN) targetPosition = clawOpenPos;
        else targetPosition = clawClosedPos;

        this.hardware.getClaw().setPosition(targetPosition);
    }
    public void setClaw(float pos) {
        float clampedPos = Math.max(Math.min(pos, clawOpenPos), clawClosedPos);

        this.hardware.getClaw().setPosition(clampedPos);
    }

    public void waitForLift() {
        while(this.hardware.getLift().isBusy() && this.opMode.opModeIsActive());
    }
}
