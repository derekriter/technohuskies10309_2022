package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team10309.API.RobotHardware;

    @TeleOp(name="TeleOpMain", group="Test")
    public class TeleOpMain extends OpMode {

        public RobotHardware hardware;

        @Override
        public void init() {
            this.hardware = new RobotHardware(this, true); //MEANT TO BE TRUE!!!
            this.hardware.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hardware.getLift().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardware.getClaw().setDirection(Servo.Direction.REVERSE);
        }

        private void prepareLiftForRotate(int position, double speed){
            int adjustLiftForArmRotate = -200;
            this.hardware.getLift().setTargetPosition(-position + adjustLiftForArmRotate);
            this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hardware.getLift().setPower(-speed);
        }

        @Override
        public void loop() {

            boolean Raise = this.gamepad2.y;
            boolean Lower = this.gamepad2.a;
            double Liftpos = this.hardware.getLift().getCurrentPosition();
            boolean ArmPosR = gamepad2.dpad_right;
            boolean ArmPosC = gamepad2.dpad_up;
            boolean ArmPosL = gamepad2.dpad_left;
            boolean  grab = gamepad2.right_bumper;
            boolean release = gamepad2.left_bumper;
            double liftPos = this.hardware.getLift().getCurrentPosition();
            double needForLiftSpeed = 0.45;

            telemetry.addData("liftpos", liftPos);
            telemetry.addData("Claw Position", this.hardware.getClaw().getPosition());
            telemetry.update();

            // Arm
            int currentPosition = this.hardware.getLift().getCurrentPosition();
            if (ArmPosC) {
                prepareLiftForRotate(currentPosition, needForLiftSpeed);

                while(this.hardware.getLift().isBusy()) {}

                while(this.hardware.getClawRotater().getPosition() != 0.4) {
                    this.hardware.getClawRotater().setPosition(.4);
                }

                this.hardware.getLift().setTargetPosition(currentPosition);
                this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.hardware.getLift().setPower(needForLiftSpeed);
            }
            if (ArmPosL) {
                prepareLiftForRotate(currentPosition, needForLiftSpeed);

                while(this.hardware.getLift().isBusy()) {}

                while(this.hardware.getClawRotater().getPosition() != 0.05) {
                    this.hardware.getClawRotater().setPosition(0.05);
                }
                this.hardware.getLift().setTargetPosition(currentPosition);
                this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.hardware.getLift().setPower(needForLiftSpeed);
            }
            if (ArmPosR) {
                prepareLiftForRotate(currentPosition, needForLiftSpeed);

                while(this.hardware.getLift().isBusy()) {}

                while(this.hardware.getClawRotater().getPosition() != .8) {
                    this.hardware.getClawRotater().setPosition(.8);
                }

                this.hardware.getLift().setTargetPosition(currentPosition);
                this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.hardware.getLift().setPower(needForLiftSpeed);
            }

            //Claw
            if (grab) {
                this.hardware.getClaw().setPosition(0.15);
            }
            if (release) {
                this.hardware.getClaw().setPosition(0.4);
            }

            // elevator
            if(Raise){
                this.hardware.getLift().setPower(-needForLiftSpeed);
            } else if(Lower){
                this.hardware.getLift().setPower(needForLiftSpeed);
            } else {
                this.hardware.getLift().setPower(0);
            }

            // Drive Code

            telemetry.addData("Liftpos", Liftpos);
            telemetry.update();

            double strafe = this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = this.gamepad1.right_stick_x;

            double FLSpeed = forward + strafe + turn;
            double FRSpeed = forward - strafe - turn;
            double BRSpeed = forward + strafe - turn;
            double BLSpeed = forward - strafe + turn;

            double largest = 1.0;
            largest=Math.max(largest, Math.abs(FLSpeed));
            largest=Math.max(largest, Math.abs(FRSpeed));
            largest=Math.max(largest, Math.abs(BLSpeed));
            largest=Math.max(largest, Math.abs(BRSpeed));

            this.hardware.getFLMotor().setPower(FLSpeed);
            this.hardware.getBRMotor().setPower(BRSpeed);
            this.hardware.getFRMotor().setPower(FRSpeed);
            this.hardware.getBLMotor().setPower(BLSpeed);

            telemetry.addData("Strafe", strafe);
            telemetry.addData("forward", forward);
            telemetry.addData("turn", turn);
            telemetry.update();
        }
    }
