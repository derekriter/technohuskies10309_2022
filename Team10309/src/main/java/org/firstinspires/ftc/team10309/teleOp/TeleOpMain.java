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
        //diameter = 34.4mm
        // -11781 top ticks


        @Override
        public void init() {
            this.hardware = new RobotHardware(this, true); //MEANT TO BE TRUE!!!
            this.hardware.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hardware.getLift().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardware.getClaw().setDirection(Servo.Direction.REVERSE);
        }

        /**
         * No longe lifting before/after rotate of arm... it's on the "driver"
         */
//        private void prepareLiftForRotate(int position, double speed){
//            int adjustLiftForArmRotate = -2800;
//            this.hardware.getLift().setTargetPosition(position + adjustLiftForArmRotate);
//            this.hardware.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            this.hardware.getLift().setPower(-speed);
//        }

        @Override
        public void loop() {

            boolean Raise = this.gamepad2.y;
            boolean Lower = this.gamepad2.a;
            boolean ArmPosR = gamepad2.dpad_right;
            boolean ArmPosC = gamepad2.dpad_up;
            boolean ArmPosL = gamepad2.dpad_left;
            boolean  grab = gamepad2.right_bumper;
            boolean release = gamepad2.left_bumper;
            double liftPos = this.hardware.getLift().getCurrentPosition();
            double needForLiftSpeed = 0.45;

            telemetry.addData("Lift Start Pos", liftPos);
            telemetry.addData("Claw Position", this.hardware.getClaw().getPosition());
            telemetry.update();

            // Arm
            /**
             * Leaving just incase we DO want to go to the center
             *
             * if (ArmPosC) {
                while(this.hardware.getClawRotater().getPosition() != 0.4) {
                    this.hardware.getClawRotater().setPosition(.4);
                }
            }
             **/
            if (ArmPosL) {
                double leftDestinationPos = 0.05;
                while(this.hardware.getClawRotater().getPosition() != leftDestinationPos) {
                    this.hardware.getClawRotater().setPosition(leftDestinationPos);
                }

                //Potential Slower Arm Code
//                double armPos = this.hardware.getClawRotater().getPosition();
//                double increment = .1;
//                while(this.hardware.getClawRotater().getPosition() != leftDestinationPos){
//                    if(armPos > leftDestinationPos){
//                        armPos -= increment;
//                        this.hardware.getClawRotater().setPosition(armPos);
//                    } else {
//                        this.hardware.getClawRotater().setPosition(leftDestinationPos);
//                    }
//                }
            }
            if (ArmPosR) {
                double rightDestinationPos = .75;
                while(this.hardware.getClawRotater().getPosition() != rightDestinationPos) {
                    this.hardware.getClawRotater().setPosition(rightDestinationPos);
                }

                //Potential Slower Arm Code,
                // flaw... if you're moving the arm and try to go the other direction
                // you could go past the destintation and hit the hub
//                double armPos = this.hardware.getClawRotater().getPosition();
//                double increment = .1;
//                while(this.hardware.getClawRotater().getPosition() != rightDestinationPos){
//                    if(armPos < rightDestinationPos){
//                        armPos += increment;
//                        this.hardware.getClawRotater().setPosition(armPos);
//                    } else {
//                        this.hardware.getClawRotater().setPosition(rightDestinationPos);
//                    }
//                }
            }
            //End Arm

            //Claw
            if (grab) {
                this.hardware.getClaw().setPosition(0.15);
            }
            if (release) {
                this.hardware.getClaw().setPosition(0.4);
            }
            //End Claw

            // elevator
            if(Raise){
                this.hardware.getLift().setPower(-needForLiftSpeed);
            } else if(Lower){
                this.hardware.getLift().setPower(needForLiftSpeed);
            } else {
                this.hardware.getLift().setPower(0);
            }
            //End elevator

            // Drive Code
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

            //END Drive Code

            telemetry.addData("Strafe", strafe);
            telemetry.addData("forward", forward);
            telemetry.addData("turn", turn);
            telemetry.addData("Liftpos", liftPos);
            telemetry.update();
        }
    }
