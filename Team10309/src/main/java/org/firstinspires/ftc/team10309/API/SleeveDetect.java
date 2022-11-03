package org.firstinspires.ftc.team10309.API;/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
public class SleeveDetect {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private final Telemetry telemetry;
    private WebcamName camera;
    private RobotHardware rh;
    public SleeveDetect(Telemetry telemetry, WebcamName camera, RobotHardware rh) {
        this.telemetry = telemetry;
        this.camera = camera;
        this.rh = rh;
    }
    public void init() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
    }
    public SignalState scan() {

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        } else {
            telemetry.addLine("tfod doesn't exist..?");
            telemetry.update();
        }
        if (tfod != null) {
            int counter = 0;
            while (tfod != null && counter != 20) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() > 1) {
                    telemetry.addLine("More than one " +
                            "recogn.");
                    telemetry.update();
                    return SignalState.ERROR;
                } else if (updatedRecognitions.size() < 1) {
                    telemetry.addLine("no recogn.");
                    telemetry.update();
                    return SignalState.ERROR;
                }
                
                Recognition recognition = updatedRecognitions.get(0);
                double col = (recognition.getLeft() + recognition.getRight()) / 2;
                double row = (recognition.getTop() + recognition.getBottom()) / 2;
                double width = Math.abs(recognition.getRight() - recognition.getLeft());
                double height = Math.abs(recognition.getTop() - recognition.getBottom());
                
                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                if (recognition.getLabel() == "Blue Triangle") {
                    return SignalState.BLUE_TRIANGLE;
                } else if (recognition.getLabel() == "Red Square") {
                    return SignalState.RED_SQUARE;
                } else if (recognition.getLabel() == "Green Circle") {
                    return SignalState.GREEN_CIRCLE;
                } else {
                    counter ++;
                    continue;
                }
            }
            telemetry.update();
            tfod.deactivate();
        }
        } else {
            return SignalState.ERROR;
        }
        return SignalState.ERROR;
    }
    
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = RobotInfo.VUFORIA_KEY;
        parameters.cameraName = camera;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    
    private void initTfod() {
        int tfodMonitorViewId = rh.getTfodMonitorViewID();
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromFile(RobotInfo.pathToModel, RobotInfo.model_labels);
    }
    
    public enum SignalState {
        BLUE_TRIANGLE,
        GREEN_CIRCLE,
        RED_SQUARE,
        ERROR
    }
}
