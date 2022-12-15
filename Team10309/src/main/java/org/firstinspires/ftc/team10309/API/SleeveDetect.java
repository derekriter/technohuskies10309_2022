package org.firstinspires.ftc.team10309.API;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import java.util.List;

public class SleeveDetect {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry telemetry;
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
    public SignalState scan() throws InterruptedException {
        telemetry.addLine("In scan");
        telemetry.addLine("Tfod?" + (tfod == null));
        telemetry.update();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (tfod != null) {
            telemetry.addLine("Zoom...");
            telemetry.update();
            tfod.activate();
            tfod.setZoom(2, 16.0 / 9.0);
        } else {
            telemetry.addLine("tfod doesn't exist..?");
            telemetry.update();
        }
        telemetry.addLine("Done with zooom");
        telemetry.update();
        Thread.sleep(500);
        if (tfod != null) {
            int counter = 0;
            int ii = 0;
            while (tfod != null && ii < 5) {
            telemetry.addLine("In detection loop.");
            telemetry.update();
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() > 1) {
                    telemetry.addLine("More than one " +
                            "recogn.");
                    Thread.sleep(500);
                    telemetry.update();
                } else if (updatedRecognitions.size() < 1) {
                    telemetry.addLine("no recogn.");
                    telemetry.update();
                    Thread.sleep(500);
                }
                else {
                    Recognition recognition = updatedRecognitions.get(0);
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());
    
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    telemetry.update();
                    Thread.sleep(500);
                    if (recognition.getLabel() == "Blue Triangle") {
                        return SignalState.BLUE_TRIANGLE;
                    } else if (recognition.getLabel() == "Red Square") {
                        return SignalState.RED_SQUARE;
                    } else if (recognition.getLabel() == "Green Circle") {
                        return SignalState.GREEN_CIRCLE;
                    } else {
                        counter++;
                    }
                }
            }
            ii++;
            telemetry.addData("IR Attempts", ii);
            telemetry.update();
            Thread.sleep(500);
        }
            telemetry.update();
            tfod.deactivate();
        } else {
            return SignalState.GREEN_CIRCLE;
        }
        
        telemetry.addLine("Failed to recognize, defaulted to GREEN_CIRCLE");
        telemetry.update();
        Thread.sleep(500);
        
        return SignalState.GREEN_CIRCLE;
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
