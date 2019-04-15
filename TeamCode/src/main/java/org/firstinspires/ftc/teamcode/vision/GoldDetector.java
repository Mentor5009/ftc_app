package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class GoldDetector {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AfZdcpz/////AAAAGeFAEIQ7eEL9ilMArx0PrTpfGi14uY5DxJNi9A/pNhrpWpMLBsZIt21zn61HlpOEsX4SW/GyN//S+CJpHALNQkDftlrlJ3+cGtzrVC0ZZcEpltXAdp/5CkO+M7Q3rDOtKBeFhCBnjDUVswvmD0sU9mRgjVhn5TvOSXcSuJIJymIy5x15BUxbqsZe+5Rkzt4a/4ltQvr3jN13s4RECp03x+zfPWKR7S79x1+VSITaBB5lrv43p9ZEBJeIaWlAXQTST8O0uf2YhNXCuzrBxuAgL5onSpOWmBUzyFxE8cooXOgyktMm/mtYHG+vujg4gG9FpxFFLutypcN3hLaBOqfS1DyNrQD1i05cpRLwJ4M0Gszc";

    private LinearOpMode om;
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    public GoldDetector(LinearOpMode opMode) {
        om = opMode;
        vuforia = initVuforia();

        // Initializing tfod
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
            tfod.activate();
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public MineralPosition getGoldPos(int timeout) {
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (om.opModeIsActive() && t.milliseconds() < timeout) {
            if (tfod == null) break;

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                om.telemetry.addData("# Objects Detected", updatedRecognitions.size());
                int i = 1;

                String rightMineral = "";
                String centreMineral = "";
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLeft() > 350) {
                        om.telemetry.addData("object " + String.valueOf(i), recognition.getLabel() + "," + recognition.getTop() + "," + recognition.getLeft());
                        i++;

                        if (recognition.getTop() < 600 && !rightMineral.equals("Silver Mineral")) {
                            rightMineral = recognition.getLabel();
                        } else if (recognition.getTop() >= 600 && recognition.getTop() <= 1000 && !centreMineral.equals("Silver Mineral")) {
                            centreMineral = recognition.getLabel();
                        }
                    }
                }

                om.telemetry.addData("rightMineral", rightMineral);
                om.telemetry.addData("centreMineral", centreMineral);

                if (!rightMineral.equals("Silver Mineral")) {
                    return MineralPosition.RIGHT;
                } else if (!centreMineral.equals("Silver Mineral")) {
                    return MineralPosition.CENTRE;
                } else {
                    return MineralPosition.LEFT;
                }
            }
        }
        return MineralPosition.RIGHT;
    }

    public void shutdown() {
        if (tfod != null)
            tfod.shutdown();
    }

    private VuforiaLocalizer initVuforia() {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        return ClassFactory.getInstance().createVuforia(parameters);
    }
}
