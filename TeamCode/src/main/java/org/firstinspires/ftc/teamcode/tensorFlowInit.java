package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class tensorFlowInit {
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static final String VUFORIA_KEY = "AUlgfJn/////AAABmSmPlnjIykFgiXuG0cnJgVxgqg0iEKJga4zsTXBAAuj+tza9T8jqbfj+p6P52eZ5mUih3cbSRZXpLptKQIbkKdFZ/Bu+2DdMRHHi5jgc26PeUDgsttVKtAT+nET3SfAeI+XUvqbBoknhmjURqIyG0hrJZwDutq5FL+6pz54WC34kOciNuE3kWzsCiyYxeFbejWexFeYWbxN1DJd27Im1wElw2vDWWjq4j2rcFJQow98/HrqMV4Qen6DbPHa6pTHAoxAmIoQzaowqb/m3BOdeF1pxMQUqbLXk6S195f7V4sxDhmD9fTFKi8+5kzpj6pE6Km6Svce5m8xGEQz4LcOIwu7OPxh+yIheNDQ7mnLWdOvd";
    public static VuforiaLocalizer vuforia;
    public static TFObjectDetector tfod;

    public static void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    public static void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
