/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlow Object Detection", group = "Concept")

public class ConceptTensorFlowObjectDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private DcMotor input = null;
    private DcMotor output = null;
    private DcMotor wobbleArm = null;
    private Servo topClaw = null;
    private Servo bottomClaw = null;
    private Servo transfer = null;
    private DcMotor input2 = null;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AUlgfJn/////AAABmSmPlnjIykFgiXuG0cnJgVxgqg0iEKJga4zsTXBAAuj+tza9T8jqbfj+p6P52eZ5mUih3cbSRZXpLptKQIbkKdFZ/Bu+2DdMRHHi5jgc26PeUDgsttVKtAT+nET3SfAeI+XUvqbBoknhmjURqIyG0hrJZwDutq5FL+6pz54WC34kOciNuE3kWzsCiyYxeFbejWexFeYWbxN1DJd27Im1wElw2vDWWjq4j2rcFJQow98/HrqMV4Qen6DbPHa6pTHAoxAmIoQzaowqb/m3BOdeF1pxMQUqbLXk6S195f7V4sxDhmD9fTFKi8+5kzpj6pE6Km6Svce5m8xGEQz4LcOIwu7OPxh+yIheNDQ7mnLWdOvd";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        input = hardwareMap.get(DcMotor.class, "input");
        output = hardwareMap.get(DcMotor.class, "output");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        transfer = hardwareMap.get(Servo.class, "transferServo");
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        input = hardwareMap.get(DcMotor.class, "input");
        output = hardwareMap.get(DcMotor.class, "output");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        transfer = hardwareMap.get(Servo.class, "transferServo");
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        input2 = hardwareMap.get(DcMotor.class, "input2");
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        transfer.setPosition(0.8);
        topClaw.setPosition(0.23);
        bottomClaw.setPosition(0.4);
        waitForStart();

        /*
        Blue Alliance Left side
         */

        //forward 1.5 tiles
        wobbleArm.setPower(1.0);
        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(21)
                .build();
        //strafe right 0.5 tile
        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .strafeRight(10)
                .build();

        drivetrain.followTrajectory(traj1);
        drivetrain.followTrajectory(traj2);
        //PAUSE TO SENSE
        sleep(2500);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                String blah = "";
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            blah = recognition.getLabel();
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
                telemetry.addData("New label: ", blah);
                if(blah.equals("Quad")){
                    //strafe left 1 tile
                    Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                            .strafeLeft(22)
                            .build();
                    drivetrain.followTrajectory(traj3);

                    //forward 2 tiles
                    Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                            .forward(88)
                            .build();
                    drivetrain.followTrajectory(traj4);

                    //drop wobble 1
                    wobbleArm.setPower(0.0);
                    topClaw.setPosition(0.6);
                    bottomClaw.setPosition(0.1);
                    sleep(100);

                    Trajectory blah1 = drivetrain.trajectoryBuilder(traj4.end())
                            .back(34)
                            .build();
                    drivetrain.followTrajectory(blah1);

                    //strafe right 1 tile
                    Trajectory traj5 = drivetrain.trajectoryBuilder(blah1.end())
                            .strafeRight(18)
                            .build();
                    drivetrain.followTrajectory(traj5);
                    output.setPower(1.0);
                    drivetrain.turn(Math.toRadians(183));
                    sleep(100);
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);
                    drivetrain.turn(Math.toRadians(-5));
                    sleep(100);
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    drivetrain.turn(Math.toRadians(-5));
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);
                    drivetrain.turn(Math.toRadians(-5));
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);

                    drivetrain.turn(Math.toRadians(18));
                    output.setPower(0.0);

                    //forward 3.5 tiles
                    Trajectory back = drivetrain.trajectoryBuilder(new Pose2d(traj5.end().getX(), traj5.end().getY(), Math.toRadians(185)))
                            .strafeLeft(29)
                            .build();
                    drivetrain.followTrajectory(back);

                    //forward 2 tiles
                    Trajectory traj6 = drivetrain.trajectoryBuilder(back.end())
                            .forward(54)
                            .build();
                    drivetrain.followTrajectory(traj6);

                    //pick up wobble 2
                    topClaw.setPosition(0.23);
                    bottomClaw.setPosition(0.4);
                    wobbleArm.setPower(-1.0);
                    sleep(1000);
                    wobbleArm.setPower(0.0);

                    drivetrain.turn(Math.toRadians(185));

                    Trajectory traj7 = drivetrain.trajectoryBuilder(new Pose2d(traj6.end().getX(), traj6.end().getY(), traj6.end().getHeading()+Math.toRadians(185)))
                            .forward(43)
                            .build();
                    drivetrain.followTrajectory(traj7);

                    wobbleArm.setPower(-1.0);
                    sleep(1500);

                    //strafe forward 1 tile
                    Trajectory traj8 = drivetrain.trajectoryBuilder(traj7.end())
                            .strafeLeft(85)
                            .build();
                    drivetrain.followTrajectory(traj8);

                    wobbleArm.setPower(-1.0);
                    sleep(2000);

                    //}
                    //drop wobble 2
                    wobbleArm.setPower(1.0);
                    sleep(1000);
                    wobbleArm.setPower(0.0);
                    sleep(5000);

                    Trajectory traj9 = drivetrain.trajectoryBuilder(traj8.end())
                            .back(42)
                            .build();
                    drivetrain.followTrajectory(traj9);
                }
                else if(blah.equals("Single")){
                    //strafe left 1 tile
                    Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                            .strafeLeft(22)
                            .build();
                    drivetrain.followTrajectory(traj3);

                    //forward 2 tiles
                    Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                            .forward(70)
                            .build();
                    drivetrain.followTrajectory(traj4);

                    //forward 2 tiles
                    Trajectory blah2 = drivetrain.trajectoryBuilder(traj4.end())
                            .strafeRight(12)
                            .build();
                    drivetrain.followTrajectory(blah2);

                    //drop wobble 1
                    wobbleArm.setPower(0.0);
                    topClaw.setPosition(0.6);
                    bottomClaw.setPosition(0.1);
                    sleep(100);

                    Trajectory blah1 = drivetrain.trajectoryBuilder(blah2.end())
                            .back(22)
                            .build();
                    drivetrain.followTrajectory(blah1);

                    //strafe right 1 tile
                    output.setPower(1.0);
                    drivetrain.turn(Math.toRadians(183));
                    sleep(100);
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);
                    drivetrain.turn(Math.toRadians(-5));
                    sleep(100);
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    drivetrain.turn(Math.toRadians(-5));
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);
                    drivetrain.turn(Math.toRadians(-5));
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);

                    drivetrain.turn(Math.toRadians(18));
                    output.setPower(0.0);

                    //forward 3.5 tiles
                    Trajectory back = drivetrain.trajectoryBuilder(new Pose2d(blah1.end().getX(), blah1.end().getY(), Math.toRadians(185)))
                            .strafeLeft(24)
                            .build();
                    drivetrain.followTrajectory(back);

                    //forward 2 tiles
                    Trajectory traj6 = drivetrain.trajectoryBuilder(back.end())
                            .forward(30)
                            .build();
                    drivetrain.followTrajectory(traj6);

                    //pick up wobble 2
                    topClaw.setPosition(0.23);
                    bottomClaw.setPosition(0.4);
                    wobbleArm.setPower(-1.0);
                    sleep(1000);
                    wobbleArm.setPower(0.0);

                    drivetrain.turn(Math.toRadians(185));

                    Trajectory traj7 = drivetrain.trajectoryBuilder(new Pose2d(traj6.end().getX(), traj6.end().getY(), traj6.end().getHeading()+Math.toRadians(185)))
                            .forward(43)
                            .build();
                    drivetrain.followTrajectory(traj7);

                    //strafe forward 1 tile
                    Trajectory traj8 = drivetrain.trajectoryBuilder(traj7.end())
                            .strafeLeft(22)
                            .build();
                    drivetrain.followTrajectory(traj8);

                    wobbleArm.setPower(-1.0);
                    sleep(1500);

                    Trajectory blah3 = drivetrain.trajectoryBuilder(traj8.end())
                            .forward(21)
                            .build();
                    drivetrain.followTrajectory(blah3);

                    wobbleArm.setPower(1.0);
                    sleep(2000);

                    //}
                    //drop wobble 2
                    wobbleArm.setPower(1.0);
                    sleep(1000);
                    wobbleArm.setPower(0.0);
                    sleep(5000);

                    Trajectory traj9 = drivetrain.trajectoryBuilder(traj8.end())
                            .back(24)
                            .build();
                    drivetrain.followTrajectory(traj9);

                    //strafe forward 1 tile
                    Trajectory traj10 = drivetrain.trajectoryBuilder(traj9.end())
                            .strafeLeft(22)
                            .build();
                    drivetrain.followTrajectory(traj10);
                }
                else{
                    //strafe left 1 tile
                    Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                            .strafeLeft(22)
                            .build();
                    drivetrain.followTrajectory(traj3);

                    //forward 2 tiles
                    Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                            .forward(46)
                            .build();
                    drivetrain.followTrajectory(traj4);

                    //drop wobble 1
                    wobbleArm.setPower(0.0);
                    topClaw.setPosition(0.6);
                    bottomClaw.setPosition(0.1);
                    sleep(100);

                    Trajectory blah1 = drivetrain.trajectoryBuilder(traj4.end())
                            .back(10)
                            .build();
                    drivetrain.followTrajectory(blah1);

                    //strafe right 1 tile
                    Trajectory traj5 = drivetrain.trajectoryBuilder(blah1.end())
                            .strafeRight(18)
                            .build();
                    drivetrain.followTrajectory(traj5);
                    output.setPower(1.0);
                    drivetrain.turn(Math.toRadians(183));
                    sleep(100);
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);
                    drivetrain.turn(Math.toRadians(-5));
                    sleep(100);
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    drivetrain.turn(Math.toRadians(-5));
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);
                    drivetrain.turn(Math.toRadians(-5));
                    transfer.setPosition(0.7);
                    sleep(100);
                    transfer.setPosition(0.8);
                    sleep(100);

                    drivetrain.turn(Math.toRadians(18));
                    output.setPower(0.0);

                    //forward 3.5 tiles
                    Trajectory back = drivetrain.trajectoryBuilder(new Pose2d(traj5.end().getX(), traj5.end().getY(), Math.toRadians(185)))
                            .strafeLeft(24)
                            .build();
                    drivetrain.followTrajectory(back);

                    //forward 2 tiles
                    Trajectory traj6 = drivetrain.trajectoryBuilder(back.end())
                            .forward(28)
                            .build();
                    drivetrain.followTrajectory(traj6);

                    wobbleArm.setPower(-1.0);
                    sleep(1500);

                    Trajectory blah4 = drivetrain.trajectoryBuilder(traj6.end())
                            .strafeRight(7)
                            .build();
                    drivetrain.followTrajectory(blah4);

                    wobbleArm.setPower(1.0);
                    sleep(2000);

                    //pick up wobble 2
                    topClaw.setPosition(0.23);
                    bottomClaw.setPosition(0.4);
                    wobbleArm.setPower(-1.0);
                    sleep(1000);
                    wobbleArm.setPower(0.0);

                    drivetrain.turn(Math.toRadians(185));

                    Trajectory traj7 = drivetrain.trajectoryBuilder(new Pose2d(blah4.end().getX(), blah4.end().getY(), blah4.end().getHeading()+Math.toRadians(185)))
                            .forward(43)
                            .build();
                    drivetrain.followTrajectory(traj7);

                    //strafe forward 1 tile
                    Trajectory traj8 = drivetrain.trajectoryBuilder(traj7.end())
                            .strafeLeft(43)
                            .build();
                    drivetrain.followTrajectory(traj8);
                    //}
                    //drop wobble 2
                    wobbleArm.setPower(1.0);
                    sleep(1000);
                    wobbleArm.setPower(0.0);
                    sleep(5000);
                }
            }
            //PoseStorage.currentPose = drivetrain.getPoseEstimate();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
