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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "autonRight", group = "Autonomous")

public class autonRight extends LinearOpMode {

    public DcMotorEx liftMotor;
    public DcMotorEx liftMotor2;
    public static final int BOTTOM_LEVEL_POSITION = 1800;
    public static final int MIDDLE_LEVEL_POSITION = 3100;
    public static final int TOP_LEVEL_POSITION = 4000;
    public static final int TOP_LEVEL = 3;
    public static final int MIDDLE_LEVEL = 2;
    public static final int BOTTOM_LEVEL = 1;
    public Servo rightClaw;
    public Servo leftClaw;
    public int detectedLevel;

    private String label = "";
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/PowerPlay3.tflite";

    private static final String[] LABELS = {
            "Checker",
            "Donut",
            "Melon"
    };

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
    private static final String VUFORIA_KEY =
            "ARzmjp7/////AAABmd0h9zcHGEqrnG8hGLpGgOGFJh8GZLjNLJuHiuATJ+jS0YFgEkcTrADzFQMI5JqXF7O5RSMnopPM7mvBjWVeaSYuhDCHsduGTOULaG2ET3QEKnxo2z8fMF3vpcbFAXuQrZq7UqXgrocnkQqfronZLpU78zmsU/WqoA1OIIPzDfrxDWLdxZy3sxDCHFmKZHMcykOC2AKxqbQd8s9fjzdEZ5xeLx41F3iWW6rJ+9g9/Ggmzdd7IqQu9WPUCaacw3y1lSlZSgqdzuDgLAeRUThH9RtVEjT7uebk3u5C8ApwAqUjrnMKbeQ4ZDoc0cCFxafuhwFdt2chVLqVizgMNHp9FWjnwDyHn4IMEI3L2zeueEnX";

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

    public void pickUp() throws InterruptedException {
        rightClaw.setPosition(.65);
        leftClaw.setPosition(0);
        Thread.sleep(500);
    }

    public void drop() throws InterruptedException {
        rightClaw.setPosition(.37);
        leftClaw.setPosition(.3);
        Thread.sleep(500);
    }

    public void changeLift (int height)  throws InterruptedException {
        Thread.sleep(100);

        if (liftMotor.getCurrentPosition() < height) {
            liftMotor.setTargetPosition(height);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.9);
        }
        else {
            liftMotor.setTargetPosition(height);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-0.7);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        double strafeDistance = 1;


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);




        Trajectory strafeRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(10, 21))
                .build();

        Trajectory dropBlock = drive.trajectoryBuilder(strafeRight.end())
                .lineToLinearHeading(new Pose2d(39.25, 16.5, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory goBack1 = drive.trajectoryBuilder(dropBlock.end())
                .back(5, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory strafeRight2 = drive.trajectoryBuilder(goBack1.end())
                .strafeLeft(12)
                .build();

        Trajectory pickBlock = drive.trajectoryBuilder(strafeRight2.end())
                .lineToLinearHeading(new Pose2d(54, -30.5, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory goBack = drive.trajectoryBuilder(pickBlock.end())
                .back(20, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory dropBlock2 = drive.trajectoryBuilder(goBack.end())
                .lineToLinearHeading(new Pose2d(49.2, -13.9, Math.toRadians(180)))
                .build();

        Trajectory goBack2 = drive.trajectoryBuilder(dropBlock2.end())
                .back(6, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory pickBlock2 = drive.trajectoryBuilder(goBack2.end())
                .lineToLinearHeading(new Pose2d(53, -30.5, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory reset = drive.trajectoryBuilder(dropBlock2.end())
                .lineToLinearHeading(new Pose2d(56, -28, Math.toRadians(184)))
                .build();



        while(!opModeIsActive()) {
            if (tfod != null) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        label = recognition.getLabel();
                        telemetry.addData("object detection", label);
                    }
                    telemetry.update();
                }
            }
        }

        if (label.equals("Donut")) { strafeDistance = 24; }
        else if (label.equals("Checker")) { strafeDistance = 48; }

        Trajectory park = drive.trajectoryBuilder(reset.end())
                .strafeRight(strafeDistance)
                .build();

        waitForStart();
        if (opModeIsActive()) {
            pickUp();
            drive.followTrajectory(strafeRight);
            changeLift(MIDDLE_LEVEL_POSITION);
            drive.followTrajectory(dropBlock);
            drop();
            drive.followTrajectory(goBack1);
            drive.followTrajectory(strafeRight2);
            changeLift(650);
            drive.followTrajectory(pickBlock);
            Thread.sleep(500);
            pickUp();
            changeLift(BOTTOM_LEVEL_POSITION);
            Thread.sleep(600);
            drive.followTrajectory(goBack);
            drive.followTrajectory(dropBlock2);
            drop();
            changeLift(550);
            drive.followTrajectory(goBack2);
            drive.followTrajectory(pickBlock2);
            Thread.sleep(460);
            pickUp();
            changeLift(BOTTOM_LEVEL_POSITION);
            Thread.sleep(600);
            drive.followTrajectory(goBack);
            drive.followTrajectory(dropBlock2);
            drop();
            changeLift(0);
            drive.followTrajectory(reset);
            drive.followTrajectory(park);
            //if (label.equals())
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "frontCamera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
