
    /* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


    /**
     * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
     * determine the position of the gold and silver minerals.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
     *
     * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
     * is explained below.
     */
    @Autonomous(name = "Webcam", group = "Concept")

    public class Webcam extends JackalopeAutoMode {
        private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
        private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

        public void strafe(boolean strafe) {
            FR.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            FL.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.FORWARD);
            BR.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.REVERSE);
            BL.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            pullup.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        }


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
        private static final String VUFORIA_KEY = "AaZ4Ce//////AAABmQNH9qjW3kdipeX3pwALo7AMqq0hegMq/4GSQffFZ8gOdNhWPhKqb7J9OuwyehkaEpRdzeiK/Z9mUAnBM50iClTSQHlwx+JEhbQR28BSvIsdTrsPMpKyQw7cc8CIRCf+dTRK1kkq4llrAxvZkAco2OdPT5kHfsDD0ybnM6i7hL7YvfrW4ot3AAPRikoBpinoJhE4dNP1Cd5t7eOlM5JlTsTw2x3TR38Ic7ocqs3cCRGWpiuaFD5+WNgZUNSBsat5zeuclM28cBz3eoRIxtwGGGdXC0bLMXvlNPQr69raULr4GGGIQMoTOpY83X/iI64q86y2vIjbu+FFzHDrVZz258w02PBH6Knny42TU3dCLBGq";

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        private VuforiaLocalizer vuforia;

        /**
         * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
         * Detection engine.
         */
        private TFObjectDetector tfod;

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry.addData("Here", "You have reached this point");
            string = hardwareMap.get(DcMotor.class, "string");
            string.setDirection(DcMotor.Direction.REVERSE);
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();

            FR = hardwareMap.get(DcMotor.class, "FR");
            FL = hardwareMap.get(DcMotor.class, "FL");
            BR = hardwareMap.get(DcMotor.class, "BR");
            BL = hardwareMap.get(DcMotor.class, "BL");
            pullup = hardwareMap.get(DcMotor.class, "pullup");

            // Set the initial directions of the motors
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            BR.setDirection(DcMotor.Direction.REVERSE);
            FR.setDirection(DcMotor.Direction.REVERSE);
            pullup.setDirection(DcMotorSimple.Direction.FORWARD);

            // Set the behaviour when motors' power is set to zero -- whether to brake
            FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
            pullup.setDirection(DcMotorSimple.Direction.FORWARD);

            // Wait for the start button to be pressed on the phone.
            waitForStart();
            //see if this works

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();

//            if (opModeIsActive()) {

                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }

                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
//                                        string.setPower(.5);
                                        goLeft();
                                        sleep(500);
                                        goStop();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                        string.setPower(-.5);
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                        string.setPower(.2);
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
//            }

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
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }

        /**
         * Initialize the Tensor Flow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }
    }

