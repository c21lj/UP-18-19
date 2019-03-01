package org.firstinspires.ftc.teamcode;

//UP!!

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Iterator;
import java.util.List;

@Autonomous(name = "NJ Marker")

public class AutoTests extends JackalopeAutoMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime clock = new ElapsedTime();
    private double startTime = 0.0;
    double scale;
    double drive_scale;
    double frontLeft;
    double frontRight;
    double backRight;
    double backLeft;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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
    private VuforiaInterop vf;

    @Override
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
     * Code to run ONCE when the driver hits INIT
     */
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        pullup = hardwareMap.get(DcMotor.class, "pullup");
        flipper = hardwareMap.get(Servo.class, "flipper");

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

        pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        // Reset the timer to zero.
        runtime.reset();

        // Initialize vuforia
        vf = new VuforiaInterop(telemetry, hardwareMap);
        vf.init();

        // Wait for the start button to be pressed on the phone.
        waitForStart();

        flipper.setPosition(.01);

        pullup.setPower(-1);
//        pullup.setTargetPosition(pullup.getCurrentPosition() - 21500);
        pullup.setTargetPosition(pullup.getCurrentPosition() - 19500);

        while (pullup.getTargetPosition() + 50 < pullup.getCurrentPosition() && opModeIsActive()) {
            telemetry.addData("current position", pullup.getCurrentPosition());
            telemetry.addData("target position", pullup.getTargetPosition());

            telemetry.update();
            sleep(10);
            idle();
        }

        telemetry.update();
        goForward();
        sleep(500);

        goStop();//
        sleep(1000);//

        goBack();
        sleep(500);

        goStop();
//        sleep(1000); //

//        goLeft();
//        sleep(2000);
//        flipper.setPosition(.90);

        // Wait for a few seconds to allow detection to settle
        sleep((long) (INITIAL_DETECTION_DELAY_SECONDS * 1000));

        MineralLocation mineralLocation = null;

        // The number of times detection has been attempted
        int attempts = 0;
        telemetry.addData("Detection attempts", attempts);

        telemetry.addData("Detection status", "Detecting");
        telemetry.update();

        if (vf.tfod != null) {
            vf.tfod.activate();

            // Try to detect the gold mineral until a conclusion is reached
            while (opModeIsActive() && mineralLocation == null && attempts <= MAX_DETECTION_ATTEMPTS) {
                mineralLocation = sample();
                attempts++;
                telemetry.addData("Detection attempts:", attempts);
                telemetry.update();

                // Wait for some time to allow a result to be reached
                sleep((long) (INITIAL_DETECTION_DELAY_SECONDS * 1000));
            }

            // Drive in different directions depending on the location of the gold mineral
            if (mineralLocation == MineralLocation.LEFT) {
                telemetry.addData("Detection status", "Success: Left");
                telemetry.update();
                pushLeft();
            } else if (mineralLocation == MineralLocation.CENTER) {
                telemetry.addData("Detection status", "Success: Center");
                telemetry.update();
                pushCenter();
            } else if (mineralLocation == MineralLocation.RIGHT) {
                telemetry.addData("Detection status", "Success: Right");
                telemetry.update();
                pushRight();
            } else {
                // If nothing was detected, default to going left
                telemetry.addData("Detection status", "Failure (default Left)");
                telemetry.update();
                pushLeft();
            }
        } else {
            // If tfod fails to activate, default to going right
            telemetry.addData("Detection status", "Failure");
            telemetry.update();
            pushRight();
        }

//        goStop();
        telemetry.update();
        sleep(100000);
        telemetry.update();
//        idle();

    }

    /**
     * Copied from General 2-28-19
     * Use TFOD to detect the location of the gold mineral
     *
     * @return The location of the gold mineral, or null if could not detect
     */
    private MineralLocation sample() {
        // Get a list of recognized objects
        List<Recognition> updatedRecognitions = vf.tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {

            // Get the number of recognized objects
            final int numRecognitions = updatedRecognitions.size();
            telemetry.addData("# Objects Detected", numRecognitions);
            telemetry.update();

            float goldMineralX = 0;
            float silverMineral1X = 0;
            float silverMineral2X = 0;

            boolean goldFound = false;
            boolean silver1Found = false;
            boolean silver2Found = false;

            // Check how many minerals were recognized
            if (numRecognitions == 3) {
                // If three minerals were detected, find the x and type (gold or silver) of each
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && !goldFound) {
                        goldMineralX = recognition.getLeft();
                        goldFound = true;
                    } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) && !silver1Found) {
                        silverMineral1X = recognition.getLeft();
                        silver1Found = true;
                    } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) && !silver2Found) {
                        silverMineral2X = recognition.getLeft();
                        silver2Found = true;
                    }
                }

                // Check if 1 gold and two silver minerals were found; if not, return null
                if (goldFound && silver1Found && silver2Found) {
                    // If gold is to the left of both silvers, return LEFT.
                    // If gold is to the right of both silvers, return RIGHT.
                    // Otherwise, return CENTER.
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        return MineralLocation.LEFT;
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        return MineralLocation.RIGHT;
                    } else {
                        return MineralLocation.CENTER;
                    }
                } else {
                    return null;
                }
            } else if (numRecognitions == 2) {
                // If two minerals were recognized, assume that the two minerals seen are the left two
                // Get each recognition.
                Recognition recognition1 = updatedRecognitions.get(0);
                Recognition recognition2 = updatedRecognitions.get(1);

                // Mineral types: false: silver, true: gold
                boolean isGold1;
                boolean isGold2;

                // Figure out whether mineral 1 is gold or silver
                if (recognition1.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    isGold1 = true;
                } else {
                    isGold1 = false;
                }

                // Figure out whether mineral 2 is gold or silver
                if (recognition2.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    isGold2 = true;
                } else {
                    isGold2 = false;
                }

                // If neither mineral is gold, then return RIGHT, since the other mineral is the gold one
                // If both are gold, return null (this represents failure)
                if (!isGold1 && !isGold2) {
                    return MineralLocation.RIGHT;
//                    return MineralLocation.LEFT;
                } else if (isGold1 && isGold2) {
                    return null;
                } else {
                    // If one is gold and the other is silver, figure out if gold is to the right or left of the silver
                    if (isGold1) {
                        if (recognition1.getLeft() < recognition2.getLeft()) {
                            return MineralLocation.LEFT;
//                            return MineralLocation.CENTER;
                        } else {
                            return MineralLocation.CENTER;
//                            return MineralLocation.RIGHT;
                        }
                    } else {
                        if (recognition2.getLeft() < recognition1.getLeft()) {
                            return MineralLocation.LEFT;
//                            return MineralLocation.CENTER;
                        } else {
                            return MineralLocation.CENTER;
//                            return MineralLocation.RIGHT;
                        }
                    }
                }
            } else {
                return null;
            }
        } else {
            return null;
        }
    }

    /**
     * Drive to and displace the left mineral
     */
    private void pushLeft() {
        goForward();
        sleep(250);
        goStop();
        sleep(250);
        goRight();
        sleep(1000);
        goStop();
    }

    /**
     * Drive to and displace the left mineral
     */
    private void pushCenter() {
        goBack();
        sleep(250);
        goStop();
        sleep(250);
        goRight();
        sleep(1000);
        goStop();
    }

    /**
     * Drive to and displace the left mineral
     */
    private void pushRight() {
        goBack();
        sleep(1300);
        goStop();
        sleep(250);
        goRight();
        sleep(1600);
        goStop();
    }

    // VUFORIA DETECTION CONSTANTS
    public static final int MAX_DETECTION_ATTEMPTS = 8;
    public static final double INITIAL_DETECTION_DELAY_SECONDS = 3;
    public static final double DETECTION_DELAY_SECONDS = .75;
}

/**
 * Represents the location of the gold mineral
 */
enum MineralLocation {
    LEFT, CENTER, RIGHT
}