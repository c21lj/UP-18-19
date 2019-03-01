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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.annotation.Target;

@Autonomous(name = "Pullup Only")

public class autoPullupOnly extends JackalopeAutoMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo flipper = null;
    private boolean read = false;
    private DcMotor pullupEncoder;
    private ColorSensor CBL;
    private boolean gripped = false;
    private boolean lifted = false;
    private double short_drive_x;
    private boolean modeBool = false;
    private double power;
    private int distance;
    private double short_drive_y;
    private ElapsedTime clock = new ElapsedTime();
    private double startTime = 0.0;
    double scale;
    double drive_scale;

    //for encoders:
//    static final double     TICKS    = 537.6 ;
    static final int     TICKS    = 1120 ;

    double frontLeft;
    double frontRight;
    double backRight;
    double backLeft;

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
        string.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
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
        string = hardwareMap.get(DcMotor.class, "string");
        flipper = hardwareMap.get(Servo.class, "flipper");
//        pullupEncoder = hardwareMap.get(DcMotor.class, "string");

        // Set the initial directions of the motors
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        pullup.setDirection(DcMotorSimple.Direction.FORWARD);
        string.setDirection(DcMotor.Direction.REVERSE);

        // Set the behaviour when motors' power is set to zero -- whether to brake
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        pullup.setDirection(DcMotor.Direction.FORWARD);
        string.setDirection(DcMotor.Direction.FORWARD);
        flipper.setDirection(Servo.Direction.FORWARD);

//        //encoders modes:
//        pullup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pullup.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pullup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

       // pullup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        pullup.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  pullup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        // Reset the timer to zero.
        runtime.reset();

        // Wait for the start button to be pressed on the phone.
        waitForStart();

        telemetry.addData("read", read);

        pullup.setPower(-1);
        pullup.setTargetPosition(pullup.getCurrentPosition() - 19900);

        while (pullup.getTargetPosition() + 50 < pullup.getCurrentPosition() && opModeIsActive()) {
            telemetry.addData("current position", pullup.getCurrentPosition());
            telemetry.addData("target position", pullup.getTargetPosition());

            telemetry.update();
            sleep(10);
            idle();
        }

//        goForward();
//        sleep(600);
//        goStop();

//        goBack();
//        sleep(500);
//        goStop();
//        BR.setTargetPosition(BR.getCurrentPosition() - 2000);
//
//        FL.setPower(-.4);
//        BL.setPower(-.4);
//        FR.setPower(.4);
//        BR.setPower(.4);
//
//        while (BR.getCurrentPosition() + 50 > pullup.getCurrentPosition() && opModeIsActive()) {
////            goForward();
//            telemetry.addData("BR position", BR.getTargetPosition());
//            telemetry.update();
//            sleep(10);
//            idle();
//        }

//
//            pullup.setPower(0);

//        else {
//            pullup.setPower(0);
//        }

            // Send the power variables to the driver.
            telemetry.addData("FR", frontRight);
            telemetry.addData("FL", frontLeft);
            telemetry.addData("BR", backRight);
            telemetry.addData("BL", backLeft);

            // Set the powers of the motors to the power variables.
            // Update the displayed values on the driver phone.
            telemetry.update();
            idle();
        }


    /*
     * Scales a value to the appropriate range--used for calculating motor powers/servo positions.
     * For instance, you could use this to map 5 in the range (0,10) to 0.25 in the range (0,0.5)
     */


    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}

