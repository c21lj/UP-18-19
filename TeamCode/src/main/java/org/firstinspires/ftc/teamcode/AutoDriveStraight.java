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

@Autonomous(name = "AutoMarker")

public class AutoDriveStraight extends JackalopeAutoMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
//    private CRServo leftnom = null;
//    private CRServo rightnom = null;
    private Servo flipper = null;
    private boolean read = false;
    private ColorSensor CBL;
    private boolean gripped = false;
    private boolean lifted = false;
    private double short_drive_x;
    private boolean modeBool = false;
    private double short_drive_y;
    private ElapsedTime clock = new ElapsedTime();
    private double startTime = 0.0;
    double scale;
    double drive_scale;
    //    double gamepad1LeftY;
//    double gamepad1LeftX;
//    double gamepad1RightX;
//    boolean rightbumper;
//    boolean leftbumper;
//    boolean abutton;
//    boolean bbutton;
//    boolean xbutton;
    boolean ybutton;
    double frontLeft;
    double frontRight;
    double backRight;
    double backLeft;
//    double righttrigger;
//    double lefttrigger;
//    boolean gamepad2DpadDown;
//    boolean gamepad2DpadUp;

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

    public void delay(int time){
        double startTime = clock.milliseconds();
        while((clock.milliseconds() - startTime < time) && !isStopRequested()){

        }
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
        flipper.setDirection(Servo.Direction.FORWARD);

        // Set the behaviour when motors' power is set to zero -- whether to brake
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        pullup.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        // Reset the timer to zero.
        runtime.reset();

        // Wait for the start button to be pressed on the phone.
        waitForStart();

            // Loop until the op mode is stopped. changes
            telemetry.addData("read", read);
            flipper.setPosition(0);

//            delay(25000);
            pullup.setPower(.7);
            telemetry.addData("power", pullup.getPower());
            telemetry.update();
            sleep(8500);
            pullup.setPower(0);
            telemetry.addData("power", pullup.getPower());
            telemetry.update();
            goLeft();
            sleep(3000);
            goStop();
            flipper.setPosition(1);
            sleep (2000);
            flipper.setPosition(0);
            sleep (2000);


//            strafe(true);
//            delay(1000);
//            strafe(false);
//
//            goForward();
//            delay(2500\
//            goStop();
//            delay(500);
//
//            turnLeft();
//            delay(1500);
//            goStop();
//
//            elbow.setPower(.5);//elbow up?
//            shoulder.setPower(-.5);//shoulder up
//            delay(750);
//            elbow.setPower(0);
//            shoulder.setPower(0);
//
//            leftnom.setPower(-1);//spit out
//            rightnom.setPower(-1);//spit out
//            delay(2000);
//            leftnom.setPower(0);
//            rightnom.setPower(0);









//                frontRight = 0;
//                frontLeft = 0;
//                backRight = 0;
//                backLeft = 0;
//
//                shoulder.setPower(.5);//shoulder down
//
//                shoulder.setPower(-1);//shoulder up
//
//                elbow.setPower(.5);//elbow up?
//
//                elbow.setPower(-1);//elbow down?
//
//                leftnom.setPower(1);//take in
//                rightnom.setPower(1);//take in
//
//                leftnom.setPower(-1);//spit out
//                rightnom.setPower(-1);//spit out
//
//                pullup.setPower(.7);//pullup up?
//                pullup.setPower(-.7);//pullup down?

        // Send the power variables to the driver.
        telemetry.addData("FR", frontRight);
        telemetry.addData("FL", frontLeft);
        telemetry.addData("BR", backRight);
        telemetry.addData("BL", backLeft);

        // When the op mode is told to stop, stop the motors.
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        pullup.setPower(0);

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

