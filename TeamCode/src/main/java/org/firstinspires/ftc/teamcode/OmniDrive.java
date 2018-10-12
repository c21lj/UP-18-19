package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "JackalopeOmniDrive")

public class OmniDrive extends JackalopeOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private Servo SRelicRotate = null;
    private Servo SBlock = null;
//    private Orientation mode = Orientation.LIFT;
    private Servo SRelicPickup = null;
    private boolean read = false;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private ColorSensor CBL;
    private boolean gripped = false;
//    private boolean lifted = false;
    private double short_drive_x;
    private boolean modeBool = false;
    private double short_drive_y;
    private ElapsedTime clock = new ElapsedTime();
    private double startTime = 0.0;
    double scale;
    double drive_scale;
    double gamepad1LeftY;
    double gamepad1LeftX;
    double gamepad1RightX;
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
//        lift = hardwareMap.get(DcMotor.class, "lift");

        // Set the initial directions of the motors
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
//        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set the behaviour when motors' power is set to zero -- whether to brake
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
//        lift.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Reset the timer to zero.
        runtime.reset();

        // Wait for the start button to be pressed on the phone.
        waitForStart();

        // Loop until the op mode is stopped. changes
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("read", read);

            // left stick controls direction
            // right stick X controls rotation

            // Get data from the gamepad and scale it appropriately. The scale is based upon whether the right bumper is pressed.
            scale = (gamepad1.right_bumper ? .3 : .7);
            drive_scale = (gamepad1.right_bumper ? .3 : 1);
            gamepad1LeftY = -gamepad1.left_stick_y * drive_scale;
            gamepad1LeftX = gamepad1.left_stick_x * drive_scale;
            gamepad1RightX = gamepad1.right_stick_x * scale;

            // Apply the holonomic formulas to calculate the powers of the motors
            frontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            frontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            backRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            backLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // If the joystick values are past the threshold, set the power variables to the clipped calculated power.
            // Otherwise, set them to zero.
            if (Math.abs(gamepad1LeftX) > .2 || Math.abs(gamepad1LeftY) > .2 || Math.abs(gamepad1RightX) > .2) {

                // clip the right/left values so that the values never exceed +/- 1
                frontRight = Range.clip(frontRight, -1, 1);
                frontLeft = Range.clip(frontLeft, -1, 1);
                backLeft = Range.clip(backLeft, -1, 1);
                backRight = Range.clip(backRight, -1, 1);
            } else {
                frontRight = 0;
                frontLeft = 0;
                backRight = 0;
                backLeft = 0;
            }

            // Send the power variables to the driver.
            telemetry.addData("FR", frontRight);
            telemetry.addData("FL", frontLeft);
            telemetry.addData("BR", backRight);
            telemetry.addData("BL", backLeft);

            // Set the powers of the motors to the power variables.
            FR.setPower(frontRight);
            FL.setPower(frontLeft);
            BR.setPower(backRight);
            BL.setPower(backLeft);
            // Update the displayed values on the driver phone.
            telemetry.update();
            idle();
        }

        // When the op mode is told to stop, stop the motors.
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }



    /*
     * Scales a value to the appropriate range--used for calculating motor powers/servo positions.
     * For instance, you could use this to map 5 in the range (0,10) to 0.25 in the range (0,0.5)
     */
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}

