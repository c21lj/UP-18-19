package org.firstinspires.ftc.teamcode;

//UP!!
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Test Directions")

public class TestDirections extends JackalopeOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean modeBool = false;
    private double short_drive_y;
    private ElapsedTime clock = new ElapsedTime();
    private double startTime = 0.0;
    boolean gamepad2DpadDown;
    boolean gamepad2DpadUp;
    boolean gamepad1DpadUp;
    boolean gamepad1DpadDown;
    boolean gamepad1DpadLeft;
    boolean gamepad1DpadRight;

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
//        leftnom = hardwareMap.get(CRServo.class, "leftnom");
//        rightnom = hardwareMap.get(CRServo.class, "rightnom");

        // Set the initial directions of the motors
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        pullup.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftnom.setDirection(CRServo.Direction.FORWARD);
//        rightnom.setDirection(CRServo.Direction.REVERSE);

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
        while (!isStopRequested() && opModeIsActive()) {

            // left stick controls direction
            // right stick X controls rotation

            // Get data from the gamepad and scale it appropriately. The scale is based upon whether the right bumper is pressed.
            gamepad2DpadDown = gamepad2.dpad_down;
            gamepad2DpadUp = gamepad2.dpad_up;
            gamepad1DpadUp = gamepad1.dpad_up;
            gamepad1DpadDown = gamepad1.dpad_down;
            gamepad1DpadLeft = gamepad1.dpad_down;
            gamepad1DpadRight = gamepad1.dpad_right;

            if (gamepad1DpadUp) {
                //forward
                FL.setPower(.4);
                BL.setPower(.4);
                FR.setPower(-.4);
                BR.setPower(-.4);
            }

            else if (gamepad1DpadDown) {
                //back
                FL.setPower(-.4);
                BL.setPower(-.4);
                FR.setPower(.4);
                BR.setPower(.4);
            }

            else if (gamepad1DpadRight) {
                //left
                FL.setPower(.4);
                BL.setPower(-.4);
                FR.setPower(.4);
                BR.setPower(-.4);
            }

           else if (gamepad1DpadLeft) {
                //right
                FL.setPower(-.4);
                BL.setPower(.4);
                FR.setPower(-.4);
                BR.setPower(.4);

            }
            else {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                pullup.setPower(0);
            }

            // Update the displayed values on the driver phone.
            telemetry.update();
            idle();
        }

        // When the op mode is told to stop, stop the motors.
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        pullup.setPower(0);

    }

}

