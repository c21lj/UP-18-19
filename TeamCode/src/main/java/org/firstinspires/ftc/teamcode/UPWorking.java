/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareUpChassis;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="UPWorking", group="UPWorking")
public class UPWorking extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareUpChassis robot           = new HardwareUpChassis();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() {
        double frontleft;
        double frontright;
        double backleft;
        double backright;
        double scale;
        double drive_scale;
        double gamepad1LeftY;
        double gamepad1LeftX;
        double gamepad1RightX;
        double left;
        double right;
        double threshold = .3;
        double sidemax;
        double frontmax;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
//            // This way it's also easy to just drive straight, or just turn.
            // Get data from the gamepad and scale it appropriately. The scale is based upon whether the right bumper is pressed.
            scale = (gamepad1.right_bumper ? .3 : .7);
            drive_scale = (gamepad1.right_bumper ? .3 : 1);
            gamepad1LeftY = -gamepad1.left_stick_y * drive_scale;
            gamepad1LeftX = gamepad1.left_stick_x * drive_scale;
            gamepad1RightX = gamepad1.right_stick_x * scale;

            // Apply the holonomic formulas to calculate the powers of the motors
            frontleft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            frontright = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            backright = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            backleft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
//
//            // Combine drive and turn for blended motion.
//
            // Normalize the values so neither exceed +/- 1.0
           // backmax = Math.max(Math.abs(frontleft), Math.abs(gamepad1LeftY), Math.abs(gamepad1RightX));
           // frontmax = Math.max(Math.abs(frontleft), Math.abs(gamepad1LeftY), Math.abs(gamepad1RightX));

////            if (max > .2) {
////                frontright /= max;
////                frontleft /= max;
////                backright /= max;
////                backleft /= max;
//                robot.frontleft.setPower(frontleft);
//                // clip the right/left values so that the values never exceed +/- 1
//                frontright = Range.clip(frontright, -1, 1);
//                frontleft = Range.clip(frontleft, -1, 1);
//                backleft = Range.clip(backleft, -1, 1);
//                backright = Range.clip(backright, -1, 1);
//            } else {
//                frontright = 0;
//                frontleft = 0;
//                backright = 0;
//                backleft = 0;
//            }
//            if (left > threshold){
//                robot.frontleft.setPower(left);
//                robot.frontright.setPower(left);
//                robot.backleft.setPower(left);
//                robot.backright.setPower(left);
//            } else {
//                robot.frontleft.setPower(0);
//                robot.frontright.setPower(0);
//                robot.backleft.setPower(0);
//                robot.backright.setPower(0);
//            }
//
//            // Output the safe vales to the motor drives.
//
//            telemetry.addData("right",  "%.2f", right);
//            telemetry.addData("left", "%.2f", left);
//            telemetry.addData("drive",  "%.2f", drive);
//            telemetry.addData("turn",  "%.2f", turn);
//            telemetry.update();
//        }
//    }
//}
