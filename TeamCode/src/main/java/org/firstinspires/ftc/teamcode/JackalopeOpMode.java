package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

abstract class JackalopeOpMode extends LinearOpMode {

    HardwareUPLauren robot = new HardwareUPLauren();

    /**
     * Front-right Servo

     */
DcMotor FR = null;
    /**
     * Front-left Servo
     */
    DcMotor FL = null;

    /**
     * Back-right Servo
     */
    DcMotor BR = null;

    /**
     * Back-left Servo
     */
    DcMotor BL = null;

    /**
     * left nom servo
     */
    CRServo leftnom = null;

    /**
     * right nom servo
     */
    CRServo rightnom = null;

    DcMotor shoulder = null;
    DcMotor elbow = null;
    DcMotor pullup = null;

//    Behaviour when the motors are stopped

    static final ZeroPowerBehavior ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.BRAKE;
    public void strafe(boolean strafe) {
        FR.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        FL.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.FORWARD);
        BR.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.REVERSE);
        BL.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

    }
    public double getAngleDiff(double angle1, double angle2) {
        if(Math.abs(angle1 - angle2) < 180.0)
            return Math.abs(angle1-angle2);
        else if(angle1 > angle2)
        {
            angle1 -= 360;
            return Math.abs(angle2-angle1);
        }
        else
        {
            angle2 -= 360;
            return Math.abs(angle1-angle2);
        }
    }
}

