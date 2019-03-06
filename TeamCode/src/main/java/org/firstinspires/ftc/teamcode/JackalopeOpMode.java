package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

abstract class JackalopeOpMode extends LinearOpMode {

    HardwareUPLauren robot = new HardwareUPLauren();
    ElapsedTime clock = new ElapsedTime();

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

    Servo rightRotate = null;
    Servo leftRotate = null;
    /**
     * left nom servo
     */
    CRServo leftnom = null;

    /**
     * right nom servo
     */
//    CRServo rightnom = null;

    //team marker flipper
    Servo flipper = null;

    DcMotor pullup = null;

    DcMotor nom = null;

    DcMotor arm = null;

    DcMotor string = null;

//    Behaviour when the motors are stopped

    static final ZeroPowerBehavior ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.BRAKE;
    public void strafe(boolean strafe) {
        FR.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        FL.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.FORWARD);
        BR.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.REVERSE);
        BL.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

    }

    public void delay(int time){
        double startTime = clock.milliseconds();
        while((clock.milliseconds() - startTime < time) && !isStopRequested()){

        }
    }
    public void turnRight(){
        FL.setPower(.4);
        BL.setPower(.4);
        FR.setPower(.4);
        BR.setPower(.4);
    }
    public void turnLeft(){
        FL.setPower(-.4);
        BL.setPower(-.4);
        FR.setPower(-.4);
        BR.setPower(-.4);

    }
    public void goForward(){
        FL.setPower(.9);
        BL.setPower(.9);
        FR.setPower(-.9);
        BR.setPower(-.9);

    }
    public void goBack(){
        FL.setPower(-.9);
        BL.setPower(-.9);
        FR.setPower(.9);
        BR.setPower(.9);
    }
    public void goStop(){
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
}

