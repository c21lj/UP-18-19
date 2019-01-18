package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

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

abstract class JackalopeAutoMode extends LinearOpMode {

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


    //team marker flipper
    Servo flipper = null;
    Servo marker = null;

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
        FL.setPower(.4);
        BL.setPower(.4);
        FR.setPower(-.4);
        BR.setPower(-.4);

    }

    public void goRight(){
        FL.setPower(-.4);
        BL.setPower(.4);
        FR.setPower(-.4);
        BR.setPower(.4);

    }

    public void goLeft(){
        FL.setPower(.4);
        BL.setPower(-.4);
        FR.setPower(.4);
        BR.setPower(-.4);

    }

    public void goBack(){
        FL.setPower(-.4);
        BL.setPower(-.4);
        FR.setPower(.4);
        BR.setPower(.4);
    }
    public void goStop(){
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

//    public void runOpMode() throws InterruptedException {
//        I2cDeviceSynch pixy;
//
//            //our Pixy device
//            //setting up Pixy to the hardware map
//            pixy = hardwareMap.i2cDeviceSynch.get("pixy");
//
//            //setting Pixy's I2C Address
//            pixy.setI2cAddress(I2cAddr.create7bit(0x50));
//
//            //setting Pixy's read window.
//            I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(1, 26, I2cDeviceSynch.ReadMode.REPEAT);
//            pixy.setReadWindow(readWindow);
//
//            //Turns the thing on
//            pixy.engage();
//
//            waitForStart();
//
//            /*
//            Bytes    16-bit word    Description
//            ----------------------------------------------------------------
//            0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
//            2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
//            4, 5     y              signature number
//            6, 7     y              x center of object
//            8, 9     y              y center of object
//            10, 11   y              width of object
//            12, 13   y              height of object
//            */
//
//            while (opModeIsActive()) {
//                //sends all the data to the phone screen
//                telemetry.addData("Byte 0", pixy.read8(0));
//                telemetry.addData("Byte 1", pixy.read8(1));
//                telemetry.addData("Byte 2", pixy.read8(2));
//                telemetry.addData("Byte 3", pixy.read8(3));
//                telemetry.addData("Byte 4", pixy.read8(4));
//                telemetry.addData("Byte 5", pixy.read8(5));
//                telemetry.addData("Byte 6", pixy.read8(6));
//                telemetry.addData("Byte 7", pixy.read8(7));
//                telemetry.addData("Byte 8", pixy.read8(8));
//                telemetry.addData("Byte 9", pixy.read8(9));
//                telemetry.addData("Byte 10", pixy.read8(10));
//                telemetry.addData("Byte 11", pixy.read8(11));
//                telemetry.addData("Byte 12", pixy.read8(12));
//                telemetry.addData("Byte 13", pixy.read8(13));
//                telemetry.update();
//
//                //Turn in circles until it detects the thing
//                FL.setPower(.05);
//                BL.setPower(.05);
//                FR.setPower(.05);
//                BR.setPower(.05);
//
//                //I'm not sure if this section will work because so far I'm not sure what the data means and why there are two different things
//                if( (pixy.read8(6)==0) || (pixy.read8(7)==0) ){
//                    //If pixy signature is in the center ----> drive forward
//                    //I haven't actually set the signatures yet so welp
//                    //Uh this should allow the robot to push the cube out of the thing
//                    FL.setPower(.05);
//                    BL.setPower(.05);
//                    FR.setPower(-.05);
//                    BR.setPower(-.05);
//                }
//            }
//        }
    }

