package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PixyCam", group="PixyCam")
public class PixyCam extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
            I2cDeviceSynch pixy;

            //our Pixy device
            //setting up Pixy to the hardware map
            pixy = hardwareMap.i2cDeviceSynch.get("pixy");

            //setting Pixy's I2C Address
            pixy.setI2cAddress(I2cAddr.create7bit(0x50));

            //setting Pixy's read window. You'll want these exact parameters, and you can reference the SDK Documentation to learn more
            I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(1, 26, I2cDeviceSynch.ReadMode.REPEAT);
            pixy.setReadWindow(readWindow);

            //required to "turn on" the device
            pixy.engage();

            waitForStart();

            while (opModeIsActive()) {
                /*
                Bytes    16-bit word    Description
                ----------------------------------------------------------------
                0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
                2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
                4, 5     y              signature number
                6, 7     y              x center of object
                8, 9     y              y center of object
                10, 11   y              width of object
                12, 13   y              height of object
                */
                //send every byte of data that we can to the phone screen
                telemetry.addData("Object1", pixy.read8(0));
                telemetry.addData("Object2", pixy.read8(1));
                telemetry.addData("Checksum1", pixy.read8(2));
                telemetry.addData("Checksum2", pixy.read8(3));
                telemetry.addData("Signature1", pixy.read8(4));
                telemetry.addData("Signature2", pixy.read8(5));
                telemetry.addData("XCoordinate1", pixy.read8(6));
                telemetry.addData("XCoordinate2", pixy.read8(7));
                telemetry.addData("YCoordinate1", pixy.read8(8));
                telemetry.addData("YCoordinate2", pixy.read8(9));
                telemetry.addData("Width1", pixy.read8(10));
                telemetry.addData("Width2", pixy.read8(11));
                telemetry.addData("Height1", pixy.read8(12));
                telemetry.addData("Height2", pixy.read8(13));
                telemetry.update();

                //Turn in circles until it detects the thing (Theres no motor code yet)

                if ( ( (pixy.read8(6)==0) || (pixy.read8(7)==0) ) && (pixy.read8(8)==0) || (pixy.read8(9)==0) ) {
                    //If pixy signature is in the center ----> something
                    //Most likely just drive forward
                    //I haven't actually set the signatures yet so welp
                }
            }
        }
    }