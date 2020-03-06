/*
Copyright 2019 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name="EncoderTest")

public class EncoderTest extends LinearOpMode 
{
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, slideMotor;
    private double speed = 1.0;
    CRServo grabServo;

    @Override
    public void runOpMode() throws InterruptedException
    {
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");
        grabServoInit("grabServo");
        slideMotorInit("slideMotor");
        
        telemetry.addData("Status", "Initialized");
    
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            
        // http://192.168.49.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/VanguardAutonomousOne.java
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        /*
        drive(2250, 1, "FORWARD");
        grabClose(2);
        drive(750, 1, "BACKWARD");
        turn(1000, 1, "LEFT");
        */
        while (opModeIsActive())
        {
            telemetry.addData("FL", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR", backRightDrive.getCurrentPosition());
            telemetry.update();
        }   
    }
    
    // Wheel Configuration Function
    public void wheelInit(String topLeft, String topRight, String bottomLeft, String bottomRight)
    {
        frontLeftDrive = hardwareMap.get(DcMotor.class, topLeft);
        frontRightDrive = hardwareMap.get(DcMotor.class, topRight);
        backLeftDrive = hardwareMap.get(DcMotor.class, bottomLeft);
        backRightDrive = hardwareMap.get(DcMotor.class, bottomRight);
    }
    
    // Grab System Configuration Function
    public void grabServoInit(String name)
    {
        grabServo = hardwareMap.get(CRServo.class, name);
    }
    
    // Slide System Configuration Function
    public void slideMotorInit(String name)
    {
        slideMotor = hardwareMap.get(DcMotor.class, name);
    }
    
    // Strafe movements
    public void drive(int pos, double speed, String direction) 
    {
        int fL = 1;
        int fR = 1;
        int bL = 1;
        int bR = 1;
        
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        if(direction == "FORWARD")
        {
            fL *= -1;
            bL *= -1;
        }
        else if(direction == "LEFT")
        {
            bL *= -1;
            bR *= -1;
        }
        else if(direction == "RIGHT")
        {
            fL *= -1;
            fR *= -1;
        }
        else if(direction == "BACKWARD")
        {
            fR *= -1;
            bR *= -1;
        }
        
        frontLeftDrive.setTargetPosition(pos*fL);
        frontRightDrive.setTargetPosition(pos*fR);
        backLeftDrive.setTargetPosition(pos*bL);
        backRightDrive.setTargetPosition(pos*bR);
        
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftDrive.setPower(speed*fL);
        frontRightDrive.setPower(speed*fR);
        backLeftDrive.setPower(speed*bL);
        backRightDrive.setPower(speed*bR);
        
        while ((frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy()))
        {
            telemetry.addData("Left Front:", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Right Front:", frontRightDrive.getCurrentPosition());
            telemetry.addData("Left Back:", backLeftDrive.getCurrentPosition());
            telemetry.addData("Right Back:", backRightDrive.getCurrentPosition());
            telemetry.update();
        }
        
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    
    // Turn movements
    public void turn(int pos, double speed, String direction) 
    {
        int fL = 1;
        int fR = 1;
        int bL = 1;
        int bR = 1;
        
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // change below
        if(direction == "RIGHT")
        {
            fL = -1;
            fR = -1;
            bL = -1;
            bR = -1;
        }
        
        frontLeftDrive.setTargetPosition(pos * fL);
        frontRightDrive.setTargetPosition(pos * fR);
        backLeftDrive.setTargetPosition(pos * bL);
        backRightDrive.setTargetPosition(pos * bR);
        
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftDrive.setPower(speed*fL);
        frontRightDrive.setPower(speed*fR);
        backLeftDrive.setPower(speed*bL);
        backRightDrive.setPower(speed*bR);
        
        while ((frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy()))
        {
            telemetry.addData("Left Front:", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Right Front:", frontRightDrive.getCurrentPosition());
            telemetry.addData("Left Back:", backLeftDrive.getCurrentPosition());
            telemetry.addData("Right Back:", backRightDrive.getCurrentPosition());
            telemetry.update();
        }
        
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    
    public void grabOpen(int seconds)
    {
        grabServo.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void grabClose(int seconds)
    {
        grabServo.setPower(-.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void slideUp(int seconds)
    {
        slideMotor.setPower(-.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void slideDown(int seconds)
    {
        slideMotor.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
}
