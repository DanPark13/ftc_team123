/*
Copyright 2020 

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
@TeleOp

public class NewOneManDrive extends LinearOpMode 
{
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, slideMotor
    CRServo grabServo;
    Servo LeftFoundation, RightFoundation;
    
    @Override
    public void runOpMode() 
    {
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");
        
        grabServoInit("grabServo");
        slideMotorInit("slideMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
       frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
       
        // run until the end of the match (driver presses STOP)
        
        
            
        while (opModeIsActive()) 
        {
            telemetryUpdate();
            
            DriveTrainOne();
            
            grabServo();
            
            slideDrive();
        }
    }
    
    public void telemetryUpdate()
    {
        
    }
    
    public void wheelInit(String topLeft, String topRight, String bottomLeft, String bottomRight)
    {
        frontLeftDrive = hardwareMap.get(DcMotor.class, topLeft);
        frontRightDrive = hardwareMap.get(DcMotor.class, topRight);
        backLeftDrive = hardwareMap.get(DcMotor.class, bottomLeft);
        backRightDrive = hardwareMap.get(DcMotor.class, bottomRight);
    }
    
    public void DriveTrainOne()
    {
        
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left)
        {
            frontLeftDrive.setPower(gamepad1.left_stick_y);
            frontRightDrive.setPower(-gamepad1.right_stick_y);
            backLeftDrive.setPower(gamepad1.left_stick_y);
            backRightDrive.setPower(-gamepad1.right_stick_y);  
        }
        
        if(gamepad1.dpad_up)
        {
            frontLeftDrive.setPower(-0.6);
            frontRightDrive.setPower(0.5);
            backLeftDrive.setPower(-0.6);
            backRightDrive.setPower(0.5);
        }
        
        // BACKWARD
        else if(gamepad1.dpad_down)
        {
            frontLeftDrive.setPower(0.6);
            frontRightDrive.setPower(-0.5);
            backLeftDrive.setPower(0.6);
            backRightDrive.setPower(-0.5);
        }
        
        // STRAFE RIGHT
        else if(gamepad1.dpad_right)
        {
            frontLeftDrive.setPower(-0.6);
            frontRightDrive.setPower(-0.5);
            backLeftDrive.setPower(0.6);
            backRightDrive.setPower(0.5);
        }
        
        // STRAFE LEFT
        else if(gamepad1.dpad_left)
        {
            frontLeftDrive.setPower(0.6);
            frontRightDrive.setPower(0.5);
            backLeftDrive.setPower(-0.6);
            backRightDrive.setPower(-0.5);
        }
        else if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0)
        {
          
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }
    
    public void grabServoInit(String name)
    {
        grabServo = hardwareMap.get(CRServo.class, name);
    }
    
    public void slideMotorInit(String name)
    {
        slideMotor = hardwareMap.get(DcMotor.class, name);
       
    }
    
    public void grabServo()
    {
        if(gamepad1.left_bumper)
        {
            grabServo.setPower(-1);
        }
        else if(gamepad1.right_bumper)
        {
            grabServo.setPower(1);
        }
        else if(gamepad1.left_bumper == false && gamepad1.right_bumper == false)
        {
            grabServo.setPower(0.0);
        }
    }
    
    public void slideDrive()
    {
        int positionStay = slideMotor.getCurrentPosition();
        //if right trigger is pressed, slideMotor moves forwards
        if(gamepad1.right_trigger >= 0.5)
        {
            slideMotor.setPower(-gamepad1.right_trigger);
        }
        //if left trigger is pressed, slideMotor moves backwards
        else if(gamepad1.left_trigger >= 0.5)
        {
            slideMotor.setPower(gamepad1.left_trigger);
        }
        else
        {
            slideMotor.setPower(0.0);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
