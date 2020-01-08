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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 
@Autonomous(name="BlueFoundationTimeAutonomous", group ="Concept")

public class BlueFoundationTimeAutonomous extends LinearOpMode 
{
    // Declare the motor and servo variables
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, slideMotor;
    CRServo grabServo;
    
    @Override
    public void runOpMode() 
    {
        // Telemetry Updates
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initializes all motors and servos
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");
        grabServoInit("grabServo");
        slideMotorInit("slideMotor");
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //reposition foundation 
        //time elapsed prediction: 15 seconds
        left(1);
        stopMovement(1);
        slideUp(1);
        stopMovement(1);
        forward(2);
        stopMovement(1);
        slideDown(1);
        stopMovement(1);
        backward(3);
        stopMovement(1);
        slideUp(2);
        stopMovement(1);
        right(3);
        stopMovement(1);
        
        //move under bridge
        //time elapsed prediction: 11 seconds
        slideUp(1);
        stopMovement(1);
        left(1);
        stopMovement(1);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) 
        {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
    
    // Initialize the wheels
    public void wheelInit(String topLeft, String topRight, String bottomLeft, String bottomRight)
    {
        frontLeftDrive = hardwareMap.get(DcMotor.class, topLeft);
        frontRightDrive = hardwareMap.get(DcMotor.class, topRight);
        backLeftDrive = hardwareMap.get(DcMotor.class, bottomLeft);
        backRightDrive = hardwareMap.get(DcMotor.class, bottomRight);
    }
    
    public void backward(int seconds)
    {
        frontLeftDrive.setPower(-.5);
        frontRightDrive.setPower(.5);
        backLeftDrive.setPower(-.5);
        backRightDrive.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void forward(int seconds)
    {
        frontLeftDrive.setPower(.5);
        frontRightDrive.setPower(-.5);
        backLeftDrive.setPower(.5);
        backRightDrive.setPower(-.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void left(int seconds)
    {
        frontLeftDrive.setPower(.5);
        frontRightDrive.setPower(.5);
        backLeftDrive.setPower(-.5);
        backRightDrive.setPower(-.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void right(int seconds)
    {
        frontLeftDrive.setPower(-.5);
        frontRightDrive.setPower(-.5);
        backLeftDrive.setPower(.5);
        backRightDrive.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void turnLeft(int seconds)
    {
        frontRightDrive.setPower(.5);
        backRightDrive.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void turnRight(int seconds)
    {
        frontLeftDrive.setPower(.5);
        backLeftDrive.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void turnLeftCenter(int seconds)
    {
        frontLeftDrive.setPower(.5);
        frontRightDrive.setPower(-.5);
        backLeftDrive.setPower(-.5);
        backRightDrive.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void turnRightCenter(int seconds)
    {
        frontLeftDrive.setPower(-.5);
        frontRightDrive.setPower(.5);
        backLeftDrive.setPower(.5);
        backRightDrive.setPower(-.5);
        
        seconds *= 1000;
        sleep(seconds);
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
        slideMotor.setPower(.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    public void slideDown(int seconds)
    {
        slideMotor.setPower(-.5);
        
        seconds *= 1000;
        sleep(seconds);
    }
    
    // stop all movement
    public void stopMovement(int seconds)
    {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        grabServo.setPower(0);
        slideMotor.setPower(0);
        
        seconds *= 1000;
        sleep(seconds);
    
    }
    
    // Initalize the GrabServo
    public void grabServoInit(String name)
    {
        grabServo = hardwareMap.get(CRServo.class, name);
    }
    
    // Intialize the SlideMotor
    public void slideMotorInit(String name)
    {
        slideMotor = hardwareMap.get(DcMotor.class, name);
    }
    
}
