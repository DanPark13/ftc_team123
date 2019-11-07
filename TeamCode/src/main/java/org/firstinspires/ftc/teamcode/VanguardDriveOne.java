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
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY&#39;S PATENT RIGHTS ARE
GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

@TeleOp(name="VanguardDriveOne", group ="Concept")
public class VanguardDriveOne extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, grabMotor, slideMotor;
    Servo LeftFoundation, RightFoundation;
    private int positionStay;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        double speed = 1.0;
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");
        
        LeftFoundationInit("LeftFoundationGrip");
        RightFoundationInit("RightFoundationGrip");
        
        grabMotorInit("grabMotor");
        slideMotorInit("slideMotor");
        
        //grabMotor will use encoders
        grabMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabMotor.setPower(0.7);
        
        // slide motor will use encoders
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0.7);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if( gamepad1.left_stick_button && gamepad1.right_stick_button)
                speed = speed - 0.5;
            
            // Drivetrain controls
            if( gamepad1.dpad_right )
                wheelDrive( "LEFT" , speed );
                
            else if( gamepad1.dpad_left )
                wheelDrive( "RIGHT" , speed );
                
            else if( gamepad1.dpad_up )
                wheelDrive( "BACKWARD" , speed );
                
            else if( gamepad1.dpad_down )
                wheelDrive( "FORWARD" , speed );
                
            else if( gamepad1.right_bumper )
                wheelTurn( "RIGHT" , speed);
                
            else if( gamepad1.left_bumper)
                wheelTurn( "LEFT" , speed );
                
            else 
            {
                wheelDrive( "FORWARD" , 0 );
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            // Send calculated power to wheels
            
            //if y is clicked, it will move the foundation
            
            moveFoundation();
            
            //if a or b is clicked, grabber will move
            grabMotor(0.25);
            
            //if right or left trigger, slide motor will move
            slideMotor(0.3);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed: ", "" + speed);
            telemetry.update();
        }
    }
    public void wheelInit(String topLeft, String topRight, String bottomLeft, String bottomRight)
    {
        frontLeftDrive = hardwareMap.get(DcMotor.class, topLeft);
        frontRightDrive = hardwareMap.get(DcMotor.class, topRight);
        backLeftDrive = hardwareMap.get(DcMotor.class, bottomLeft);
        backRightDrive = hardwareMap.get(DcMotor.class, bottomRight);
    }
    public void wheelDrive(String direction,double speed)
    {
        if( direction.equals("FORWARD") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        else if( direction.equals("BACKWARD") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        else if( direction.equals("LEFT") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        else if( direction.equals("RIGHT") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            telemetry.addData("Error" , "Wrong Input for wheelDrive entered.");
            telemetry.update();
        }
        
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
    }
    
    public void wheelTurn(String direction, double speed)
    {
        if(direction.equals("LEFT") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        else if(direction.equals("RIGHT") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            telemetry.addData("Error" , "Wrong Input for wheelTurn entered.");
            telemetry.update();
        }
        
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
    }
    
    public void LeftFoundationInit(String name){
        LeftFoundation = hardwareMap.get(Servo.class, name);
    }
    
    public void RightFoundationInit(String name){
        RightFoundation = hardwareMap.get(Servo.class, name);
    }
    
    
    
    public void moveFoundation(){
        //if y is clicked, it will move the foundation
        if(gamepad1.y)
        {
            LeftFoundation.setPosition(0.0);
            RightFoundation.setPosition(0.75);
        }
        else
        {
            LeftFoundation.setPosition(0.75);
            RightFoundation.setPosition(0.0);
        }
    }
    
    public void grabMotorInit(String name)
    {
        grabMotor = hardwareMap.get(DcMotor.class, name);
    }
    
    public void slideMotorInit(String name)
    {
        slideMotor = hardwareMap.get(DcMotor.class, name);
    }
    
    public void grabMotor(double powa)
    {
        //if a is clicked, grabMotor will move forwards OPEN
        if(gamepad1.a){
            grabMotor.setTargetPosition(grabMotor.getCurrentPosition() + 2240);
        }
        //if b is clicked, grabMotor will move backwards CLOSE
        else if(gamepad1.b){
            grabMotor.setTargetPosition(grabMotor.getCurrentPosition() - 2240);
        }
        else
        {
            grabMotor.setPower(0);
        }
    }
    
    public void slideMotor(double powa)
    {
        //if right trigger is pressed, slideMotor moves forwards
        if(gamepad1.right_trigger >= 0.75)
        {
            slideMotor.setTargetPosition(slideMotor.getCurrentPosition()+200);
            positionStay = slideMotor.getCurrentPosition();
        }
        //if left trigger is pressed, slideMotor moves backwards
        else if(gamepad1.left_trigger >= 0.75)
        {
            slideMotor.setTargetPosition(slideMotor.getCurrentPosition()-200);
            positionStay = slideMotor.getCurrentPosition();
        }
        else
        {
            slideMotor.setTargetPosition(positionStay);
        }
    }
}
