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


public class VanguardDriveOne extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        double speed = 1.0;
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            

            // Drivetrain controls
            if( gamepad1.dpad_right )
            {
                wheelDrive( "RIGHT" , speed );
            }
            else if( gamepad1.dpad_left )
            {
                wheelDrive( "LEFT" , speed );
            }
            else if( gamepad1.dpad_up )
            {
                wheelDrive( "FORWARD" , speed );
            }
            else if( gamepad1.dpad_down )
            {
                wheelDrive( "BACKWARD" , speed );
            }
            else if( gamepad1.right_bumper )
            {
                wheelTurn( "RIGHT" , speed);
            }
            else if( gamepad1.dpad_left )
            {
                wheelTurn( "LEFT" , speed );
            }
            else
            {
                wheelDrive( "FORWARD" , 0 );
            }

            // Send calculated power to wheels
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
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
    public void wheelTurn(String direction,double speed)
    {
        if(direction.equals("RIGHT") )
        {
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        else if(direction.equals("LEFT") )
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
}

    
