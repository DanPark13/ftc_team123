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

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "IMU DRIVE FIX", group = "Sensor")

public class IMU_LEARN extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, slideMotor, grabMotor, tapeMotor;
    private double speed = 1;
    Servo autoGrabServo;
    CRServo grabServo, capstoneServo;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    
    public double fLPDrive, fRPDrive, bLPDrive, bRPDrive;
    public double fLPStrafe, fRPStrafe, bLPStrafe, bRPStrafe;
    
    public boolean strafeCorrect;
    public boolean driveCorrect;
    
    public boolean correctStrafe = false;
    
    double desiredHeading;
    double currentHeading;
    double headingDifference;
    double headingPercentage;
    
    double fLP = 0.5;
    double fRP = 0.5;
    double bLP = 0.5;
    double bRP = 0.5;
    
    double nfLP = 0;
    double nfRP = 0;
    double nbLP = 0;
    double nbRP = 0;
    
    double morePowerMultiplier;
    double lessPowerMultiplier;
    
    boolean Left = false;
    boolean Right = true;
    double wheelAlterationPercentage;
    
    int position = 0;
    

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {
        
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");
        grabServoInit("grabServo");
        slideMotorInit("slideMotor");
        grabMotorInit("grabMotor");
        autoGrabServoInit("autoGrabServo");
        tapeMotorInit("tapeMotor");
        capstoneServoInit("capstoneServo");
        
       frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
       grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            DriveTrainOne();
            
            grabServo();
            
            slideDrive();
            
            setDesiredDirection();
            
            telemetry.addData("Grab Motor Position", grabMotor.getCurrentPosition());
            
            skystoneGrab();
            
            tapePark();
            
            moveCapstone();
            
            /*
            SetDirection();
            recordHeading();
            setHeading();
            findHeadingDifference();
            findHeadingPercentage();
            findDrivingPercentage();
            modifyWheelPower();
            */
            
            telemetry.addData("Marker Servo Position", position);
            telemetry.addData("Desired Heading", desiredHeading );
            telemetry.addData("Current Heading", currentHeading );
            telemetry.addData("Difference in Heading", headingDifference );
            telemetry.addData("Heading Percentage", headingPercentage );
            telemetry.addData("Wheel Percentage" , wheelAlterationPercentage);
            telemetry.addData("More Power Multiplier", morePowerMultiplier );
            telemetry.addData("Less Power Multiplier", lessPowerMultiplier );
            telemetry.addData("Original FL, FR, BL, BR", .5 + "," + .5 + "," + .5 + "," + .5);
            telemetry.addData("New FL, FR, BL, BR", nfLP + "," + nfRP + "," + nbLP + "," + nbRP);
            telemetry.update();
            
        }
    }
    
    public void setDesiredDirection()
    {
        if(gamepad1.x == true)
        {
            setHeading();
            correctStrafe = true;
        }
        else if(gamepad1.a == true)
        {
            correctStrafe = false;
        }
        if( correctStrafe == false)
        {
            setHeading();
        }
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
            fLPDrive = gamepad1.left_stick_y*.75;
            fRPDrive = -gamepad1.right_stick_y*.75;
            bLPDrive = gamepad1.left_stick_y*.75;
            bRPDrive = -gamepad1.right_stick_y*.75;
            
            fLPStrafe = 0;
            fRPStrafe = 0;
            bLPStrafe = 0;
            bRPStrafe = 0;
            
            driveCorrect = true;
            strafeCorrect = false;
        }
        if(gamepad1.dpad_up)
        {
            fLPDrive = -0.5;
            fRPDrive = 0.5;
            bLPDrive = -0.5;
            bRPDrive = 0.5;
            
            driveCorrect = true;
            strafeCorrect = false;
        }
        // BACKWARD
        else if(gamepad1.dpad_down)
        {
            fLPDrive = 0.5;
            fRPDrive = -0.5;
            bLPDrive = 0.5;
            bRPDrive = -0.5;
            
            driveCorrect = true;
            strafeCorrect = false;
        }
        // STRAFE RIGHT
        else if(gamepad1.dpad_right)
        {
            fLPStrafe = -fLP;
            fRPStrafe = -fRP;
            bLPStrafe = bLP;
            bRPStrafe = bRP;
            
            driveCorrect = false;
            strafeCorrect = true;
            
            if(correctStrafe == true)
            {
                findHeadingDifference();
                findHeadingPercentage();
                findDrivingPercentage();
                modifyWheelPower(); 
                
            }
            else if (correctStrafe == false)
            {
                 nfLP = -0.5;
                 nfRP = -0.5;
                 nbLP = 0.5;
                 nbRP = 0.5;
            }
        }
        // STRAFE LEFT
        else if(gamepad1.dpad_left)
        {
            fLPStrafe = fLP;
            fRPStrafe = fRP;
            bLPStrafe = -bLP;
            bRPStrafe = -bRP;
            
            driveCorrect = false;
            strafeCorrect = true;
            
                
            if(correctStrafe == true)
            {
                findHeadingDifference();
                findHeadingPercentage();
                findDrivingPercentage();
                modifyWheelPower(); 
                
            }
            else if (correctStrafe == false)
            {
                 nfLP = 0.5;
                 nfRP = 0.5;
                 nbLP = -0.5;
                 nbRP = -0.5;
            }
        }
        else if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0)
        {
            fLPDrive = 0;
            fRPDrive = 0;
            bLPDrive = 0;
            bRPDrive = 0;
            
            nfLP = 0;
            nfRP = 0;
            nbLP = 0;
            nbRP = 0;
        }
        
        frontLeftDrive.setPower(fLPDrive + nfLP);
        frontRightDrive.setPower(fRPDrive + nfRP);
        backLeftDrive.setPower(bLPDrive + nbLP);
        backRightDrive.setPower(bRPDrive + nbRP);
    }
    
    public void grabServoInit(String name)
    {
        grabServo = hardwareMap.get(CRServo.class, name);
    }
    
    public void slideMotorInit(String name)
    {
        slideMotor = hardwareMap.get(DcMotor.class, name);
       
    }
    
    public void grabMotorInit(String name)
    {
        grabMotor = hardwareMap.get(DcMotor.class, name);
    }
    
    public void autoGrabServoInit(String name)
    {
        autoGrabServo = hardwareMap.get(Servo.class, name);
    }
    
    public void tapeMotorInit(String name)
    {
        tapeMotor = hardwareMap.get(DcMotor.class, name);
    }
    
    public void capstoneServoInit(String name)
    {
        capstoneServo = hardwareMap.get(CRServo.class, name);
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
        else if(gamepad2.dpad_left)
        {
            grabServo.setPower(-1);
        }
        else if(gamepad2.dpad_right)
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
        else if(gamepad2.right_trigger >= 0.5)
        {
            slideMotor.setPower(-gamepad2.right_trigger);
        }
        //if left trigger is pressed, slideMotor moves backwards
        else if(gamepad2.left_trigger >= 0.5)
        {
            slideMotor.setPower(gamepad2.left_trigger);
        }
        else
        {
            slideMotor.setPower(0.0);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    public void skystoneGrab()
    {
        if(gamepad2.a)
        {
            grabMotor.setTargetPosition(200);
            grabMotor.setPower(0.75);
            grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(grabMotor.isBusy())
            {
                    
            }
            grabMotor.setPower(0); 
        }
        else if(gamepad2.y)
        {
            grabMotor.setTargetPosition(800);
            grabMotor.setPower(-0.75);
            grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(grabMotor.isBusy())
            {
                    
            }
            grabMotor.setPower(0); 
        }    
        else if(gamepad2.b)
        {
            autoGrabServo.setPosition(0);
        }
        else if(gamepad2.x)
        {
            autoGrabServo.setPosition(1);
        }
        else
        {
            grabMotor.setPower(0); 
        }
    }
    
    void tapePark()
    {
        if(gamepad2.dpad_up)
        {
            tapeMotor.setPower(-1);
        }
        else if(gamepad2.dpad_down)
        {
            tapeMotor.setPower(0.5);
        }
        else
        {
            tapeMotor.setPower(0);
            tapeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    void moveCapstone()
    {
        if(gamepad2.left_bumper)
        {
            capstoneServo.setPower(-1);
        }
        else if(gamepad2.right_bumper)
        {
            capstoneServo.setPower(1);
        }
        else
        {
           capstoneServo.setPower(0);
        }
    }

    void SetDirection()
    {
        if(gamepad1.dpad_left)
        {
            boolean Left = true;
        boolean Right = false;
        telemetry.addData("Going","Left" );
        }
        if(gamepad1.dpad_right)
        {
            boolean Left = false;
        boolean Right = true;
        telemetry.addData("Going","Right" );
        }
    }
    void recordHeading()
    {
        
    }
    void setHeading()
    {
            desiredHeading = currentHeading;
        
    }
    void findHeadingDifference()
    {
        headingDifference = currentHeading - desiredHeading;
    }
    void findHeadingPercentage()
    {
        headingPercentage = headingDifference / 90;
    }
    void findDrivingPercentage()
    {
        wheelAlterationPercentage = Math.abs(headingPercentage);
    }
    void modifyWheelPower()
    {
        morePowerMultiplier = 1 + wheelAlterationPercentage;
        lessPowerMultiplier = 1 - wheelAlterationPercentage;
        
        if(headingDifference < 0)
        {
            nfLP = fLP*morePowerMultiplier;
            nfRP = fRP*lessPowerMultiplier;
            nbLP = bLP*lessPowerMultiplier;
            nbRP = bRP*morePowerMultiplier;
            if(gamepad1.dpad_left == true)
            {
                nbLP = -nbLP;
                nbRP = -nbRP;
            }
            else if(gamepad1.dpad_right == true)
            {
                nfLP = -nfLP;
                nfRP = -nfRP;
            }
        }
        else if(headingDifference > 0)
        {
            nfLP = fLP*lessPowerMultiplier;
            nfRP = fRP*morePowerMultiplier;
            nbLP = bLP*morePowerMultiplier;
            nbRP = bRP*lessPowerMultiplier;
            if(gamepad1.dpad_left == true)
            {
                nbLP = -nbLP;
                nbRP = -nbRP;
            }
            else if(gamepad1.dpad_right == true)
            {
                nfLP = -nfLP;
                nfRP = -nfRP;
            }
        }
        
        else if (headingDifference == 0)
        {
            nfLP = 0.5;
            nfRP = 0.5;
            nbLP = -0.5;
            nbRP = -0.5;
        }
        
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    currentHeading = angles.firstAngle;
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
