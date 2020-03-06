/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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

//


@Autonomous(name="Blue Two", group ="Concept")

public class BlueTwo extends LinearOpMode {
    
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, slideMotor, grabMotor, tapeMotor;
    private double speed = 1.0;
    CRServo grabServo;
    Servo autoGrabServo;



    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    public double currentHeading;
    public double startingHeading;
    public double headingDifference;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AQNhJYP/////AAABmSlHO0hxYUtiqjSb5PXofD83A37XD0kWwLbnMny90SNbQrvQkG5hbKejehVWmHsWOgnLjWjfLme+8T68dqDUen2qBaAN4ueO1WrTOjCYHDnKlU3N3rJqRO9xqRuw61t6xM201ZtMQa2Y8b1BooS/H1rXy6WlCXQa4qpUmo2R5nWqNCyZKPNIdkanXdbLfGf+GB/aPRDVix1WWPEfdRrKC/HZVeg4cVCsjVTXqcXdxqgnBbPgFj8XhcSC4Zd0VpptBUiwog5RDXsIcErZy6qJKTyQgEfjcQtt6U13YXCbb+hU4T3GIJMnb1l+IceFqCwsSTjv/7qClhWlP7pTSWTcX5yZG44U7H7CyWZ55x0SRgFs";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    
    ElapsedTime timer = new ElapsedTime();
    
    String skystonePosition = "";
    
    double currentTime = 0;
    
    @Override public void runOpMode() 
    {
        
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersIMU  = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        // Set up our telemetry dashboard
        composeTelemetry();
        
        
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        startingHeading = currentHeading;
        
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        
        wheelInit("frontLeft", "frontRight", "backLeft", "backRight");
        grabServoInit("grabServo");
        slideMotorInit("slideMotor");
        grabMotorInit("grabMotor");
        autoGrabServoInit("autoGrabServo");
        tapeMotorInit("tapeMotor");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        
        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();
        
        timer.reset();
        
        /*
            Put functions here
        */
        drive(750, 0.75, "BACKWARD");

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        // Wheel Configuration Function

        targetsSkyStone.activate();
        while (!isStopRequested()) 
        {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    
                    if(trackable.getName().equals("Stone Target"))
                    {
                        telemetry.addLine("Stone Target is Visible");
                    }
                    
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know)
        
            if (targetVisible) 
            {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                        
                double yPosition = translation.get(1);
                if(yPosition <= 0)
                {
                    skystonePosition = "left";
                    telemetry.speak("The Block is in The Left Position");
                    telemetry.update();
                }
                else
                {
                    skystonePosition = "center";
                    telemetry.speak("The Block is in The Middle Position");
                }
                
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else 
            {
                skystonePosition = "right";
            }
            telemetry.addData("Skystone Position:", skystonePosition);
            
            currentTime = timer.time();
            telemetry.addData("Timer", currentTime);
            telemetry.update();
            
            if(currentTime >= 3.0)
            {
               
                if(skystonePosition == "left")
                {
                 
                    
                    //First block
                    skystoneGrab(500, -0.75);
                    drive(275, 0.75, "RIGHT");
                    drive(1000, 0.5, "BACKWARD");
                    autoGrab(0, 1);
                    drive(500, 1, "FORWARD");

                    skystoneGrab(10, 0.75);
                    
                    turnDeg(900, 0.25, "LEFT", 80);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    
                    
                    drive(2700, 0.5, "BACKWARD");
                    //turn(825, 0.75, "RIGHT");
                    /*
                    drive(500, 1, "BACKWARD");
                    */
                     skystoneGrab(500, -0.75);
                    autoGrab(1000, 1);
                    
                    
                    //turn(825, 0.75, "LEFT");
                    drive(4450, 0.5, "FORWARD");
                    
                    //Go for second block
                    
                    turnDeg(940, 0.25, "RIGHT", 80);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    
                    
                    //skystoneGrab(500, -0.75);
                    drive(550, 1, "BACKWARD");
                    autoGrab(0, 1);
                    
                    skystoneGrab(10, 0.75);
                    drive(585, 1, "FORWARD");
                    
                    turnDeg(900, 0.75, "LEFT", 80);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    
                    
                    drive(4000, 0.65, "BACKWARD");
                    skystoneGrab(500, -0.75);
                    autoGrab(1000, 1);
                    drive(800, 1, "FORWARD");
                    
                    
                    stop();
                }
                else if(skystonePosition == "center")
                {
                  
                    //First block
                    skystoneGrab(500, -0.75);
                    drive(275, 0.75, "LEFT");
                    drive(1000, 0.75, "BACKWARD");
                    autoGrab(0, 1);
                    drive(500, 1, "FORWARD");

                    skystoneGrab(10, 0.75);
                    
                    turnDeg(900, 0.25, "LEFT", 80);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    startingHeading = currentHeading;
                    
                    drive(2800, 0.65, "BACKWARD");
                    //turn(825, 0.75, "RIGHT");
                    /*
                    drive(500, 1, "BACKWARD");
                    */
                     skystoneGrab(500, -0.75);
                    autoGrab(1000, 1);
                    
                    
                    //turn(825, 0.75, "LEFT");
                    drive(4405, 0.65, "FORWARD");
                    
                    //Go for second block
                    
                    turnDeg(900, .75, "RIGHT", 75);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    startingHeading = currentHeading;
                    
                    //skystoneGrab(500, -0.75);
                    drive(500, 1, "BACKWARD");
                    autoGrab(0, 1);
                    
                    skystoneGrab(10, 0.75);
                    drive(575, 1, "FORWARD");
                    
                    turnDeg(900, 0.25, "LEFT", 80);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    startingHeading = currentHeading;
                    
                    drive(4405, 0.65, "BACKWARD");
                    skystoneGrab(500, -0.75);
                    autoGrab(1000, 1);
                    drive(3500, 1, "FORWARD");
                    
                    tapePark(2000);
                   
                    stop();
                }
                else
                {
                     targetsSkyStone.deactivate();
                    
                    //First block
                    skystoneGrab(500, -0.75);
                    drive(750, 0.75, "LEFT");
                    drive(1000, 0.75, "BACKWARD");
                    autoGrab(0, 1);
                    drive(500, 1, "FORWARD");

                    skystoneGrab(10, 0.75);
                    
                    turnDeg(900, 0.80, "LEFT", 75);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    startingHeading = currentHeading;
                    
                    drive(2900, 0.65, "BACKWARD");
                    //turn(825, 0.75, "RIGHT");
                    /*
                    drive(500, 1, "BACKWARD");
                    */
                     skystoneGrab(500, -0.75);
                    autoGrab(1000, 1);
                    
                    
                    //turn(825, 0.75, "LEFT");
                    drive(4705, 0.65, "FORWARD");
                    
                    //Go for second block
                    
                    turnDeg(900, 0.75, "RIGHT", 75);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    startingHeading = currentHeading;
                    
                    //skystoneGrab(500, -0.75);
                    drive(500, 1, "BACKWARD");
                    autoGrab(0, 1);
                    
                    skystoneGrab(10, 0.75);
                    drive(575, 1, "FORWARD");
                    
                    turnDeg(900, 0.75, "LEFT", 75);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    startingHeading = currentHeading;
                    
                    drive(4750, 0.65, "BACKWARD");
                    skystoneGrab(500, -0.75);
                    autoGrab(1000, 1);
                    drive(3500, 1, "FORWARD");
                    
                    tapePark(200);
                    sleep(1000);
                    tapePark(200);
                    sleep(1000);
                    tapePark(200);
                    sleep(1000);
                    tapePark(200);
                    stop();
               
                }
            }
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
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
        
        while ((frontLeftDrive.isBusy()))
        { double cH = currentHeading;
            if(direction == "FORWARD")
            {
                //Going Right
               if (currentHeading < startingHeading)
               {
                    frontLeftDrive.setPower((speed*bR) * (1 - (Math.abs(cH)/22.5)));
                    backLeftDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
                    
                    frontRightDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
                    backRightDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
               }
               
               //Going Left
               else if (currentHeading > startingHeading)
               {
                    frontLeftDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
                    backLeftDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
                    
                    frontRightDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
                    backRightDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
               }
            }
            else if(direction == "BACKWARD")
            {
                //Going Right
               if (currentHeading < startingHeading)
               {
                    frontLeftDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
                    backLeftDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
                    
                    frontRightDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
                    backRightDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
               }
               
               //Going Left
               else if (currentHeading > startingHeading)
               {
                    frontLeftDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
                    backLeftDrive.setPower((speed*bR) * (1 - (Math.abs((cH/22.5)))));
                    
                    frontRightDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
                    backRightDrive.setPower((speed*bR) * (1 + (Math.abs((cH/22.5)))));
               }
            }
        }
        
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    
    public void turn90Deg(int pos, double speed, String direction)
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
        
        while ((frontLeftDrive.isBusy() ))
        {
            
        }
        
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void turnDeg(int pos, double speed, String direction, double degreeDifference)
    {
        
        double finishingHeading = startingHeading + degreeDifference;

        if(direction == "LEFT")
        {
            int fL = 1;
        int fR = 1;
        int bL = 1;
        int bR = 1;
        
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
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
        
        while(currentHeading < finishingHeading )
        {
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("startingHeading", startingHeading);
            telemetry.addData("Final Heading", finishingHeading);
            telemetry.update();
        }
        }
        
        // change below
        else if(direction == "RIGHT")
        {
            int fL = -1;
            int fR = -1;
            int bL = -1;
            int bR = -1;
            finishingHeading = currentHeading - degreeDifference;
        
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
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
        
            while(currentHeading > finishingHeading )
        {
            
            
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("startingHeading", startingHeading);
            telemetry.addData("Final Heading", finishingHeading);
            telemetry.update();
            
        }
        }
        
        startingHeading = currentHeading - finishingHeading;
        
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
        
        while ((frontLeftDrive.isBusy() ))
        {
            
        }
        
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    
    public void skystoneGrab(int position, double power)
    {
        grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotor.setTargetPosition(position);
        grabMotor.setPower(power);
        grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(grabMotor.isBusy())
        {
                    
        }
        grabMotor.setPower(0);    
    }
    
    public void autoGrab(int position, int seconds)
    {
        autoGrabServo.setPosition(position);
        sleep(seconds*1000);
    }
    
    public void grabOpen(int seconds)
    {
        grabServo.setPower(-1);
        
        sleep(seconds);
    }
    
    public void grabClose(int seconds)
    {
        grabServo.setPower(1);
        
        sleep(seconds);
    }
    
    public void slideUp(int seconds, double power)
    {
        slideMotor.setPower(-power);
        
        sleep(seconds);
    }
    
    public void slideDown(int seconds, double power)
    {
        slideMotor.setPower(power);
        
        sleep(seconds);
    }
    
    void tapePark(int seconds)
    {
        tapeMotor.setPower(-1);
        sleep(seconds);
        tapeMotor.setPower(0);
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
        sleep(seconds);
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
