package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class BasicDriveTrainHardware {

    //---------//
    // Vuforia //
    //---------//

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
    private static final String VUFORIA_KEY =
            "AdlTbyT/////AAABmRzwXqB2QkgjoGVrNLf2cn1fVKNUyrJY52lWZSj6LLjLBbboeY6C+yHcl6Z2IktdtHNUkPoYp4NCpGu5Gi4WJZ+LYITD/iHVbH+AcfUijOwAtAlyczB7zogzmQ4cTwL8f71iwtt6tn1zb9hhFL32l3nEmeqw4wE0j/c5Cbw8oObewGJkPNBtcVNdsu8fGw8MxCYgXL+JgvTvY9UXtcl4vt9dlW/wwGo5oScv5iuH3gFVQfQSg88YdT7VEuPGmY1eXMEwlpLllzvpCNueMnR7ZzbZHJS6/JGPIrCwiAzTOQUTtha8/9doDiR7wPfKD6h0+WQSh8nbPFlxETcIm9h+DM6Fq7/0tIaN2cRkSqPGRFyy";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
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

    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    float PhoneDisplacementForward = 6;
    float PhoneDisplacementVertical = 5;
    float PhoneDisplacementLeft = 0;
    float phoneXRotate    = 0;
    float phoneYRotate    = 0;
    float phoneZRotate    = 0;

    public VuforiaTrackables targetsSkyStone;
    public VuforiaTrackables navTargets;
    public VuforiaTrackables stoneTarget;

    public FieldPosVuf fieldTracker;
    public FieldPosVuf stoneTracker;

    //-------------//
    // Other Stuff //
    //-------------//

    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor RearLeft = null;
    public DcMotor RearRight = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public BasicDriveTrainHardware(){}

    public void init(HardwareMap ahwMap, Telemetry telemetry){
        hwMap = ahwMap;

        //---------//
        // Vuforia //
        //---------//

        try {
            /*
             * Retrieve the camera we are to use.
             */
            webcamName = hwMap.get(WebcamName.class, "Webcam 1");

            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
             * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
             */
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
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
            targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            navTargets = (VuforiaTrackables)targetsSkyStone.subList(1,12);
            stoneTarget = (VuforiaTrackables)targetsSkyStone.subList(0,0);

            // Only stoneTarget will not be included
            VuforiaTrackable skyStone = stoneTarget.get(0);
            stoneTarget.setName("Stone Target");


            VuforiaTrackable blueRearBridge = navTargets.get(0);
            blueRearBridge.setName("Blue Rear Bridge");

            VuforiaTrackable redRearBridge = navTargets.get(1);
            redRearBridge.setName("Red Rear Bridge");

            VuforiaTrackable redFrontBridge = navTargets.get(2);
            redFrontBridge.setName("Red Front Bridge");

            VuforiaTrackable blueFrontBridge = navTargets.get(3);
            blueFrontBridge.setName("Blue Front Bridge");

            VuforiaTrackable red1 = navTargets.get(4);
            red1.setName("Red Perimeter 1");

            VuforiaTrackable red2 = navTargets.get(5);
            red2.setName("Red Perimeter 2");

            VuforiaTrackable front1 = navTargets.get(6);
            front1.setName("Front Perimeter 1");

            VuforiaTrackable front2 = navTargets.get(7);
            front2.setName("Front Perimeter 2");

            VuforiaTrackable blue1 = navTargets.get(8);
            blue1.setName("Blue Perimeter 1");

            VuforiaTrackable blue2 = navTargets.get(9);
            blue2.setName("Blue Perimeter 2");

            VuforiaTrackable rear1 = navTargets.get(10);
            rear1.setName("Rear Perimeter 1");

            VuforiaTrackable rear2 = navTargets.get(11);
            rear2.setName("Rear Perimeter 2");

            // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
            // Rotated it to to face forward, and raised it to sit on the ground correctly.
            // This can be used for generic target-centric approach algorithms
            skyStone.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            //Set the position of the bridge support targets with relation to origin (center of field)
            blueFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

            blueRearBridge.setLocation(OpenGLMatrix
                    .translation(bridgeX, bridgeY, bridgeZ)
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
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

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
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            rear2.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(PhoneDisplacementForward * mmPerInch, PhoneDisplacementLeft * mmPerInch, PhoneDisplacementVertical * mmPerInch)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : targetsSkyStone) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }

            fieldTracker = new FieldPosVuf(navTargets, robotFromCamera);
            stoneTracker = new FieldPosVuf(stoneTarget, robotFromCamera);

        }catch(Exception e){
            telemetry.addData("There was a problem with vuforia!",e.toString());
        }

        //-------------//
        // Other Stuff //
        //-------------//

        try {
            FrontLeft = hwMap.get(DcMotor.class, "front_left");
            FrontRight = hwMap.get(DcMotor.class, "front_right");
            RearLeft = hwMap.get(DcMotor.class, "rear_left");
            RearRight = hwMap.get(DcMotor.class, "rear_right");

            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            RearLeft.setDirection(DcMotor.Direction.FORWARD);
            RearRight.setDirection(DcMotor.Direction.FORWARD);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearLeft.setPower(0);
            RearRight.setPower(0);
        }catch(Exception e){
            telemetry.addData("There was a problem with setting up the motors!",e.toString());
        }
    }


}
