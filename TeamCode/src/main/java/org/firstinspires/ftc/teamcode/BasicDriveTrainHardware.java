package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    public FieldPosVuf fieldTracker;
    public FieldPosVuf stoneTracker;

    //--------//
    // Wheels //
    //--------//

    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor RearLeft = null;
    public DcMotor RearRight = null;

    //-------------//
    // Other Stuff //
    //-------------//

    public DcMotor ScissorLift = null;
    public Servo GripperLeft;
    public Servo GripperRight;

    // ToDo Set Values
    public static int[] ScissorLiftPositionList = new int[]{0,10,100};
    public static double ScissorLiftPower = 0;
    public static double GripperDefaultPosition = 0;
    public static double GripperClosePosition = 0;
    public static double GripperOpenPosition = 0;

    private int currentPosition = 0;
    private boolean gripperState = false;

    public boolean manualOverride = false;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public BasicDriveTrainHardware(){}

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        //---------//
        // Vuforia //
        //---------//

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
            List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

            // Only stoneTarget will not be included
            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");


            VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
            blueRearBridge.setName("Blue Rear Bridge");
            allTrackables.add(blueRearBridge);

            VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
            redRearBridge.setName("Red Rear Bridge");
            allTrackables.add(redRearBridge);

            VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
            redFrontBridge.setName("Red Front Bridge");
            allTrackables.add(redFrontBridge);

            VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
            blueFrontBridge.setName("Blue Front Bridge");
            allTrackables.add(blueFrontBridge);

            VuforiaTrackable red1 = targetsSkyStone.get(5);
            red1.setName("Red Perimeter 1");
            allTrackables.add(red1);

            VuforiaTrackable red2 = targetsSkyStone.get(6);
            red2.setName("Red Perimeter 2");
            allTrackables.add(red2);

            VuforiaTrackable front1 = targetsSkyStone.get(7);
            front1.setName("Front Perimeter 1");
            allTrackables.add(front1);

            VuforiaTrackable front2 = targetsSkyStone.get(8);
            front2.setName("Front Perimeter 2");
            allTrackables.add(front2);

            VuforiaTrackable blue1 = targetsSkyStone.get(9);
            blue1.setName("Blue Perimeter 1");
            allTrackables.add(blue1);

            VuforiaTrackable blue2 = targetsSkyStone.get(10);
            blue2.setName("Blue Perimeter 2");
            allTrackables.add(blue2);

            VuforiaTrackable rear1 = targetsSkyStone.get(11);
            rear1.setName("Rear Perimeter 1");
            allTrackables.add(rear1);

            VuforiaTrackable rear2 = targetsSkyStone.get(12);
            rear2.setName("Rear Perimeter 2");
            allTrackables.add(rear2);

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
            ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }

            ArrayList<VuforiaTrackable> stone = new ArrayList<VuforiaTrackable>();
            stone.add(stoneTarget);

            targetsSkyStone.activate();
            fieldTracker = new FieldPosVuf(allTrackables, robotFromCamera);
            stoneTracker = new FieldPosVuf(stone, robotFromCamera);


        //--------//
        // Wheels //
        //--------//

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

        //-------------//
        // Other Stuff //
        //-------------//

            ScissorLift = hwMap.get(DcMotor.class, "scissor_lift");
            ScissorLift.setDirection(DcMotor.Direction.FORWARD);
            ScissorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ScissorLift.setPower(ScissorLiftPower);

            GripperLeft = hwMap.get(Servo.class, "gripper_left");
            GripperRight = hwMap.get(Servo.class, "gripper_right");

            ChangeGripperPosition(GripperDefaultPosition);

    }

    public void OpenGripper(){
        gripperState = false;
        ChangeGripperPosition(GripperOpenPosition);
    }

    public void CloseGripper(){
        gripperState = true;
        ChangeGripperPosition(GripperClosePosition);
    }

    private void ChangeGripperPosition(double position){
        // ToDo Make Servo Code
    }

    public void SetScissorLiftPosition(int newPos){
        currentPosition = newPos;
        if(currentPosition > ScissorLiftPositionList.length-1){
            currentPosition = ScissorLiftPositionList.length-1;
        }
        if(currentPosition < 0){
            currentPosition = 0;
        }

        if(!manualOverride){
            ScissorLift.setTargetPosition(ScissorLiftPositionList[currentPosition]);
        }
    }

    public int getScissorLiftPosition(){
        return  currentPosition;
    }

    public void incrementPosition(){
        SetScissorLiftPosition(currentPosition++);
    }

    public void decrementPosition(){
        SetScissorLiftPosition(currentPosition--);
    }

    public void scissorLiftManualOverride(){
        manualOverride = true;
        ScissorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ScissorLift.setPower(0);
    }

    public void revokeScissorLiftManualOverride(){
        manualOverride = false;
        ScissorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ScissorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ScissorLift.setPower(ScissorLiftPower);
        currentPosition = 0;
    }
}
