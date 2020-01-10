package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="AutoskystoneRed", group="Argus2.0")
public class AutoSkystoneRed extends LinearOpMode {
    private static final float mmPerInch        = 25.4f;

    // Change this for blue (-0.6f) or red (0.6f) autonomous
    private float turnPower = 0.6f;
    // Change this for blue (-0.2f) or red (0.2f) autonomous
    private float strafePower = 0.2f;
    // Change this for blue (-0.03f) or red (0.03f) autonomous
    private float strafeFudge = 0.03f;

    enum AutoState {
        FWD,
        STRAFE,
        ALIGN_ROTATATION,
        FWD_TO_BLOCK,
        GRAB,
        TURN,
        FWD_DROP_BACK,
        FINISH,
    }

    @Override
    public void runOpMode() {
        BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();
        hardware.init(hardwareMap);

        waitForStart();

        hardware.OpenGripper();
        AutoState state = AutoState.FWD;
        int repeated_state = 0;
        int strafeCounter = 0;
        final float angleThresh = 2.0f;
        OpenGLMatrix robotLocationTransform;

        float x, y, xvec = 0.0f, yvec = 0.0f;
        while (opModeIsActive()) {
            telemetry.addData("state", state);
            switch (state) {
                case FWD:
                    hardware.FrontLeft.setPower(0.25f);
                    hardware.FrontRight.setPower(-0.25f);
                    hardware.RearLeft.setPower(0.25f);
                    hardware.RearRight.setPower(-0.25f);
                    sleep(600);
                    state = AutoState.STRAFE;
                    break;
                case STRAFE:
                    robotLocationTransform = hardware.getStoneTransform();
                    if (robotLocationTransform != null) {
                        VectorF translation = robotLocationTransform.getTranslation();
                        x = (translation.get(0) / mmPerInch) + 1.25f;
                        y = (translation.get(1) / mmPerInch) - 3.5f;
                        telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f", x, y);


                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(robotLocationTransform, EXTRINSIC, XYZ, DEGREES);

                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        if (y > 3.0f) {
                            yvec = -0.2f;
                        } else if (y > 1.0f) {
                            yvec = -0.15f;
                        } else if (y < -3.0f) {
                            yvec = 0.2f;
                        } else if (y < -1.0f) {
                            yvec = 0.15f;
                        } else {
                            yvec = 0.0f;
                            repeated_state = 0;
                            state = AutoState.ALIGN_ROTATATION;
                        }

                        telemetry.addData("Vectors", "x = %.2f,y = %.2f", xvec, yvec);
                    } else {
                        yvec = strafePower;
                    }

                    if (++strafeCounter == 26) {
                        strafePower = -strafePower;
                        strafeFudge = -strafeFudge;
                    }

                    hardware.FrontRight.setPower(yvec - strafeFudge);
                    hardware.FrontLeft.setPower(yvec - strafeFudge);
                    hardware.RearLeft.setPower(-yvec);
                    hardware.RearRight.setPower(-yvec);

                    break;
                case ALIGN_ROTATATION:
                    float rotation_power = 0.0f;
                    robotLocationTransform = hardware.getStoneTransform();
                    if (robotLocationTransform != null) {
                        Orientation rotation = Orientation.getOrientation(robotLocationTransform, EXTRINSIC, XYZ, DEGREES);
                        VectorF translation = robotLocationTransform.getTranslation();
                        x = (translation.get(0) / mmPerInch) + 1.25f;
                        y = (translation.get(1) / mmPerInch) - 3.5f;

                        telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f", x, y);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                        if (rotation.thirdAngle < (90.0f - angleThresh)) {
                            rotation_power = -0.25f;
                            repeated_state = 1;
                        } else if (rotation.thirdAngle > (90.0f + angleThresh)) {
                            rotation_power = 0.25f;
                            repeated_state = 1;

                        } else if (repeated_state == 0) {
                            state = AutoState.FWD_TO_BLOCK;
                        } else {
                            state = AutoState.STRAFE;
                            repeated_state = 0;
                        }
                    }

                    hardware.FrontLeft.setPower(rotation_power);
                    hardware.FrontRight.setPower(rotation_power);
                    hardware.RearLeft.setPower(rotation_power);
                    hardware.RearRight.setPower(rotation_power);

                    break;
                case FWD_TO_BLOCK:
                    hardware.FrontLeft.setPower(0.28f);
                    hardware.FrontRight.setPower(-0.28f);
                    hardware.RearLeft.setPower(0.28f);
                    hardware.RearRight.setPower(-0.28f);
                    sleep(800);
                    hardware.FrontLeft.setPower(0.2f);
                    hardware.FrontRight.setPower(-0.2f);
                    hardware.RearLeft.setPower(0.2f);
                    hardware.RearRight.setPower(-0.2f);
                    sleep(400);

                    hardware.FrontLeft.setPower(0);
                    hardware.FrontRight.setPower(0);
                    hardware.RearLeft.setPower(0);
                    hardware.RearRight.setPower(0);

                    state = AutoState.GRAB;
                    break;
                case GRAB:
                    hardware.CloseGripper();
                    state = AutoState.TURN;
                    break;
                case TURN:
                    hardware.FrontLeft.setPower(turnPower);
                    hardware.FrontRight.setPower(turnPower);
                    hardware.RearLeft.setPower(turnPower);
                    hardware.RearRight.setPower(turnPower);
                    sleep(1200);
                    state = AutoState.FWD_DROP_BACK;
                    break;
                case FWD_DROP_BACK:
                    hardware.FrontLeft.setPower(0.6f);
                    hardware.FrontRight.setPower(-0.6f);
                    hardware.RearLeft.setPower(0.6f);
                    hardware.RearRight.setPower(-0.6f);
                    sleep(1000);
                    hardware.OpenGripper();
                    sleep(200);

                    hardware.FrontLeft.setPower(-0.6f);
                    hardware.FrontRight.setPower(0.6f);
                    hardware.RearLeft.setPower(-0.6f);
                    hardware.RearRight.setPower(0.6f);
                    sleep(1);

                    state = AutoState.FINISH;
                    break;
                case FINISH:
                    hardware.FrontLeft.setPower(0);
                    hardware.FrontRight.setPower(0);
                    hardware.RearLeft.setPower(0);
                    hardware.RearRight.setPower(0);
                    break;
            }
            telemetry.update();
            sleep(250);
        }
    }
}

