package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.*;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class FieldPosVuf {

    List<VuforiaTrackable> trackables;
    OpenGLMatrix lastLocation = null;

    private final OpenGLMatrix cameraRelativeToRobot;

    private static final float mmPerInch = 25.4f;

    boolean targetVisible = false;

    public FieldPosVuf(List<VuforiaTrackable> trackables, OpenGLMatrix cameraRelativeToRobot){
        this.trackables = trackables;
        this.cameraRelativeToRobot = cameraRelativeToRobot;
    }

    public OpenGLMatrix getPosVuf(){


        targetVisible = false;
        for (VuforiaTrackable trackable : trackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
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

        if (targetVisible) {
            return lastLocation;
        } else {
            return null;
        }
    }

}

