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

    ArrayList<VuforiaTrackable> trackables;
    ArrayList<Position> relativePositions;
    OpenGLMatrix lastLocation = null;
    int lastImageId = -1;

    private static final float mmPerInch = 25.4f;

    boolean targetVisible = false;

    public FieldPosVuf(ArrayList<VuforiaTrackable> trackables, ArrayList<Position> relativePositions){
        this.trackables = trackables;
        this.relativePositions = relativePositions;
    }

    public Position getPosAbs(){
        PositionAndInt data = getPosVuf();
        Position imageFieldPosition = relativePositions.get(data.integer);
        Position finalPosition = new Position(imageFieldPosition.X - data.position.X, imageFieldPosition.Y - data.position.Y, (imageFieldPosition.rotation - data.position.rotation)%360, data.position.confidence);
        return finalPosition;
    }

    private PositionAndInt getPosVuf(){


        targetVisible = false;
        for (VuforiaTrackable trackable : trackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    lastImageId = trackables.indexOf(trackable);
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return new PositionAndInt(new Position(translation.get(0)/mmPerInch,translation.get(1)/mmPerInch, rotation.thirdAngle, 1), lastImageId);
        }else{
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return new PositionAndInt(new Position(translation.get(0)/mmPerInch,translation.get(1)/mmPerInch,rotation.thirdAngle,0), lastImageId);
        }
    }

}

