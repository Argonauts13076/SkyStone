package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.*;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Robot {
    public static float maxDist = 48;
    public static float minPower = 0.1f;

    public double posErr = 1.0;
    public BasicDriveTrainHardware hardware;

    public Robot(BasicDriveTrainHardware hw) {
        this.hardware = hw;
    }

    public void moveAngle(double theta, double power) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        if (power < -1.0) {
            power = -1.0;
        } else if (power > 1.0) {
            power = 1.0;
        }


        hardware.FrontRight.setPower(-((cos - sin) * power));
        hardware.RearRight.setPower(-((cos + sin) * power));
        hardware.FrontLeft.setPower((cos + sin) * power);
        hardware.RearLeft.setPower((cos - sin) * power);
    }

    public void turn(double power) {
        if (power < -1.0) {
            power = -1.0;
        } else if (power > 1.0) {
            power = 1.0;
        }
        hardware.FrontLeft.setPower(power);
        hardware.RearLeft.setPower(power);
        hardware.FrontRight.setPower(-power);
        hardware.RearRight.setPower(-power);
    }

    public void motorOn(int motorNumber, double power) {
        switch (motorNumber) {
            case 0:
                hardware.FrontLeft.setPower(power);
                hardware.RearLeft.setPower(0);
                hardware.FrontRight.setPower(0);
                hardware.RearRight.setPower(0);
                break;
            case 1:
                hardware.FrontLeft.setPower(0);
                hardware.RearLeft.setPower(power);
                hardware.FrontRight.setPower(0);
                hardware.RearRight.setPower(0);
                break;
            case 2:
                hardware.FrontLeft.setPower(0);
                hardware.RearLeft.setPower(0);
                hardware.FrontRight.setPower(power);
                hardware.RearRight.setPower(0);
                break;
            case 3:
                hardware.FrontLeft.setPower(0);
                hardware.RearLeft.setPower(0);
                hardware.FrontRight.setPower(0);
                hardware.RearRight.setPower(power);
                break;
            default:
                hardware.FrontLeft.setPower(0);
                hardware.RearLeft.setPower(0);
                hardware.FrontRight.setPower(0);
                hardware.RearRight.setPower(0);



        }
    }

    public void goToPosition(Position target) {
        Position curr = getPosition();
        while (Math.abs(curr.X - target.X) > posErr || Math.abs(curr.Y - target.Y) > posErr) {
/*            while (!vuforiaLockOn()) {
                turn(0.25);
            } */
            curr = getPosition();

            double[] angleAndPower = computeAngleAndPower(curr, target);
            moveAngle(angleAndPower[0], angleAndPower[1]);
        }
    }


    public static double[] computeAngleAndPower(Position curr, Position target) {
        double[] ret = new double[2];

        double beta = Math.atan2(target.Y - curr.Y, target.X - curr.X);
        double targetAngle = -curr.rotation + beta;
        double distance =
                Math.sqrt(Math.pow(curr.X - target.X, 2) + Math.pow(curr.Y - target.Y, 2));
        double power;

        if (distance > maxDist) {
            power = 1.0;
        } else {
            power = minPower + (distance / maxDist) * (1 - minPower);
        }

        ret[0] = targetAngle;
        ret[1] = power;

        return ret;

    }

    public Position getPosition() {
        OpenGLMatrix fieldPos = hardware.fieldTracker.getPosVuf();
        Orientation rotation = Orientation.getOrientation(fieldPos, EXTRINSIC, XYZ, DEGREES);
        VectorF translation = fieldPos.getTranslation();

        return new Position(translation.get(0)/25.4, translation.get(1)/25.4,
                rotation.thirdAngle, 1);
    }

    private boolean vuforiaLockOn() {
        OpenGLMatrix fieldPos = hardware.fieldTracker.getPosVuf();
        return fieldPos != null;
    }

}
