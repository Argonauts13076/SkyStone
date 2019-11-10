package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public double posErr = 1.0;
    public BasicDriveTrainHardware hardware;

    public Robot(HardwareMap ahwMap) {
        hardware = new BasicDriveTrainHardware();
        hardware.init(ahwMap);
    }

    public void moveAngle (double theta, double power) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        if (power < -1.0) {
            power = -1.0;
        } else if (power > 1.0) {
            power = 1.0;
        }

        hardware.FrontRight.setPower((sin - cos) * power);
        hardware.FrontLeft.setPower((sin + cos) * power);
        hardware.RearLeft.setPower((sin - cos) * power);
        hardware.RearRight.setPower((sin + cos) * power);
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


    public void goToPosition (Position target) {
        Position curr = getPosition();
        while (Math.abs(curr.X - target.X) > posErr || Math.abs(curr.Y - target.Y) > posErr) {
            while (!vuforiaLockOn()) {
                turn(0.25);
            }
            curr = getPosition();

            double beta = Math.atan2(target.Y - curr.Y, target.X - curr.X);
            double targetAngle = (Math.PI / 2) - curr.rotation + beta;
            double distance =
                Math.sqrt(Math.pow(curr.X - target.X, 2) + Math.pow(curr.Y - target.Y, 2));
            double power;

            if (distance > 24.0) {
                power = 1.0;
            } else {
                power = 0.1 + (distance / 24.0 - 0.1);
            }

            moveAngle(targetAngle, power);
        }
    }

    public Position getPosition() {
        return new Position(0,0,0,0);
    }

    public boolean vuforiaLockOn() {
        return true;
    }
}
