package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="Argus2.0: Park", group="Argus2.0")
public class BlueParkAuto extends LinearOpMode {

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);
        Robot robot = new Robot(hardware);
        telemetry.addData("hardware", hardware);
        telemetry.addData("fieldTracker", hardware.fieldTracker);
        telemetry.update();

        waitForStart();
        telemetry.addData("Started", 1);
        telemetry.update();

        Position tempTarget = new Position(0, 48, 0, 0);
        double[] tempResult;

        while (opModeIsActive()) {
            OpenGLMatrix fieldPos = hardware.fieldTracker.getPosVuf();

            if (fieldPos != null) {
                Orientation rotation = Orientation.getOrientation(fieldPos, EXTRINSIC, AxesOrder.XYZ, RADIANS);
                VectorF translation = fieldPos.getTranslation();
                Position curPos;
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / 25.4, translation.get(1) / 25.4, translation.get(2) / 25.4);
                telemetry.addData("Headinng", "%.2f, %.2f, %.2f", Math.toDegrees(rotation.thirdAngle - (Math.PI / 2.0)),
                        Math.toDegrees(rotation.firstAngle), Math.toDegrees(rotation.secondAngle));

                curPos = new Position(translation.get(0) / 25.4, translation.get(1) / 25.4,
                        rotation.thirdAngle - (Math.PI / 2.0), 100);
                tempResult = Robot.computeAngleAndPower(curPos, tempTarget);
                telemetry.addData("Beta", "%.1f", Math.toDegrees(Math.atan2(tempTarget.Y - curPos.Y, tempTarget.X - curPos.X)));
                telemetry.addData("Angle and power", "%.1f, %.1f", Math.toDegrees(tempResult[0]), tempResult[1]);
                robot.moveAngle(tempResult[0], tempResult[1]);
                // PLEASE REMOVE WHEN DONE
                double sin = Math.sin(tempResult[0]);
                double cos = Math.cos(tempResult[0]);
                double tempPower = tempResult[1];
                if (tempResult[1] < -1.0) {
                    tempPower = -1.0;
                } else if (tempResult[1] > 1.0) {
                    tempPower = 1.0;
                }

                double frontRight = (-((cos - sin) * tempResult[1]));
                double frontLeft = ((cos + sin) * tempResult[1]);
                double rearLeft = ((cos - sin) * tempResult[1]);
                double rearRight = (-((cos + sin) * tempResult[1]));

                telemetry.addData("FR FL RL RR Power", "%.1f, %.1f, %.1f, %.1f", frontRight, frontLeft,
                        rearLeft, rearRight);
                telemetry.addData("power", "%.1f", tempPower);
            } else {
                telemetry.addData("No image found", "");
            }
            telemetry.update();
            sleep(500);
        }
    }
}
