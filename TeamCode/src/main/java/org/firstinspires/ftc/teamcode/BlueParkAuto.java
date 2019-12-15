package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="Argus2.0: Park Blue", group="Argus2.0")
public class BlueParkAuto extends LinearOpMode {

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();

    private float[] prevAngle;

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);
        Robot robot = new Robot(hardware);
        telemetry.addData("hardware", hardware);
        telemetry.addData("fieldTracker",hardware.fieldTracker);
        telemetry.update();
        prevAngle = new float[3];

        waitForStart();
        telemetry.addData("Started", 1);
        telemetry.update();

        Position tempTarget = new Position(0, 48, 0, 0);
        double[] tempResult;

        while (opModeIsActive()) {
            OpenGLMatrix fieldPos = hardware.fieldTracker.getPosVuf();

            if (fieldPos != null) {
                Orientation rotation = Orientation.getOrientation(fieldPos, EXTRINSIC, AxesOrder.XYZ, RADIANS );
                VectorF translation = fieldPos.getTranslation();
                Position curPos;
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0)/25.4, translation.get(1)/25.4, translation.get(2)/25.4);

                /* if the Y rotation flips, add half a rotation to the "heading" */
                if (Math.abs(rotation.firstAngle - prevAngle[0]) > 90) {
                    rotation.thirdAngle += Math.PI;
                }

                curPos = new Position(translation.get(0)/25.4, translation.get(1)/25.4,
                        rotation.thirdAngle - (Math.PI/2.0), 100);
                tempResult = Robot.computeAngleAndPower(curPos, tempTarget);
                telemetry.addData("Beta", "%.1f", Math.toDegrees(Math.atan2(tempTarget.Y - curPos.Y, tempTarget.X - curPos.X)));
                telemetry.addData("Angle and power", "%.1f, %.1f", Math.toDegrees(tempResult[0]), tempResult[1]);

                telemetry.addData("Headinng", "%.2f, %.2f, %.2f", Math.toDegrees(rotation.thirdAngle),
                        Math.toDegrees(rotation.firstAngle), Math.toDegrees(rotation.secondAngle));

                if (curPos != tempTarget) {
                    robot.moveAngle(tempResult[0], tempResult[1]);
                    //robot.goToPosition(tempTarget);
                }
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
                prevAngle[0] = rotation.firstAngle;
                prevAngle[1] = rotation.secondAngle;
                prevAngle[2] = rotation.thirdAngle;

            } else {
                robot.turn(0.2);
                telemetry.addData("Turn", 0.2);
                telemetry.addData("No image found", "");
            }


            telemetry.update();
            sleep(500);
        }


/*

    robot.moveAngle(0, 0.3);
    sleep(1000);
    robot.moveAngle(Math.PI/4, 0.3);
    sleep(1000);
    robot.moveAngle(Math.PI/2, 0.3);
    sleep(1000);
    robot.moveAngle(Math.PI, 0.3);
    sleep(1000);
    robot.moveAngle((Math.PI * 3)/2.0, 0.3);
    sleep(1000);
*/
/*
    while(opModeIsActive()) {
        robot.motorOn(0, 0.3);
        sleep(1000);
        robot.motorOn(1, 0.3);
        sleep(1000);
        robot.motorOn(2, 0.3);
        sleep(1000);
        robot.motorOn(3, 0.3);
        sleep(1000);
        robot.motorOn(4, 0.3);
        sleep(1000);

    }

 */
    }
}
