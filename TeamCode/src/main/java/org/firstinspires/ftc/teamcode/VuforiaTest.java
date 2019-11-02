package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name="Test: VufioriaTest", group="Test")
public class VuforiaTest extends LinearOpMode {

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);

        telemetry.addData("hardware", hardware);
        telemetry.addData("fieldTracker",hardware.fieldTracker);
        telemetry.update();

        waitForStart();

        hardware.navTargets.activate();
        while (opModeIsActive()) {
            OpenGLMatrix fieldPos = hardware.fieldTracker.getPosVuf();
            if(fieldPos != null) {
                Orientation rotation = Orientation.getOrientation(fieldPos, EXTRINSIC, XYZ, DEGREES);
                VectorF translation = fieldPos.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0)/25.4,
                              translation.get(1)/25.4,
                              translation.get(2)/25.4);
                telemetry.addData("Rotation (deg)", "{{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                        rotation.firstAngle,
                        rotation.secondAngle,
                        rotation.thirdAngle);
            }else{
                telemetry.addData("There was a problem!","");
            }
            telemetry.update();
            sleep(50);
        }
        hardware.navTargets.deactivate();
    }
}
