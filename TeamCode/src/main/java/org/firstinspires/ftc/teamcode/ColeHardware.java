package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColeHardware {

    public Servo testServo = null;
    public DigitalChannel digitalTouch = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public static final double  SERVO_INIT_POSITION = 0;
    public static final double  SERVO_FINAL_POSITION = 1;

    public ColeHardware(){

    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        testServo = hwMap.get(Servo.class, "testServo");
        testServo.setPosition(SERVO_INIT_POSITION);

        digitalTouch = hwMap.get(DigitalChannel.class, "testTouch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    }

}
