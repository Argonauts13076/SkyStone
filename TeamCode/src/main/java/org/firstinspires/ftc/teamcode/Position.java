package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Position {
    public double X;
    public double Y;

    public double rotation;

    public double confidence;

    public Position(double X, double Y, double rotation, double confidence){
        this.X = X;
        this.Y = Y;
        this.rotation = AngleUnit.normalizeRadians(rotation);
        this.confidence = confidence;
    }
}
