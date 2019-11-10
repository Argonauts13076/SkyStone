package org.firstinspires.ftc.teamcode;



public class FieldObstacle {
    public float x, y;


    // width is the X dimension, length is the Y dimension
    public float width, length;
    public float theta;

    public FieldObstacle(float x, float y, float width, float length, float theta) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.length = length;
        this.theta = theta;
    }


    public FieldObstacle(float x, float y, float width, float length) {
        this(x, y, width, length, 0);
    }

}
