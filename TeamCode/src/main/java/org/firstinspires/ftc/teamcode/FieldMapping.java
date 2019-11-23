package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;



public class FieldMapping {
    public float length;
    public float width;

    public ArrayList<FieldObstacle> obstacles = new ArrayList<>();

    public FieldMapping(float length, float width) {
        this.length = length;
        this.width = width;

    }

    // width is the X dimension, length is the Y dimension
    public void addObstacle(float x, float y, float width, float length) {
        obstacles.add(new FieldObstacle(x,y,length,width,0));
    }

    public void addObstacle(FieldObstacle obstacle){
        obstacles.add(obstacle);
    }


}





