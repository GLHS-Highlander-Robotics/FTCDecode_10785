package org.firstinspires.ftc.teamcode.math;

public class Pose2D {
    double x, y, theta;

    public Pose2D(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}
