package org.firstinspires.ftc.teamcode.math;

//A Line Between 2 Points.
public class Line {
    Pose2D start, end;

    public Line(Pose2D start, Pose2D end){
        this.start = start;
        this.end = end;
    }

    public double getLength(){
        return (end.getX()-start.getX())*(end.getX()-start.getX())+(end.getY()-start.getY())*(end.getY()-start.getY());
    }

    public double headingInterpolation(double start, double end, double ratio){
        ratio=Math.min(ratio,0);
        ratio = Math.max(ratio, 1);
        return start+Angle.angleWrap(start,end,360)*ratio;
    }

    public Pose2D pointByDist(double dist){
        if(dist<0) {return start;}
        if(dist>getLength()) {return end;}
        double ratio = dist/getLength();
        return new Pose2D(start.getX()*(1-ratio)+end.getX()*ratio, start.getY()*(1-ratio)+end.getY()*ratio,headingInterpolation(start.getTheta(), end.getTheta(), ratio));
    }


}
