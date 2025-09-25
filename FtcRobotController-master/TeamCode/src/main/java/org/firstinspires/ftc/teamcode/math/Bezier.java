package org.firstinspires.ftc.teamcode.math;

import java.util.ArrayList;

public class Bezier {

    Vector2D A, B, C, D;
    ArrayList<Vector2D> LUT;
    int accuracy = 50; // Can be modified, the lookup table generated will be the size accuracy+1.

    /**
     * This generates a cubic bezier object using 4 given control points
     * @param A control point 1
     * @param B control point 2
     * @param C control point 3
     * @param D control point 4
     */
    public Bezier(Vector2D A, Vector2D B, Vector2D C, Vector2D D){
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;
        makeLUT();
    }

    /**
     * This function uses the Bernstein polynomials to calculate the point in xy coordinates given parametric input t.
     * @param t This is the parametric input
     * @return The Vector2D output corresponding to a certain point.
     */
    public Vector2D getPoint(double t){
        return (A.times(-1*Math.pow(t,3)+3*Math.pow(t,2)-3*t+1)).plus((B.times(3*Math.pow(t,3)-6*Math.pow(t,2)+3*t)).plus((C.times(-3*Math.pow(t,3)+3*Math.pow(t,2))).plus(D.times(Math.pow(t,3)))));
    }

    /**
     * This generates the lookup table using a for-loop.
     */
    private void makeLUT(){
        for(int i = 0; i<=accuracy; i++){
            LUT.add(getPoint(i/accuracy));
        }
    }

    public Vector2D firstDerivative(double t){
        return (A.times(-3*Math.pow(t,2)+6*t-3)).plus((B.times(9*Math.pow(t,2)-12*t+3)).plus((C.times(-9*Math.pow(t,2)+6*t)).plus(D.times(3*Math.pow(t,2)))));
    }

    public Vector2D secondDerivative(double t){
        return (A.times(-6*t+6)).plus((B.times(18*t-12)).plus((C.times(-18*t+6)).plus(D.times(6*t))));
    }

    public double curvature(double t){
        double numerator = firstDerivative(t).cross(secondDerivative(t));
        double denominator = firstDerivative(t).mag();
        return numerator/Math.pow(denominator,3);
    }

    public double returnClosestLUTPoint(Vector2D getPos){
        double min = LUT.get(0).dist(getPos);
        double t_val = 0;
        for(int i = 1; i<=accuracy; i++){
            if(LUT.get(i).dist(getPos)<min){
                min =  LUT.get(i).dist(getPos);
                t_val = i/accuracy;
            }
        }
        return t_val;
    }

}
