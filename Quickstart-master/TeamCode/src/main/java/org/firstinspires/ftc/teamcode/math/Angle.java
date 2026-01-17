package org.firstinspires.ftc.teamcode.math;

public class Angle {

    public static int FULLROT_DEGREES = 360;
    public static double FULLROT_RADIANS = 2*Math.PI;

    /**
     * returns the signed shortest angle from current to target
     *
     * @param current can be any value
     * @param target  ideally from 0 to a full rotation
     * @param fullRot the measure for a full rotation
     * @return the signed shortest angle from current to target eg: angleWrap(350, 20, 360)=10
     */

    public static double angleWrap(double current, double target, double fullRot) {
        //TEST FUNCTION
        double minimized_current = (current % fullRot + fullRot) % fullRot;
        double minimized_target = (target % fullRot + fullRot) % fullRot;
        double difference = minimized_target - minimized_current;
        if (Math.abs(difference) > fullRot/2) {
            difference -= (Math.signum(difference) * fullRot);
        }
        return difference;
    }

    public static int angleWrap(int current, int target, int fullRot) {
        //TEST FUNCTION
        int minimized_current = (current % fullRot + fullRot) % fullRot;
        int minimized_target = (target % fullRot + fullRot) % fullRot;
        int difference = minimized_target - minimized_current;
        if (Math.abs(difference) > fullRot/2) {
            difference -= (int) (Math.signum(difference) * fullRot);
        }
        return difference;
    }
}