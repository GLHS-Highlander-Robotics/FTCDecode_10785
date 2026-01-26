package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    double kP;
    double kI;
    double kD;
    double kF = 0;



    double lastTime = 0;
    double lastInput = 0;
    public double integral = 0;
    double integralCap = 30;
    public boolean isBusy = false;

    public PID(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }

    public PID(double p, double i, double d, double f){
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }


    public double output(ElapsedTime elapsedTime, double input, double reference){
        double error = input-reference;
        double lastError= lastInput-reference;
        double derivative = (error-lastError)/(elapsedTime.seconds()-lastTime);
        integral += (error)*(elapsedTime.seconds()-lastTime);
        lastTime = elapsedTime.seconds();
        lastInput = input;
        if(Math.abs(error) >= 1) {isBusy = true;}
        else isBusy = false;
        return kF*Math.signum(error)+(error)*kP+integral*kI+derivative*kD;

    }
    public void getData(Telemetry telemetry){
        telemetry.addData("integral", integral);
    }
}
