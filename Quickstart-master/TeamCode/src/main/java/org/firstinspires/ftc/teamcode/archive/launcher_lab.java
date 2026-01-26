package org.firstinspires.ftc.teamcode.archive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "launcher lab", group = "lab")
public class launcher_lab extends OpMode {

    DcMotorEx turret_motor;
    static double Kf = 0.0034, Kp = 0.12;
    VoltageSensor voltageSensor;
    Follower follower;

    @Override
    public void init(){
        turret_motor = hardwareMap.get(DcMotorEx.class, "turret motor");
        turret_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(22,125,Math.toRadians(145)));
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop(){
        follower.update();
        if(gamepad1.dpad_up){
            follower.setTeleOpDrive(0.2, 0,0,true);
        }
        else if(gamepad1.dpad_down){
            follower.setTeleOpDrive(-0.2,0,0,true);
        }
        else if(gamepad1.dpad_left){
            follower.setTeleOpDrive(0,0,-0.2,true);
        }
        else if(gamepad1.dpad_right){
            follower.setTeleOpDrive(0,0,0.2,true);
        }
        else follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,false);
        double distance = Math.sqrt((follower.getPose().getX()*follower.getPose().getX())+((144-follower.getPose().getY())*(144-follower.getPose().getY())));
        double target_velocity = 0.8689*distance+130;
        if(gamepad1.x){
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }
        if(gamepad1.b){
            turret_motor.setPower(Kf*voltageSensor.getVoltage()+Kp*(target_velocity-turret_motor.getVelocity(AngleUnit.DEGREES)));
        }
        else turret_motor.setPower(0);
        telemetry.addData("Velocity", turret_motor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Dist", distance);
        telemetry.update();
    }
}
