package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.launcher;

@Configurable
@Autonomous(name = "auto blue")
public class auto_blue extends OpMode {

    drivetrain drivetrain;
    launcher launcher;
    DcMotorEx intake, transfer;
    static double tempTargetSpeed = 1500;
    static Pose2D blue_goal = new Pose2D(DistanceUnit.INCH, 144, 0, AngleUnit.RADIANS, 0);
    static Pose2D blue_far_side_start = new Pose2D(DistanceUnit.INCH, 11, 49, AngleUnit.RADIANS, Math.toRadians(20));

    boolean b = false;
    boolean shoot = false;
    double zeroHeading = 0;
    ElapsedTime timer;

    @Override
    public void init(){
        drivetrain = new drivetrain(hardwareMap, telemetry, true);
        launcher = new launcher(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        timer = new ElapsedTime();
        drivetrain.pinpoint.setPosition(blue_far_side_start);
    }

    @Override
    public void start(){
        drivetrain.pinpoint.setPosition(blue_far_side_start);
        timer.reset();
    }

    public void loop(){
        drivetrain.pinpoint.update();
        if(0<timer.milliseconds()&&timer.milliseconds()<5000){
            launcher.powerFlywheel(1600);
        }
        else if(5000<timer.milliseconds()&&timer.milliseconds()<12000){
            launcher.powerFlywheel(1600);
            transfer.setPower(0.6);
            intake.setPower(0.5);
        }
        else if(12000<timer.milliseconds()&&timer.milliseconds()<15000){
            launcher.powerFlywheel(0);
            transfer.setPower(0);
            intake.setPower(0);
            drivetrain.RCDrive(0.3, 0,0);
        }
        else{
            launcher.powerFlywheel(0);
            drivetrain.RCDrive(0, 0,0);
        }
    }
}
