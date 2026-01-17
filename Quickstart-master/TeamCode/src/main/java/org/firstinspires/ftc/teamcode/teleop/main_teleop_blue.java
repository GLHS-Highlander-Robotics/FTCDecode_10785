package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.launcher;

@Configurable
@TeleOp(name = "main teleop blue")
public class main_teleop_blue extends OpMode {

    drivetrain drivetrain;
    launcher launcher;
    DcMotorEx intake, transfer;
    static double tempTargetSpeed = 1600;
    static double intake_speed = 0.7, transfer_speed = 0.5;
    static Pose2D blue_goal = new Pose2D(DistanceUnit.INCH, 144, 0, AngleUnit.RADIANS, 0);
    static Pose2D blue_far_side_start = new Pose2D(DistanceUnit.INCH, 9, 64, AngleUnit.RADIANS, 0);

    boolean b = false;
    boolean shoot = false;
    double zeroHeading = 0;

    @Override
    public void init(){
        drivetrain = new drivetrain(hardwareMap, telemetry, false);
        launcher = new launcher(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    @Override
    public void loop(){
        drivetrain.pinpoint.update();
        double distance = Math.hypot((drivetrain.pinpoint.getPosX(DistanceUnit.INCH)-blue_goal.getX(DistanceUnit.INCH)), (drivetrain.pinpoint.getPosY(DistanceUnit.INCH)-blue_goal.getY(DistanceUnit.INCH)));
        double angle = -1*Math.atan2((drivetrain.pinpoint.getPosY(DistanceUnit.INCH)-blue_goal.getY(DistanceUnit.INCH)), (drivetrain.pinpoint.getPosX(DistanceUnit.INCH)-blue_goal.getX(DistanceUnit.INCH)));
        updateDrivetrain(angle);
        updateIntake();
        updateShooter(distance);
        if(gamepad1.b && !b){
            shoot = !shoot;
        }
        b = gamepad1.b;

        telemetry.addData("xDist", (drivetrain.pinpoint.getPosX(DistanceUnit.INCH)-blue_goal.getX(DistanceUnit.INCH)));
        telemetry.addData("yDist", (drivetrain.pinpoint.getPosY(DistanceUnit.INCH)-blue_goal.getY(DistanceUnit.INCH)));
        telemetry.addData("distance", distance);
        telemetry.addData("angle", angle);
        telemetry.update();
    }

    public void updateDrivetrain(double angle){

        if(gamepad1.dpad_up){
            drivetrain.RCDrive(0.3, 0,0);
        }
        else if(gamepad1.dpad_down){
            drivetrain.RCDrive(-0.3,0,0);
        }
        else if(gamepad1.dpad_left){
            drivetrain.RCDrive(0,0,0.3);
        }
        else if(gamepad1.dpad_right){
            drivetrain.RCDrive(0,0,-0.3);
        }
        else{
            if(gamepad1.y){
                drivetrain.targetAngle(-gamepad1.left_stick_y, gamepad1.left_stick_x,angle);
            }
            else {
                drivetrain.FC_Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x,drivetrain.pinpoint.getHeading(AngleUnit.RADIANS)-zeroHeading);
            }
        }
        if(gamepad1.x){
            zeroHeading = drivetrain.pinpoint.getHeading(AngleUnit.RADIANS);
        }
    }

    public void updateIntake(){
        if(gamepad1.a||gamepad1.left_trigger >= 0.3){
            intake.setPower(intake_speed);
        }
        else intake.setPower(0);
        if(gamepad1.right_trigger >= 0.3||gamepad1.left_trigger >= 0.3){
            transfer.setPower(transfer_speed);
        }
        else transfer.setPower(0);
    }
    public void updateShooter(double distance){
        if(shoot){
            launcher.powerFlywheel(tempTargetSpeed);
        }
        else launcher.powerFlywheel(0);
    }
}
