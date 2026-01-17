package org.firstinspires.ftc.teamcode.archive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "drivetrain", group = "lab")
public class drivetrainLab extends OpMode {
    Follower follower;

    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(22,125,Math.toRadians(145)));
        follower.setStartingPose(new Pose(22,125,Math.toRadians(145)));
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop(){
        if(gamepad1.dpad_left){
            follower.setTeleOpDrive(0, -0.5, 0, 0);
        }
        else if(gamepad1.dpad_right){
            if(gamepad1.dpad_left){
                follower.setTeleOpDrive(0, 0.5, 0, 0);
            }
        }
        else follower.setTeleOpDrive(0,0,0,0);
    }
}
