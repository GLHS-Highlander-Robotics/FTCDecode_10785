package org.firstinspires.ftc.teamcode.archive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.storage;

@Autonomous(name = "3ball_auto")
public class auto_3ball extends OpMode {
    Follower follower;
    Timer pathTimer, opmodeTimer;
    DcMotorEx launchMotor;
    storage storage;

    public enum PathState{
        //DRIVE_STARTPOS_ENDPOS or SHOOT_POS
        //DRIVE => Moving, Shooter not activated
        //SHOOT => Shooting, Stationary
        DRIVE_START_SHOOT,
        SHOOT_SHOOT1,
        SHOOT_SHOOT2,
        SHOOT_SHOOT3,
        END
    }

    static PathState pathState;

    static Pose start_pos = new Pose(0,100,Math.toRadians(0));
    static Pose end_pos = new Pose(12,100,Math.toRadians(0));

    PathChain drive_start_shoot, shoot_shoot;

    public void buildPaths(){
        drive_start_shoot = follower.pathBuilder()
                .addPath(new BezierLine(start_pos, end_pos))
                .setLinearHeadingInterpolation(start_pos.getHeading(), end_pos.getHeading())
                .build()
        ;
    }

    public void updatePathState(){
        switch (pathState){
            case DRIVE_START_SHOOT:
                follower.followPath(drive_start_shoot, true);
                setPathState(PathState.SHOOT_SHOOT1);
                break;
            case SHOOT_SHOOT1:
                if(pathTimer.getElapsedTimeSeconds()>=4){
                    follower.holdPoint(end_pos);
                    setPathState(PathState.END);
                }
            case SHOOT_SHOOT2:
                if(pathTimer.getElapsedTimeSeconds()>=4){
                    follower.holdPoint(end_pos);
                    setPathState(PathState.END);
                }
            case SHOOT_SHOOT3:
                if(pathTimer.getElapsedTimeSeconds()>=4){
                    follower.holdPoint(end_pos);
                    setPathState(PathState.END);
                }
                break;
            case END:
                if(pathTimer.getElapsedTimeSeconds()>=4){
                    telemetry.addLine("END!!!!");
                }
            default:
                telemetry.addLine("NO STATE ACTIVATED");
                break;
        }
    }

    public void setPathState(PathState inputState){
        pathState = inputState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_START_SHOOT;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(start_pos);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        updatePathState();
    }
}
