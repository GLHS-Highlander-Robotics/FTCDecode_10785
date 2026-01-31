package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.launcher;

@Configurable
@Autonomous(name = "AUTO CLOSE BLUE 12")
public class close_blue_12 extends OpMode{

    private TelemetryManager panelsTelemetry;
    Limelight3A limelight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    org.firstinspires.ftc.teamcode.subsystems.drivetrain drivetrain;
    org.firstinspires.ftc.teamcode.subsystems.launcher launcher;
    DcMotorEx intake, transfer;
    static double farSpeed = 1540;
    static double closeSpeed = 1360;
    static double intake_speed = 0.55, transfer_speed = 0.5;
    static Pose2D blue_goal = new Pose2D(DistanceUnit.INCH, 144, 0, AngleUnit.RADIANS, 0);
    static double initial_spinup_time_seconds = 2.5, stopping_wait_time_seconds = 0.1, shooting_time_seconds = 2.4;
    boolean b = false;
    boolean shoot = false;
    double zeroHeading = 0;
    static double max_ball_pickup_power = 0.3;

    static int flywheel_speed = 1280;
    static double intake_power_regular = 0.55, intake_power_shoot = 0.55, intake_power_transfer = 0.3, transfer_power_regular = 0.2, transfer_power_shoot = 0.9;

    //Poses [ADD CONTROL POINTS!!!!!]
    static Pose start = new Pose(22, 125, Math.toRadians(142));
    static Pose shoot_pos = new Pose(55, 83, Math.toRadians(135));
    static Pose ballrow1_begin = new Pose(42, 83, Math.toRadians(180));
    static Pose ballrow1_end = new Pose(21, 83, Math.toRadians(180));
    static Pose ballrow2_begin = new Pose(42, 59, Math.toRadians(180));
    static Pose ballrow2_end = new Pose(21, 59, Math.toRadians(180));
    static Pose ballrow3_begin = new Pose(42, 36, Math.toRadians(180));
    static Pose ballrow3_end = new Pose(21, 36, Math.toRadians(180));
    static Pose park = new Pose(42, 71, Math.toRadians(180));

    private enum robotState{
        INERT,
        SPINUP,
        INTAKE,
        INTAKE_SPINUP,
        TRANSFER,
        TRANSFER_SPINUP,
        SHOOT
    }

    private Path start_to_shoot, shoot_to_row1, row1_path, row1_to_shoot, shoot_to_row2, row2_path, row2_to_shoot, shoot_to_row3, row3_path, row3_to_shoot;
    private PathChain row1, row2, row3, park_path;

    robotState state = robotState.INERT;

    public void buildPaths(){
        start_to_shoot = new Path(new BezierLine(start, shoot_pos));
        start_to_shoot.setLinearHeadingInterpolation(start.getHeading(),shoot_pos.getHeading());
        shoot_to_row1 = new Path(new BezierLine(shoot_pos, ballrow1_begin));
        shoot_to_row1.setLinearHeadingInterpolation(shoot_pos.getHeading(), ballrow1_begin.getHeading());
        row1_path = new Path(new BezierLine(ballrow1_begin, ballrow1_end));
        row1_path.setLinearHeadingInterpolation(ballrow1_begin.getHeading(), ballrow1_end.getHeading());
        row1_to_shoot = new Path(new BezierLine(ballrow1_end, shoot_pos));
        row1_to_shoot.setLinearHeadingInterpolation(ballrow1_end.getHeading(),shoot_pos.getHeading());
        shoot_to_row2 = new Path(new BezierLine(shoot_pos, ballrow2_begin));
        shoot_to_row2.setLinearHeadingInterpolation(shoot_pos.getHeading(), ballrow2_begin.getHeading());
        row2_path = new Path(new BezierLine(ballrow2_begin, ballrow2_end));
        row2_path.setLinearHeadingInterpolation(ballrow2_begin.getHeading(), ballrow2_end.getHeading());
        row2_to_shoot = new Path(new BezierLine(ballrow2_end, shoot_pos));
        row2_to_shoot.setLinearHeadingInterpolation(ballrow2_end.getHeading(),shoot_pos.getHeading());
        shoot_to_row3 = new Path(new BezierLine(shoot_pos, ballrow3_begin));
        shoot_to_row3.setLinearHeadingInterpolation(shoot_pos.getHeading(), ballrow3_begin.getHeading());
        row3_path = new Path(new BezierLine(ballrow3_begin, ballrow3_end));
        row3_path.setLinearHeadingInterpolation(ballrow3_begin.getHeading(), ballrow3_end.getHeading());
        row3_to_shoot = new Path(new BezierLine(ballrow3_end, shoot_pos));
        row3_to_shoot.setLinearHeadingInterpolation(ballrow3_end.getHeading(),shoot_pos.getHeading());
        row1 = follower.pathBuilder()
                .addPath(row1_path)
                .build();
        row2 = follower.pathBuilder()
                .addPath(row2_path)
                .build();
        row3 = follower.pathBuilder()
                .addPath(row3_path)
                .build();
        park_path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(shoot_pos, park)))
                .setLinearHeadingInterpolation(shoot_pos.getHeading(), park.getHeading())
                .build();
    }

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        drivetrain = new drivetrain(hardwareMap, telemetry, false);
        launcher = new launcher(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData("Limelight", "Initialized");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(start);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        updateRobot(state);
    }

    void updateRobot(robotState state) {
        switch (state){
            case INERT:
                intake.setPower(0);
                transfer.setPower(0);
                launcher.powerFlywheel(0);
                break;
            case SPINUP:
                intake.setPower(0);
                transfer.setPower(0);
                launcher.powerFlywheel(flywheel_speed);
                break;
            case INTAKE:
                intake.setPower(intake_power_regular);
                transfer.setPower(transfer_power_regular);
                launcher.powerFlywheel(0);
                break;
            case INTAKE_SPINUP:
                intake.setPower(intake_power_regular);
                transfer.setPower(transfer_power_regular);
                launcher.powerFlywheel(flywheel_speed);
                break;
            case SHOOT:
                intake.setPower(intake_power_shoot);
                transfer.setPower(transfer_power_shoot);
                launcher.powerFlywheel(flywheel_speed);
                break;
            case TRANSFER:
                intake.setPower(intake_power_transfer);
                transfer.setPower(transfer_power_regular);
                launcher.powerFlywheel(0);
                break;
            case TRANSFER_SPINUP:
                intake.setPower(intake_power_transfer);
                transfer.setPower(transfer_power_regular);
                launcher.powerFlywheel(flywheel_speed);
                break;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(start_to_shoot);
                setPathState(1);
                state = robotState.SPINUP;
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    if(pathTimer.getElapsedTimeSeconds() > initial_spinup_time_seconds){
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        state = robotState.SHOOT;
                    }
                    if(pathTimer.getElapsedTimeSeconds() > initial_spinup_time_seconds+shooting_time_seconds){
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(shoot_to_row1,true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    state = robotState.INTAKE_SPINUP;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(row1, max_ball_pickup_power,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    state = robotState.TRANSFER_SPINUP;
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(row1_to_shoot,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > stopping_wait_time_seconds){
                    state = robotState.SHOOT;
                }
                if(pathTimer.getElapsedTimeSeconds()>stopping_wait_time_seconds+shooting_time_seconds){
                    follower.followPath(shoot_to_row2);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    state = robotState.INTAKE_SPINUP;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(row2, max_ball_pickup_power,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    state = robotState.TRANSFER_SPINUP;
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(row2_to_shoot,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > stopping_wait_time_seconds){
                    state = robotState.SHOOT;
                }
                if(pathTimer.getElapsedTimeSeconds()>stopping_wait_time_seconds+shooting_time_seconds){
                    follower.followPath(shoot_to_row3);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    state = robotState.INTAKE_SPINUP;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(row3, max_ball_pickup_power,true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    state = robotState.TRANSFER_SPINUP;
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(row3_to_shoot,true);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds() > stopping_wait_time_seconds){
                    state = robotState.SHOOT;
                }
                if(pathTimer.getElapsedTimeSeconds()>stopping_wait_time_seconds+shooting_time_seconds){
                    state = robotState.INERT;
                    follower.followPath(park_path, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void updateIntake(){
        if(gamepad1.left_trigger >= 0.3){
            intake.setPower(0.9);
            transfer.setPower(0.9);
        }

        else if(gamepad1.right_trigger >= 0.3){
            transfer.setPower(0.25);
            intake.setPower(0.7);
        }
        else if(gamepad1.right_bumper){
            intake.setPower(0.6);
        }
        else{
            transfer.setPower(0);
            intake.setPower(0);
        }
    }
    public void updateShooter(double distance){
        if(shoot){
            if(distance > 115){
                launcher.powerFlywheel(farSpeed);
            }
            else launcher.powerFlywheel(closeSpeed);

        }
        else launcher.powerFlywheel(0);
    }
}

