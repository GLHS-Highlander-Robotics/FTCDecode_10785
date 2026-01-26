package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.launcher;

@Configurable
@TeleOp(name = "main teleop red")
public class main_teleop_red extends OpMode {

    private TelemetryManager panelsTelemetry;
    drivetrain drivetrain;
    launcher launcher;
    DcMotorEx intake, transfer;
    static double farSpeed = 1540;
    static double closeSpeed = 1360;
    static Pose2D red_goal = new Pose2D(DistanceUnit.INCH, 144, 144, AngleUnit.RADIANS, 0);
    static Pose2D red_far_side_start = new Pose2D(DistanceUnit.INCH, 9, 80, AngleUnit.RADIANS, 0);
    Limelight3A limelight;
    boolean b = false;
    boolean shoot = false;
    double zeroHeading = 0;


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
        limelight.pipelineSwitch(1);
        limelight.start();
        telemetry.addData("Limelight", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){
        drivetrain.pinpoint.update();
        double distance = Math.hypot((drivetrain.pinpoint.getPosX(DistanceUnit.INCH)-red_goal.getX(DistanceUnit.INCH)), (drivetrain.pinpoint.getPosY(DistanceUnit.INCH)-red_goal.getY(DistanceUnit.INCH)));
        double angle = -1*Math.atan2((drivetrain.pinpoint.getPosY(DistanceUnit.INCH)-red_goal.getY(DistanceUnit.INCH)), (drivetrain.pinpoint.getPosX(DistanceUnit.INCH)-red_goal.getX(DistanceUnit.INCH)));
        updateDrivetrain(angle);
        updateIntake();
        updateShooter(distance);
        if(gamepad1.b && !b){
            shoot = !shoot;
        }
        b = gamepad1.b;

        telemetry.addData("xDist", (drivetrain.pinpoint.getPosX(DistanceUnit.INCH)-red_goal.getX(DistanceUnit.INCH)));
        telemetry.addData("yDist", (drivetrain.pinpoint.getPosY(DistanceUnit.INCH)-red_goal.getY(DistanceUnit.INCH)));
        telemetry.addData("distance", distance);
        telemetry.addData("angle", angle);
        panelsTelemetry.addData("flywheel speed", launcher.getFlywheelSpeed());
        telemetry.update();
        panelsTelemetry.update();
    }

    public void updateDrivetrain(double angle){

        if(gamepad1.dpad_up){
            drivetrain.RCDrive(0.67, 0,0);
        }
        else if(gamepad1.dpad_down){
            drivetrain.RCDrive(-0.67,0,0);
        }
        else if(gamepad1.dpad_left){
            drivetrain.RCDrive(0,0,0.4);
        }
        else if(gamepad1.dpad_right){
            drivetrain.RCDrive(0,0,-0.4);
        }
        else{
            if(gamepad1.a){
                if(limelight.getLatestResult().isValid()){
                    drivetrain.targetAngle(-gamepad1.left_stick_y,gamepad1.left_stick_x, 0 ,0.015, 0.01, limelight.getLatestResult().getTy(), 0.10);
                }
                else drivetrain.targetAngle(-gamepad1.left_stick_y, gamepad1.left_stick_x,angle);
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
