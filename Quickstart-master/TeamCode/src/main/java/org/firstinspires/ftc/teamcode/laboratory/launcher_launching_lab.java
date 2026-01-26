package org.firstinspires.ftc.teamcode.laboratory;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
@TeleOp(name = "launcher launch lab")
public class launcher_launching_lab extends OpMode {

    DcMotorEx launcher, transfer, intake;

    double distance = 0;
    GoBildaPinpointDriver pinpoint;
    static Pose2D initialPos = new Pose2D(DistanceUnit.INCH, 9, 64, AngleUnit.RADIANS, 0), reference = new Pose2D(DistanceUnit.INCH, 144, 0, AngleUnit.RADIANS, 0);
    static double targetSpeed = 600;
    boolean a, b, x, y, up, dn, lt, rt;
    static double kF = 0.19, kP = 0.07;
    boolean setting_kP = false;

    @Override
    public void init(){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        pinpoint.setPosition(initialPos);
        configurePinpoint();
    }

    @Override
    public void loop(){
        pinpoint.update();

        if(gamepad1.b){
            transfer.setPower(0.5);
            intake.setPower(0.5);
        }
        else{
            transfer.setPower(0);
            intake.setPower(0);
        }

        distance = Math.hypot((pinpoint.getPosX(DistanceUnit.INCH)-reference.getX(DistanceUnit.INCH)), (pinpoint.getPosY(DistanceUnit.INCH)-reference.getY(DistanceUnit.INCH)));

        powerFlywheel(targetSpeed);

        prepareTelemetry();
        telemetry.update();
    }

    void prepareTelemetry(){
        telemetry.addData("kF", kF);
        telemetry.addData("kP", kP);
        telemetry.addData("target speed", targetSpeed);
        telemetry.addData("motor speed", launcher.getVelocity());
        telemetry.addData("motor power", launcher.getPower());
        telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("distance", distance);
    }

    void powerFlywheel(double targetSpeed){
        if(targetSpeed == 0){
            launcher.setPower(kP*(targetSpeed-launcher.getVelocity()));
        }
        else launcher.setPower(kF+kP*(targetSpeed-launcher.getVelocity()));
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(0, 0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
