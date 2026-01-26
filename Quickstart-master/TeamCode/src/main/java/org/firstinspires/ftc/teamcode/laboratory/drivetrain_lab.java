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

//Steps:
//1. Individual motor testing
//2. Translational Drive
//3. RC Drive
//4. Get Heading from Odo
//5. FC Reset
//6. FC Drive
//

@Configurable
@TeleOp(name = "drivetrain lab(ubu)")
public class drivetrain_lab extends OpMode {

    //
    DcMotorEx lf, rf, lb, rb;
    String motornames[] = {"lf", "rf", "lb", "rb"};
    GoBildaPinpointDriver pinpoint;
    static double kP = 0.06, kF = 0.2, kD = 0;
    double SLOW_SPEED = 0.2;
    double lastError = 0;

    @Override
    public void init(){
        lf = hardwareMap.get(DcMotorEx.class, motornames[0]);
        rf = hardwareMap.get(DcMotorEx.class, motornames[1]);
        lb = hardwareMap.get(DcMotorEx.class, motornames[2]);
        rb = hardwareMap.get(DcMotorEx.class, motornames[3]);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        configurePinpoint();

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        pinpoint.update();
        if(gamepad1.b){
            targetAngle(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.5*Math.PI);
        }
        else FC_Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x,pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("heading", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.update();

        if(gamepad1.x){
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), AngleUnit.RADIANS, 0));
        }
    }

    public void testMotor(DcMotorEx motor, boolean condition){
        if(condition){
            motor.setPower(SLOW_SPEED);
        }
        else motor.setPower(0);
    }

    public void RCDrive(double fwd, double strf, double rot){
        double lfp = fwd + strf - rot;
        double rfp = fwd - strf + rot;
        double lbp = fwd - strf - rot;
        double rbp = fwd + strf + rot;

        double max = Math.max((Math.abs(fwd) + Math.abs(strf) + Math.abs(rot)), 1);

        lfp = lfp /max;
        rfp = rfp /max;
        lbp = lbp /max;
        rbp = rbp /max;

        lf.setPower(lfp);
        rf.setPower(rfp);
        lb.setPower(lbp);
        rb.setPower(rbp);
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
    public void FC_Drive(double fwd, double strf, double rot, double heading){
        double rotX = strf * Math.cos(-heading) - fwd * Math.sin(-heading);
        double rotY = strf * Math.sin(-heading) + fwd * Math.cos(-heading);

        RCDrive(rotY, rotX, rot);
    }

    public void targetAngle(double fwd, double strf, double angleRadians){
        double error = pinpoint.getHeading(AngleUnit.RADIANS)-angleRadians;
        double rotPower = (kF*Math.signum(error)+kP*(error)+kD*(error-lastError));
        FC_Drive(fwd, strf, rotPower, pinpoint.getHeading(AngleUnit.RADIANS));
        lastError = error;
    }
}
