package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.constants;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PID_Profile;

@Configurable
public class storage {

     public DcMotorEx storage_wheel_motor; //Motor that controls storage wheel
     public Servo loader;
     DistanceSensor distanceSensor;

     ElapsedTime elapsedTime;
     public PID_Profile pid;



     public enum servoState{
          SERVO_UP,
          SERVO_DOWN
     }

     public enum spindexState{
          slot1_in,
          slot1_out,
          slot2_in,
          slot2_out,
          slot3_in,
          slot3_out
     }

     public enum ballState{
          EMPTY,
          SPINDEX,
          BALL
     }

     spindexState lastState = spindexState.slot1_in;

     double[] stateLUT = {0, 45, 128, 166, 256, 303};

     static double kF = 0.06, kP = 0.01, kI = 0, kD = 0, max_velo = 150;

     public storage(HardwareMap hardwareMap, Telemetry telemetry){
          storage_wheel_motor = hardwareMap.get(DcMotorEx.class, constants.STORAGE_MOTOR_NAME);
          storage_wheel_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          storage_wheel_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          pid = new PID_Profile(kP, kI, kD, kF, max_velo);

          loader = hardwareMap.get(Servo.class, "loader servo");
          distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");

          telemetry.addData("STORAGE", "INITIALIZED");
     }

     public void updateStorage(spindexState spindexState, servoState servoState, ElapsedTime elapsedTime){
          pid.powerMotor(elapsedTime, storage_wheel_motor, storage_wheel_motor.getCurrentPosition());
     }
     /**
      * Whether the frontmost ball slot is empty or not
      * @return returns 0 if empty, 1 if sees plate, 2 if full
      */
     public int isAboveEmpty(){
          if(distanceSensor.getDistance(DistanceUnit.MM) >=110 ){
               return 0;
          }
          else if(distanceSensor.getDistance(DistanceUnit.MM) > 25 && distanceSensor.getDistance(DistanceUnit.MM) < 80){
               return 1;
          }
          else return 2;
     }

     public void turnSpindex(spindexState state){
          pid.setTarget(storage_wheel_motor.getCurrentPosition()+Angle.angleWrap(storage_wheel_motor.getCurrentPosition(), stateLUT[state.ordinal()], 384));
     }


     public spindexState cycle (spindexState input){
          switch (input){
               case slot1_in:
                    return spindexState.slot1_out;
               case slot1_out:
                    return spindexState.slot2_in;
               case slot2_in:
                    return spindexState.slot2_out;
               case slot2_out:
                    return spindexState.slot3_in;
               case slot3_in:
                    return spindexState.slot3_out;
               case slot3_out:
                    return spindexState.slot1_in;
          }
          return spindexState.slot1_in;
     }

     public double getDistanceSensorReading(){
          return distanceSensor.getDistance(DistanceUnit.MM);
     }

}
