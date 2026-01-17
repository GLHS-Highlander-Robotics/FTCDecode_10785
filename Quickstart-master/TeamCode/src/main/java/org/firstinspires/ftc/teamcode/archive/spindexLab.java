package org.firstinspires.ftc.teamcode.archive;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.storage;

@TeleOp(name = "spindex lab", group = "lab")
public class spindexLab extends OpMode {
    //Step 1: Find Values for distance sensor readings
    //Step 2: Detect whether spindex is empty, full or not.
    //Step 3: Make Changes based on it.
    //Step 4: Incorporate Servo

    storage storage;
    storage.spindexState spindexState = org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot1_in;
    storage.servoState servoState = org.firstinspires.ftc.teamcode.subsystems.storage.servoState.SERVO_DOWN;
    storage.ballState ballState = org.firstinspires.ftc.teamcode.subsystems.storage.ballState.BALL;

    ElapsedTime elapsedTime;
    boolean a = false;

    @Override
    public void init(){
        storage = new storage(hardwareMap, telemetry);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void loop(){
//        //UNIVERSAL
//        if(gamepad1.left_trigger>=0.1 && !(spindexState == org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot1_in||spindexState == org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot2_in||spindexState == org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot3_in)){
//            servoState = org.firstinspires.ftc.teamcode.subsystems.storage.servoState.SERVO_UP;
//        }
//        else servoState = org.firstinspires.ftc.teamcode.subsystems.storage.servoState.SERVO_DOWN;
//        //MOVING
//        updateAutoLogic(!a && gamepad1.a);
//        storage.turnSpindex(spindexState);
//        storage.updateStorage(spindexState, servoState, elapsedTime);
//        //SHOOTING
//        spindexState = org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot1_out;
//
//        spindexState = storage.cycle(spindexState);
//        spindexState = storage.cycle(spindexState);
//
//
//        //UNIVERSAL
//        a = gamepad1.a;




        telemetry.addData("state", spindexState);
        telemetry.addData("position", storage.storage_wheel_motor.getCurrentPosition());
        telemetry.update();
    }

    public void updateAutoLogic(boolean trigger){
        switch (ballState){
            case EMPTY:
                if(trigger||!storage.pid.isBusy&&(storage.isAboveEmpty() == 2 && (spindexState == org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot1_in||spindexState == org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot2_in||spindexState == org.firstinspires.ftc.teamcode.subsystems.storage.spindexState.slot3_in))){
                    spindexState = storage.cycle(spindexState);
                    spindexState = storage.cycle(spindexState);
                    ballState = org.firstinspires.ftc.teamcode.subsystems.storage.ballState.BALL;
                }
                break;
            case BALL:
                if((!storage.pid.isBusy && storage.isAboveEmpty() == 0)||trigger){
                    ballState = org.firstinspires.ftc.teamcode.subsystems.storage.ballState.EMPTY;
                }
        }
    }
}
