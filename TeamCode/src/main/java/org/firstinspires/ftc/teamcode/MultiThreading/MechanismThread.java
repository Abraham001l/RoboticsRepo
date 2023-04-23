package org.firstinspires.ftc.teamcode.MultiThreading;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class MechanismThread extends Thread {
    DcMotorEx motor1;
    Gamepad gamepad1;

    public MechanismThread(HardwareMap hardwareMap, Gamepad gamepad1, String m1Tag) {
        motor1 = hardwareMap.get(DcMotorEx.class, m1Tag);
        this.gamepad1 = gamepad1;
    }

    public void run() {
        try {
            motor1.setPower(gamepad1.right_trigger);
        }
        catch (Exception e) {
            // chaught exception
        }
    }
}
