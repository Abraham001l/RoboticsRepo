package org.firstinspires.ftc.teamcode.MultiThreading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

public class MainRunner extends OpMode {
    MechanismThread mechanismThread;
    List<LynxModule> allHubs;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        mechanismThread = new MechanismThread(hardwareMap, gamepad1, "motor1");
        mechanismThread.start();
    }

    @Override
    public void loop() {

    }
}
