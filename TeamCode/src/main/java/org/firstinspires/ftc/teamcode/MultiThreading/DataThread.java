package org.firstinspires.ftc.teamcode.MultiThreading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class DataThread extends Thread{
    List<LynxModule> allHubs;
    double ping1 = 0;
    double ping2 = 0;

    public DataThread(List<LynxModule> module) {
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void run() {
        try {
            ping2 = System.currentTimeMillis();
            if (ping2-ping1 > 100) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
                ping1 = ping2;
            }
        }
        catch (Exception e) {
            // chaught exception
        }
    }
}
