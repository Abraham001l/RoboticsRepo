package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

public class BulkReadThread extends Thread{
    List<LynxModule> allHubs;
    double lastTime = 0;
    double curTime = 0;
    double frequency = 10;

    public BulkReadThread(List<LynxModule> module) {
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void run() {
        try {
            curTime = System.currentTimeMillis();
            if (curTime-lastTime > frequency) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
                lastTime = curTime;
            }
        }
        catch (Exception e) {
            // chaught exception
        }
    }
}
