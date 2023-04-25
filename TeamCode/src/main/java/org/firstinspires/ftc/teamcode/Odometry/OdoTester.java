package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

public class OdoTester extends OpMode {
    BulkReadThread bulkReadThread;
    List<LynxModule> allHubs;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Encoder leftEnc;
    Encoder rightEnc;
    Encoder middleEnc;
    Odometry3Wheel odometry;
    int[] leftEncPos = {0,5}; // [x,y] in inches
    int[] rightEncPos = {0,-5}; // [x,y] in inches
    int[] middleEncPos = {-5,0}; // [x,y] in inches
    double startXPos = 0;
    double startYPos = 0;
    double startAngle = 0;



    @Override
    public void init() {
        PhotonCore.enable();
        leftEnc = hardwareMap.get(Encoder.class, "leftEnc");
        rightEnc = hardwareMap.get(Encoder.class, "rightEnc");
        middleEnc = hardwareMap.get(Encoder.class, "middleEnc");
        allHubs = hardwareMap.getAll(LynxModule.class);
        odometry = new Odometry3Wheel(leftEncPos, rightEncPos, middleEncPos, leftEnc, rightEnc,
                middleEnc, startXPos, startYPos, startAngle);
        bulkReadThread = new BulkReadThread(allHubs);
        bulkReadThread.start();
        odometry.start();
    }

    @Override
    public void loop() {
        drive.setPoseEstimate(new Pose2d(odometry.getX(), odometry.getY(), odometry.getAngle()));
        drive.update();
    }
}
