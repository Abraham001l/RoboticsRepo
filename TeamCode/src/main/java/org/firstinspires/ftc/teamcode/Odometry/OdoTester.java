package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class OdoTester extends OpMode {
    Encoder leftEnc;
    Encoder rightEnc;
    Encoder middleEnc;
    Odometry3Wheel odometry;
    int[] leftEncPos = {0,5}; // [x,y] in inches
    int[] rightEncPos = {0,-5}; // [x,y] in inches
    int[] middleEncPos = {-5,0}; // [x,y] in inches



    @Override
    public void init() {
        leftEnc = hardwareMap.get(Encoder.class, "leftEnc");
        rightEnc = hardwareMap.get(Encoder.class, "rightEnc");
        middleEnc = hardwareMap.get(Encoder.class, "middleEnc");
//        odometry = new Odometry3Wheel()

    }

    @Override
    public void loop() {

    }
}
