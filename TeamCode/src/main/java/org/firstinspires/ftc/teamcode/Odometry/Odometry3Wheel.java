package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Odometry3Wheel extends Thread{
    double wheelDiameter;
    double wheelCirc;
    double tickToInch;
    OdoWheel odoLeft;
    OdoWheel odoRight;
    OdoWheel odoMiddle;
    double x = 0;
    double y = 0;
    double angle = 0;
    long curTime=0;
    long lastTime=0;
    int cycleRate=10;
    int lastL;
    int lastR;
    int lastB;
    int curL;
    int curR;
    int curB;
    int dL; // odo delta left
    int dR; // odo delta right
    int dB; // odo delta middle
    double dFwd;
    double dStr;
    double dTheta;
    double r0;
    double r1;
    double relDX;
    double relDY;

    public Odometry3Wheel(int[] leftPos, int[] rightPos, int[] middlePos, Encoder left,
                          Encoder right, Encoder middle, double x, double y, double angle) {
        odoLeft = new OdoWheel(leftPos[0], leftPos[1], left); // [x, y]
        odoRight = new OdoWheel(rightPos[0], rightPos[1], right); // [x, y]
        odoMiddle = new OdoWheel(middlePos[0], middlePos[1], middle); // [x, y]
        wheelDiameter = 1.37795; // inches (35mm)
        wheelCirc = Math.PI*wheelDiameter;
        tickToInch = wheelCirc/8192.0;
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public void run() {
        try {
            curTime = System.currentTimeMillis();
            curL = odoLeft.getEncoderPos();
            curR = odoRight.getEncoderPos();
            curB = odoMiddle.getEncoderPos();

            if (curTime - lastTime >= cycleRate) {
                lastTime = System.currentTimeMillis();
                lastL = odoLeft.getEncoderPos();
                lastR = odoRight.getEncoderPos();
                lastB = odoMiddle.getEncoderPos();
                updateDeltaEncPos();
                updPos();
            }
        }
        catch (Exception e) {
            // chaught exception
        }
    }

    public void updPos() {
        calcDFwd();
        calcDTheta();
        calcDStr();
        calcAngle();
        if (dTheta != 0) {
            calcR0();
            calcR1();
            calcRelDX();
            calcRelDY();
            calcNewXY();
        } else {

        }

    }

    public void updateDeltaEncPos() {
        dL = curL - lastL;
        dR = curR - lastR;
        dB = curB - lastB;
    }

    public void calcDFwd() {
        dFwd = (dL+dR)/2.0;
    }
    public void calcDTheta() {
        dTheta = (dR-dL)/(odoLeft.getY_pos()- odoRight.getY_pos());
    }
    public void calcDStr() {
        dStr = dB-(odoMiddle.x_pos*dTheta);
    }
    public void calcAngle() {
        angle = angle+dTheta;
    }
    public void calcR0() {
        r0 = dFwd/dTheta;
    }
    public void calcR1() {
        r1 = dStr/dTheta;
    }
    public void calcRelDX() {
        if (dTheta != 0) {
            relDX = (r0*Math.sin(dTheta))-(r1*(1-Math.cos(dTheta)));
        } else {
            relDX = dFwd;
        }
    }
    public void calcRelDY() {
        if (dTheta != 0) {
            relDY = (r1*Math.sin(dTheta))+(r0*(1-Math.cos(dTheta)));
        } else {
            relDY = dStr;
        }
    }
    public void calcNewXY() {
        x = x+(relDX*Math.cos(angle))-(relDY*Math.sin(angle));
        y = y+(relDY*Math.cos(angle))-(relDX*Math.sin(angle));
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getAngle() {
        return angle;
    }

}
