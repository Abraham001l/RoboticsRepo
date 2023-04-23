package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class OdoWheel {
    double x_pos;
    double y_pos;
    Encoder encoder;

    public OdoWheel(double x_pos, double y_pos, Encoder encoder) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        this.encoder = encoder;
    }

    public int getEncoderPos() {
        return encoder.getCurrentPosition();
    }

    public double getX_Pos() {
        return x_pos;
    }

    public double getY_pos() {
        return y_pos;
    }
}
