package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Odometry2Wheel extends Thread {
    double wheel_diameter; // in
    double wheel_circ; // in
    double tot_ticks;
    double tick_to_inch;
    IMU imu;
    DcMotorEx enc_m; // middle
    DcMotorEx enc_l; // left
    DcMotorEx enc_r; // right
    double r_m;
    double r_l;
    double r_r;
    boolean lefty;
    double x = 0;
    double y = 0;
    double angle = 0;
    long curr_time = 0;
    long last_time = 0;
    int cycle_rate;
    int last_middle;
    int last_left;
    int last_right;
    double last_angle;
    double curr_actual_angle;
    double d_m;
    double d_l;
    double d_r;
    double d_fwd;
    double d_theta;
    double d_str;
    double r_0;
    double r_1;
    double rel_x;
    double rel_y;

    public Odometry2Wheel(DcMotorEx left, DcMotorEx middle, DcMotorEx right, boolean lefty,
                          double wheel_diameter, IMU imu, double start_angle, double r_m,
                          double r_l, double r_r) {
        enc_m = middle;
        enc_l = left;
        enc_r = right;
        this.lefty = lefty;
        this.wheel_diameter = wheel_diameter;
        wheel_circ = Math.PI*wheel_diameter;
        tick_to_inch = wheel_circ/8192.0;
        this.imu = imu;
        curr_actual_angle = start_angle;
        this.r_m = r_m;
        this.r_l = r_l;
        this.r_r = r_r;
    }

    public void run() {
        curr_time = System.currentTimeMillis();
        double curr_angle = conv_angle(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        int curr_middle = enc_m.getCurrentPosition();
        int curr_left = 0;
        int curr_right = 0;
        if (lefty) { curr_left = enc_l.getCurrentPosition();}
        else { curr_right = enc_r.getCurrentPosition();}

        if (curr_time - last_time >= cycle_rate) {
            last_time = System.currentTimeMillis();
            int d_mid_enc = curr_middle - last_middle;
            int d_left_enc = 0;
            int d_right_enc = 0;
            if (lefty) {
                d_left_enc = curr_left - last_left;
            } else {
                d_right_enc = curr_right - last_right;
            }
            d_m = d_mid_enc * tick_to_inch;
            d_l = d_left_enc * tick_to_inch;
            d_r = d_right_enc * tick_to_inch;

            d_theta = curr_angle - last_angle;
            curr_actual_angle += d_theta;
            last_angle = conv_angle(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        }
    }

    public void upd_pos() {
        calc_fwd();
        calc_str();
        if (d_theta != 0) {
            calc_r_0();
            calc_r_1();
        }
    }

    public void calc_fwd() {
        if (lefty) {
            d_fwd = d_l - (r_l * d_theta);
        }
    }

    public void calc_str() {
        if (lefty) {
            d_str = d_m - (r_m * d_theta);
        }
    }

    public void calc_r_0() {
        r_0 = d_fwd/d_theta;
    }

    public void calc_r_1() {
        r_1 = d_str/d_theta;
    }

    public double conv_angle(double uncov_angle) {
        if (uncov_angle >= 0) {
            return (uncov_angle+90);
        } else {
            double stp_1 = uncov_angle + 450;
            if (stp_1 > 360) {
                return (stp_1-360);
            }
            return stp_1;
        }
    }
}
