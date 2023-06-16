package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Odometry2Wheel extends Thread {
    double wheel_diameter; // in
    double wheel_circ; // in
    double tick_to_inch;
    IMU imu;
    DcMotorEx enc_m; // middle
    DcMotorEx enc_l; // left
    DcMotorEx enc_r; // right
    double r_m;
    double r_l;
    double r_r;
    boolean lefty;
    double angle = 0;
    long curr_time = 0;
    long last_time = 0;
    int cycle_rate;
    int last_middle;
    int last_left;
    int last_right;
    double last_angle;
    public double curr_actual_angle;
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
    public double x = 0;
    public double y = 0;
    public int runs = 0;

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
        while (true) {
            runs++;
            curr_time = System.currentTimeMillis();
            double curr_angle = conv_angle(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//            int curr_middle = enc_m.getCurrentPosition();
            int curr_middle = Odo2Tester.front_left.getCurrentPosition();
            int curr_left = 0;
            int curr_right = 0;
            if (lefty) {
//                curr_left = enc_l.getCurrentPosition();
                curr_left = Odo2Tester.back_right.getCurrentPosition();
            }
            else { curr_right = enc_r.getCurrentPosition();}

            if (curr_time - last_time >= cycle_rate) {
                last_time = System.currentTimeMillis();
                int d_mid_enc = curr_middle - last_middle;
                last_middle = curr_middle;
                int d_left_enc = 0;
                int d_right_enc = 0;
                if (lefty) {
                    d_left_enc = curr_left - last_left;
                    last_left = curr_left;
                } else {
                    d_right_enc = curr_right - last_right;
                }
                d_m = d_mid_enc * tick_to_inch;
                d_l = d_left_enc * tick_to_inch;
                d_r = d_right_enc * tick_to_inch;

                d_theta = curr_angle - last_angle;
                curr_actual_angle += d_theta;
                d_theta = Math.toRadians(d_theta);
                last_angle = conv_angle(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                while (curr_actual_angle > 360 || curr_actual_angle < 0) {
                    if (curr_actual_angle > 360) {
                        curr_actual_angle -= 360;
                    } else {
                        curr_actual_angle += 360;
                    }
                }
                upd_pos();
            }
        }

    }

    public void upd_pos() {
        calc_fwd();
        calc_str();
        if (d_theta != 0) {
            calc_r_0();
            calc_r_1();
        }
        calc_rel_x();
        calc_rel_y();
        calc_x();
        calc_y();
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

    public Object calc_rel_x() {
        if (d_theta != 0) {
            rel_x = (r_0*Math.sin(d_theta)) - (r_1*(1 - Math.cos(d_theta)));
            return null;
        }
        rel_x = d_fwd;
        return null;
    }

    public Object calc_rel_y() {
        if (d_theta != 0) {
            rel_y = (r_1*Math.sin(d_theta)) + (r_0*(1 - Math.cos(d_theta)));
            return null;
        }
        rel_y = d_str;
        return null;
    }

    public void calc_x() {
        x = x+(rel_x*Math.cos(d_theta))-(rel_y*Math.sin(d_theta));
    }

    public void calc_y() {
        y = y+(rel_y*Math.cos(d_theta))+(rel_x*Math.sin(d_theta));
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

    public int getFL() {
        return Odo2Tester.front_left.getCurrentPosition();
    }
}
