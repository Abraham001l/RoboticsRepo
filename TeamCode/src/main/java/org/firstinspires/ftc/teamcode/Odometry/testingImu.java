package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp()
public class testingImu extends OpMode {
    IMU imu;
    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;

    @Override
    public void init() {
        front_left = hardwareMap.get(DcMotorEx.class, "frontLeft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontRight");
        back_left = hardwareMap.get(DcMotorEx.class, "backLeft");
        back_right = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");

    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = drive - strafe - turn;
        double frontRightPower = drive + strafe + turn;
        double backLeftPower = -drive - strafe + turn;
        double backRightPower = drive - strafe + turn;

        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower);

        telemetry.addData("imu: ", imu.getRobotAngularVelocity(AngleUnit.DEGREES));
        telemetry.addData("angle: ", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("true_angle: ", conv_angle(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle));
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
