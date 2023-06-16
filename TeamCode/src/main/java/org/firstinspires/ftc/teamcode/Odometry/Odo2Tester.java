package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp()
public class Odo2Tester extends OpMode {
    IMU imu;
    public static DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    public static DcMotorEx back_right;
    Odometry2Wheel odo;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        PhotonCore.enable();
        front_left = hardwareMap.get(DcMotorEx.class, "frontLeft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontRight");
        back_left = hardwareMap.get(DcMotorEx.class, "backLeft");
        back_right = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        odo = new Odometry2Wheel(front_left, back_right, null, true, 1.37795,
                imu, 90, 3.5, 2.5, 0);
        odo.start();
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = -drive + strafe + turn;
        double frontRightPower = -drive - strafe - turn;
        double backLeftPower = drive + strafe - turn;
        double backRightPower = drive - strafe + turn;

        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower);
        this.drive.setPoseEstimate(new Pose2d(odo.x, odo.y, odo.curr_actual_angle));
        this.drive.update();
        telemetry.addData("x: ", odo.x);
        telemetry.addData("y: ", odo.y);
        telemetry.addData("angle: ", Math.toRadians(odo.curr_actual_angle));
        telemetry.addData("runs: ", odo.runs);
        telemetry.addData("front_left", front_left.getCurrentPosition());
        telemetry.addData("curr_middle: ", odo.getFL());
        telemetry.addData("back_right", back_right.getCurrentPosition());
        telemetry.addData("time: ", odo.curr_time);
        telemetry.update();
    }
}
