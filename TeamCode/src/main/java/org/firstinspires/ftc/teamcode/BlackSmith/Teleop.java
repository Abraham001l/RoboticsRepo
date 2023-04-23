package org.firstinspires.ftc.teamcode.BlackSmith;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.annotations.EvalOnGo;
import ftc.rogue.blacksmith.listeners.Pulsar;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

public class Teleop  extends BlackOp {

    // Instantiating Drive Motors
    DcMotorEx frontLeft,
            frontRight,
            backLeft,
            backRight = null;
    
    // Instantiating GamePad Listener
    ReforgedGamepad driver1 = new ReforgedGamepad(gamepad1);
    ReforgedGamepad driver2 = new ReforgedGamepad(gamepad2);

    double drive = 0;
    double strafe = 0;
    double turn = 0;

    double frontLeftPower = 0;
    double frontRightPower = 0;
    double backLeftPower = 0;
    double backRightPower = 0;

    @Override
    public void go() {
        // Assigning Drive Motors
        frontLeft = InstantiateMotor("frontLeft");
        frontRight = InstantiateMotor("frontRight");
        backLeft = InstantiateMotor("backLeft");
        backRight = InstantiateMotor("backRight");

        Scheduler.launchOnStart(this, () -> {
            UpdateChassisControls();
            Drive();
        });
    }

    public void UpdateChassisControls() {
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
    }

    public void Drive() {
        frontLeftPower = drive + strafe + turn;
        frontRightPower = drive - strafe - turn;
        backLeftPower = drive - strafe + turn;
        backRightPower = drive + strafe - turn;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public DcMotorEx InstantiateMotor(String mName) {
        return hwMap().get(DcMotorEx.class, mName);
    }
}
