package org.firstinspires.ftc.teamcode.BlackSmith;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.annotations.EvalOnGo;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

public class RacoonBot  extends BlackOp {

    // Instantiating Drive Motors
    DcMotorEx frontLeft,
            frontRight,
            backLeft,
            backRight,
            arm = null;

    // Instantiating GamePad Listener
    ReforgedGamepad driver1 = new ReforgedGamepad(gamepad1);
    ReforgedGamepad driver2 = new ReforgedGamepad(gamepad2);

    double drive = 0;
    double turn = 0;

    double frontLeftPower = 0;
    double frontRightPower = 0;
    double backLeftPower = 0;
    double backRightPower = 0;

    // Arm state 0 means down arm state 1 means up
    int armState = 0;
    int upPosition = 1000;
    int downPosition = 0;


    @Override
    public void go() {
        // Assigning Drive Motors
        frontLeft = InstantiateMotor("frontLeft");
        frontRight = InstantiateMotor("frontRight");
        backLeft = InstantiateMotor("backLeft");
        backRight = InstantiateMotor("backRight");
        arm = InstantiateMotor("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // arm Control
        driver1.a.onRise(() -> updateArmState());

        Scheduler.launchOnStart(this, () -> {
            UpdateChassisControls();
            Drive();
        });
    }

    public void updateArmState() {
        if (armState == 0) {
            armState = 1;
            setArmUp();
        } else if (armState == 1) {
            armState = 0;
            setArmDown();
        }
    }

    public void setArmUp() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(upPosition);
        arm.setPower(.3);
    }

    public void setArmDown() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(downPosition);
        arm.setPower(-.3);
    }

    public void UpdateChassisControls() {
        // Controller Chassis Inputs
        drive = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
    }

    public void Drive() {
        // Drive Equations
        frontLeftPower = drive + turn;
        frontRightPower = drive - turn;
        backLeftPower = drive + turn;
        backRightPower = drive - turn;

        // Motor Control
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public DcMotorEx InstantiateMotor(String mName) {
        return hwMap().get(DcMotorEx.class, mName);
    }
}
