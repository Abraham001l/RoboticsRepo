package org.firstinspires.ftc.teamcode.Hungamunga;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slides",
        group = "Iterative Opmode")
public class AndHMTeleop extends OpMode {

    // Run time instantiation
    private ElapsedTime runtime = new ElapsedTime();

    // Motor Initialization & Instantiation to null
//    DcMotorEx frontLeft,
//            frontRight,
//            backLeft,
//            backRight = null;

    // Viper Motors
    DcMotor VSLeft, VSRight = null;

    double vippowL = 0;
    double vippowR = 0;

    int targetPosL = 0;
    int targetPosR = 0;

    // Sero Initialization & Instantiation to null
    Servo claw = null;

    // Gamepad Connections

    float drive = 0;
    float strafe = 0;
    double turn = 0;
    double modifier = 0.3;
    double clawPos = 1;

    // Motor Power
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    @Override
    public void init() {

        // Motor instantiation to hardwareMap

        //Drive Motors

//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Lift motor to hardwareMap
        //Viper Slide Motor
        VSLeft = hardwareMap.get(DcMotor.class, "VSLeft");
        VSRight = hardwareMap.get(DcMotor.class, "VSRight");

        VSLeft.setDirection(DcMotor.Direction.FORWARD);
        VSRight.setDirection(DcMotor.Direction.REVERSE);

        VSLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VSRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VSLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        VSRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        VSLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VSRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo to hardwareMap
//        claw = hardwareMap.get(Servo.class, "claw");

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override

    public void loop() {

        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x / 1.1;

        // Viper buttons
        boolean high = gamepad1.y;
        boolean medium = gamepad1.x;
        boolean low = gamepad1.a;
        boolean base = gamepad1.b;

        boolean stack5 = gamepad1.dpad_up;
        boolean stack4 = gamepad1.dpad_left;
        boolean stack3 = gamepad1.dpad_right;
        boolean stack2 = gamepad1.dpad_down;

        frontRightPower = drive + strafe + turn;
        frontLeftPower = drive - strafe - turn;
        backRightPower = drive - strafe + turn;
        backLeftPower = drive + strafe - turn;

        // Viper slide variables
        double heightL = VSLeft.getCurrentPosition();
        double heightR = VSRight.getCurrentPosition();

        // Viper Slide button functions
        if (high && !(medium == low == base)) {
            targetPosL = 3250;
            targetPosR = 3250;
        } else if (medium && !(high == low == base)) {
            targetPosL = 2200;
            targetPosR = 2200;
        } else if (low && !(high == medium == base)) {
            targetPosL = 1325;
            targetPosR = 1325;
        } else if (base && !(high == medium == low)) {
            targetPosL = 0;
            targetPosR = 0;
            claw.setPosition(0.32);
        } else if (gamepad1.dpad_up) {
            targetPosL = 650;
            targetPosR = 650;
        } else if (gamepad1.dpad_left) {
            targetPosL = 450;
            targetPosR = 450;
        } else if (gamepad1.dpad_right) {
            targetPosL = 325;
            targetPosR = 325;
        } else if (gamepad1.dpad_down) {
            targetPosL = 200;
            targetPosR = 200;
        }

        if (heightL > 3000 || heightR > 3000) {
            modifier = 0.4;
        } else {
            modifier = 1;
        }

        if (gamepad1.right_trigger > 0) {
            targetPosL += 14;
            targetPosR += 14;
        } else if (gamepad1.left_trigger > 0 && targetPosL > 0 && targetPosR > 0) {
            targetPosL -= 14;
            targetPosR -= 14;
        }

        // Set Power based on Current position V.S. targEt position
        if (heightL < targetPosL) {
            vippowL = 1;
        } else if (heightL > targetPosL) {
            vippowL = 1;
        } else if (heightL == targetPosL) {
            vippowL = 0;
        }

        if (heightR < targetPosR) {
            vippowR = 1;
        } else if (heightR > targetPosR) {
            vippowR = 1;
        } else if (heightR == targetPosR) {
            vippowR = 0;
        }

//        frontRight.setVelocity(frontRightPower * modifier * 1150);
//        frontLeft.setVelocity(frontLeftPower * modifier * 1150);
//        backRight.setVelocity(backRightPower * modifier * 1150);
//        backLeft.setVelocity(backLeftPower * modifier * 1150);

        if (gamepad1.left_bumper) {
            clawPos = 0;
        } else {
            clawPos = 1;
        }

//        if (gamepad1.right_trigger > 0) {
//            vippowL = .8;
//            vippowR = .8;
//        } else if(gamepad1.left_trigger > 0) {
//            vippowL = -.8;
//            vippowR = -.8;
//        } else {
//            vippowL = .0;
//            vippowR = .0;
//        }

        // Viper Slide directions
        VSLeft.setTargetPosition(targetPosL);
        VSLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VSLeft.setPower(vippowL);

        VSRight.setTargetPosition(targetPosR);
        VSRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VSRight.setPower(vippowR);

//        claw.setPosition(clawPos);

        telemetry.addData("Value", heightR);
        telemetry.addData("Value", heightL);
        telemetry.addData("Value", targetPosR);
        telemetry.addData("value", targetPosL);
        telemetry.addData("trigger", gamepad1.right_trigger);
        telemetry.addData("Status", "Running");
        telemetry.addData("lpower: ", VSLeft.getPower());
        telemetry.addData("rpower: ", VSRight.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {}
}





