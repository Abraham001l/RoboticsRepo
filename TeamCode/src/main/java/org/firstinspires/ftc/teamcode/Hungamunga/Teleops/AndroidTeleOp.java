package org.firstinspires.ftc.teamcode.Hungamunga.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AndroidTeleop",
        group = "Iterative Opmode")
public class AndroidTeleOp extends OpMode {

    // Run time instantiation
    private ElapsedTime runtime = new ElapsedTime();

    // Motor Initialization & Instantiation to null
    DcMotorEx frontLeft,
            frontRight,
            backLeft,
            backRight = null;

    // Viper Motors
    DcMotorEx VSLeft = null;
    double vippowL = 0;
    double targetPos = 0;

    DcMotorEx VSRight = null;
    double vippowR = 0;

    // Sero Initialization & Instantiation to null
    Servo claw, wrist, armLeft, armRight = null;
    double arm = 0;

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

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Lift motor to hardwareMap
        //Viper Slide Motor
        VSLeft = hardwareMap.get(DcMotorEx.class, "VSLeft");
        VSRight = hardwareMap.get(DcMotorEx.class, "VSRight");

        VSLeft.setDirection(DcMotorEx.Direction.REVERSE);
        VSRight.setDirection(DcMotorEx.Direction.FORWARD);

        VSLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        VSRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        VSLeft.setMode(DcMotorEx.RunMode.RESET_ENCODERS);
        VSRight.setMode(DcMotorEx.RunMode.RESET_ENCODERS);

        VSLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        VSRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        VSLeft.setVelocityPIDFCoefficients(5, 0,0,0);
//        VSRight.setVelocityPIDFCoefficients(5, 0,0,0);




        // Servo to hardwareMap
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override

    public void loop() {

        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // Viper buttons
        boolean high = gamepad1.y;
        boolean medium = gamepad1.x;
        boolean low = gamepad1.a;
        boolean base = gamepad1.b;

        boolean stack5 = gamepad1.dpad_up;
        boolean stack4 = gamepad1.dpad_left;
        boolean stack3 = gamepad1.dpad_right;
        boolean stack2 = gamepad1.dpad_down;

        frontRightPower = -drive + strafe + turn;
        frontLeftPower = -drive - strafe - turn;
        backRightPower = -drive - strafe + turn;
        backLeftPower = -drive + strafe - turn;

        // Viper slide variables
        double heightL = VSLeft.getCurrentPosition();
        double heightR = VSRight.getCurrentPosition();

        // Viper Slide button functions
        if (high && !(medium == low == base)) {
            targetPos = 3119; // 3080
        } else if (medium && !(high == low == base)) {
            targetPos = 2167; // 2325
        } else if (low && !(high == medium == base)) {
            targetPos = 1324;
        } else if (base && !(high == medium == low)) {
            targetPos = 60;
        } else if (gamepad1.dpad_up) {
            targetPos = 646;
        } else if (gamepad1.dpad_left) {
            targetPos = 448;
        } else if (gamepad1.dpad_right) {
            targetPos = 323;
        } else if (gamepad1.dpad_down) {
            targetPos = 200;
        }

        if (heightL > 3000 || heightR > 3000) {
            modifier = 0.5;
        } else {
            modifier = 1;
        }

        // Set Power based on Current position V.S. targEt position
        if (gamepad1.right_trigger > 0) {
            targetPos += 23;
        } else if (gamepad1.left_trigger > 0 && targetPos > 0) {
            targetPos -= 23;
        }

        if (heightL < targetPos) {
            vippowL = 1;
        } else if (heightL > targetPos) {
            vippowL = -1;
        } else if (heightL == targetPos) {
            vippowL = 0;
        }

        if (heightR < targetPos) {
            vippowR = 1;
        } else if (heightR > targetPos) {
            vippowR = -1;
        } else if (heightR == targetPos) {
            vippowR = 0;
        }

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

        // Viper Slide directions
        VSLeft.setTargetPosition((int) -targetPos);
        VSLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VSLeft.setPower(vippowL);

        VSRight.setTargetPosition((int) -targetPos);
        VSRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VSRight.setPower(vippowR);

//        claw.setPosition(clawPos);

        telemetry.addData("Value", heightL);
        telemetry.addData("Value", heightR);
        telemetry.addData("Status", "Running");
        telemetry.addData("TargetPos", targetPos);
        telemetry.addData("leftFrontPower: ", frontLeft.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {}
}

