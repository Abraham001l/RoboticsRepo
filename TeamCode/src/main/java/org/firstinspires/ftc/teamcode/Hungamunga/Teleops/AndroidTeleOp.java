package org.firstinspires.ftc.teamcode.Hungamunga.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AndHMTeleOp",
        group = "Iterative Opmode")
public class AndroidTeleOp extends OpMode {

    BNO055IMU imu;
    double jTheta = 0;
    double rTheta = 0;
    double fTheta = 0;
    double rThetaRad = 0;
    double fRadTheta = 0.0;
    double mag = 0;
    double nX = 0;
    double nY = 0;
    double orgAngle;
    // Run time instantiation
    private ElapsedTime runtime = new ElapsedTime();

    // Motor Initialization & Instantiation to null
    DcMotorEx frontLeft,
            frontRight,
            backLeft,
            backRight = null;

    DcMotor VSLeft, VSRight = null;

    double vippowL = 0;
    double vippowR = 0;

    int targetPosL = 0;
    int targetPosR = 0;

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

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

        frontLeft.setVelocityPIDFCoefficients(5, 0, 0, 1);
        frontRight.setVelocityPIDFCoefficients(5, 0, 0, 1);
        backLeft.setVelocityPIDFCoefficients(5, 0, 0, 1);
        backRight.setVelocityPIDFCoefficients(5, 0, 0, 1);

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

        calcNewXY(strafe, drive);

        frontRightPower = -nY + nX + turn;
        frontLeftPower = -nY - nX - turn;
        backRightPower = -nY - nX + turn;
        backLeftPower = -nY + nX - turn;


        boolean high = gamepad1.y;
        boolean medium = gamepad1.x;
        boolean low = gamepad1.a;
        boolean base = gamepad1.b;

        boolean stack5 = gamepad1.dpad_up;
        boolean stack4 = gamepad1.dpad_left;
        boolean stack3 = gamepad1.dpad_right;
        boolean stack2 = gamepad1.dpad_down;

        // Viper slide variables
        double heightL = VSLeft.getCurrentPosition();
        double heightR = VSRight.getCurrentPosition();

        // Viper Slide button functions (TBD)
//        if (high && !(medium == low == base)) {
//            targetPosL = 3250;
//            targetPosR = 3250;
//        } else if (medium && !(high == low == base)) {
//            targetPosL = 2200;
//            targetPosR = 2200;
//        } else if (low && !(high == medium == base)) {
//            targetPosL = 1325;
//            targetPosR = 1325;
//        } else if (base && !(high == medium == low)) {
//            targetPosL = 0;
//            targetPosR = 0;
//        } else if (gamepad1.dpad_up) {
//            targetPosL = 650;
//            targetPosR = 650;
//        } else if (gamepad1.dpad_left) {
//            targetPosL = 450;
//            targetPosR = 450;
//        } else if (gamepad1.dpad_right) {
//            targetPosL = 325;
//            targetPosR = 325;
//        } else if (gamepad1.dpad_down) {
//            targetPosL = 200;
//            targetPosR = 200;
//        }

        if (heightL > 3000 || heightR > 3000) {
            modifier = 0.4;
        } else {
            modifier = 1;
        }

        if (gamepad1.right_trigger > 0 && targetPosL < 3000 && targetPosR < 3000) {
            targetPosL += 30;
            targetPosR += 30;
        } else if (gamepad1.left_trigger > 0 && targetPosL > 0 && targetPosR > 0) {
            targetPosL -= 30;
            targetPosR -= 30;
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

        VSLeft.setTargetPosition(targetPosL);
        VSLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VSLeft.setPower(vippowL);

        VSRight.setTargetPosition(targetPosR);
        VSRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VSRight.setPower(vippowR);

        frontRight.setVelocity(frontRightPower * 3000);
        frontLeft.setVelocity(frontLeftPower * 3000);
        backRight.setVelocity(backRightPower * 3000);
        backLeft.setVelocity(backLeftPower * 3000);

        telemetry.update();
    }

    public void calcNewXY(double x, double y) {
        orgAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        orgAngle = orgAngle+90;
        if (orgAngle < 0) {
            orgAngle = orgAngle+360;
        }

        if (orgAngle > 360) {
            orgAngle = orgAngle - 360;
        }
        rTheta = orgAngle;
        rThetaRad = rTheta*(Math.PI/180.0);
        double cosTheta = Math.cos(rThetaRad);
        double sinTheta = Math.sin(rThetaRad);
        nX = (x*sinTheta)-(y*cosTheta);
        nY = (x*cosTheta)+(y*sinTheta);
//        if (x == 0) {
//            if (y > 0) {
//                jTheta = 90;
//            } else if (y < 0) {
//                jTheta = 270;
//            }
//        } else {
//            jTheta = Math.atan(y/x);
//            if (x < 0) {
//                if (y > 0) {
//                    jTheta = 180+jTheta;
//                }
//                else if (y < 0) {
//                    telemetry.addData("RAN: ", "true");
//                    jTheta = 180+jTheta;
//                } else if (y == 0) {
//                    jTheta = 180;
//                }
//            }
//        }
//
//        fTheta = jTheta - rTheta + 90;
//        fRadTheta = fTheta*(Math.PI/180.0);
//        mag = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
//        nX = Math.cos(fRadTheta)*mag;
//        nY = Math.sin(fRadTheta)*mag;
    }


    @Override
    public void stop() {}

}

