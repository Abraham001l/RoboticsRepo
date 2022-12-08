package org.firstinspires.ftc.teamcode.Autonomous.RoadRunnerAutons;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "RedHungaMungaAuton22pts")
public class RedHungaMungaAuton22pts extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    Servo claw = null;

    DcMotor viperSlide = null;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        claw = hardwareMap.get(Servo.class, "claw");

        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");

        viperSlide.setDirection(DcMotor.Direction.FORWARD);

        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int targetPos = 0;
        int height = 0;

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory leftTrajectorypt0 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(33)
                .build();
        Trajectory leftTrajectorypt1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(33)
                .build();
        Trajectory leftTrajectorypt2 = drive.trajectoryBuilder(new Pose2d())
                .forward(45)
                .build();
        Trajectory leftTrajectorypt3 = drive.trajectoryBuilder(new Pose2d())
                .back(22)
                .build();
        Trajectory leftTrajectorypt4 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(36)
                .build();

        Trajectory middleTrajectorypt1 = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();

        Trajectory rightTrajectorypt0 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(33)
                .build();
        Trajectory rightTrajectorypt1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(33)
                .build();
        Trajectory rightTrajectorypt2 = drive.trajectoryBuilder(new Pose2d())
                .forward(45)
                .build();
        Trajectory rightTrajectorypt3 = drive.trajectoryBuilder(new Pose2d())
                .back(18)
                .build();
        Trajectory rightTrajectorypt4 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(36)
                .build();

        Trajectory leftTrajectoryScoreHpt1 = drive.trajectoryBuilder(new Pose2d(-36, -65.5), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 0), Math.toRadians(0))
                .build();
        Trajectory leftTrajectoryScoreHpt2 = drive.trajectoryBuilder(leftTrajectoryScoreHpt1.end())
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                .build();
        Trajectory leftTrajectoryScoreHpt3 = drive.trajectoryBuilder(leftTrajectoryScoreHpt2.end())
                .strafeLeft(24)
                .build();

        Trajectory rightTrajectoryScoreHpt1 = drive.trajectoryBuilder(new Pose2d(36, -65.5), Math.toRadians(90))
                .splineTo(new Vector2d(36, 0), Math.toRadians(180))
                .build();
        Trajectory rightTrajectoryScoreHpt2 = drive.trajectoryBuilder(leftTrajectoryScoreHpt1.end())
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                .build();
        Trajectory rightTrajectoryScoreHpt3 = drive.trajectoryBuilder(leftTrajectoryScoreHpt2.end())
                .strafeRight(24)
                .build();




        Trajectory toHigh = drive.trajectoryBuilder(new Pose2d(-36, -65.5), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 0), Math.toRadians(0))
                .build();

        Trajectory parkingPosition = drive.trajectoryBuilder(toHigh.end())
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                .build();

//        public void goTo(int ticks) {
//            targetPos = ticks;
//            height = viperSlide.getCurrentPosition();
//
//            viperSlide.setTargetPosition(targetPos);
//
//            if (height < targetPos) {
//                viperSlide.setPower(1);
//            } else if (height > targetPos) {
//                viperSlide.setPower(-1);
//            } else if (height == targetPos) {
//                viperSlide.setPower(0);
//            }
//    }

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        claw.setPosition(1);
//        viperSlide.goTo(100);
        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
//            drive.followTrajectory(leftTrajectoryScoreHpt1);
//            drive.followTrajectory(leftTrajectoryScoreHpt2);
//            drive.followTrajectory(leftTrajectoryScoreHpt3);
            drive.followTrajectory(leftTrajectorypt0);
            drive.followTrajectory(leftTrajectorypt1);
            drive.followTrajectory(leftTrajectorypt2);
            drive.followTrajectory(leftTrajectorypt3);
            drive.followTrajectory(leftTrajectorypt4);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(rightTrajectorypt0);
            drive.followTrajectory(rightTrajectorypt1);
            drive.followTrajectory(middleTrajectorypt1);
        } else {
            drive.followTrajectory(rightTrajectorypt0);
            drive.followTrajectory(rightTrajectorypt1);
            drive.followTrajectory(rightTrajectorypt2);
            drive.followTrajectory(rightTrajectorypt3);
            drive.followTrajectory(rightTrajectorypt4);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}