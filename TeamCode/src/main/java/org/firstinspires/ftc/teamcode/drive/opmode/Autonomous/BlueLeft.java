package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class BlueLeft extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this, hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Trajectory test1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(48, -6, Math.toRadians(0)))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16.5, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(57, 3.75, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence traj2_5 = drive.trajectorySequenceBuilder(traj2.end())
                .back(9.5)
                .lineToLinearHeading(new Pose2d(55, 6, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(0.75 * DriveConstants.MAX_VEL, 0.6 * DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.75 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2_5.end())
                .lineToLinearHeading(new Pose2d(58, 20.9, Math.toRadians(86)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(56, 9, Math.toRadians(86)),
                        SampleMecanumDrive.getVelocityConstraint(0.75 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(57, 21, Math.toRadians(86)),
                        SampleMecanumDrive.getVelocityConstraint(0.45 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(55.5, -13.5, Math.toRadians(86)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(57.5, 22, Math.toRadians(87)),
                        SampleMecanumDrive.getVelocityConstraint(0.75 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d(57, -13, Math.toRadians(88)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(56.5, 20.9, Math.toRadians(84)),
                        SampleMecanumDrive.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToLinearHeading(new Pose2d(55.5, -16.75, Math.toRadians(83)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .strafeLeft(4)
                .build();

        Trajectory left = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(57, 22, Math.toRadians(90)))
                .build();

        Trajectory middle = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(57, -6, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory right = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(53, -27, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory leftTest = drive.trajectoryBuilder(test1.end())
                .lineToLinearHeading(new Pose2d(48, 19, Math.toRadians(90)))
                .build();

        Trajectory middleTest = drive.trajectoryBuilder(test1.end())
                .lineToLinearHeading(new Pose2d(48, -3, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        Trajectory rightTest = drive.trajectoryBuilder(test1.end())
                .lineToLinearHeading(new Pose2d(48, -22, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                .build();

        while (opModeInInit()) {
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

                    if (tagFound)
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
        }

        waitForStart();

        if(isStopRequested()) return;

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

        /* Actually do something useful */
        if(tagOfInterest == null) {

        }

        else if (tagOfInterest != null) {
            drive.resetTurret();
            drive.resetSlides();
            drive.closeClaw();
            sleep(500);

            drive.followTrajectory(test1);

            /* drive.moveSlidesAndTurret(2000, 1.0, -440, 0.3);
            sleep(300);
            drive.followTrajectory(traj1);
            drive.openClaw();
            sleep(300);
            //Move to first low junction

            drive.moveSlidesAndTurret(800, 0.4, 0, 0.4);
            drive.followTrajectory(traj2);
            sleep(350);
            drive.followTrajectorySequence(traj2_5);
            //Move forward to align with cone stack

            drive.followTrajectory(traj3);
            drive.closeClaw();
            sleep(250);
            //Move to the cone stack

            drive.moveSlidesAndTurret(1200, 0.7, 0, 0.3);
            sleep(500);
            drive.moveSlidesAndTurret(2150, 0.7, 420, 0.3);
            drive.followTrajectory(traj4);
            drive.openClaw();
            sleep(350);
            //Align with second low

            drive.moveSlidesAndTurret(750, 0.9, 0, 0.75);
            drive.followTrajectory(traj5);
            drive.closeClaw();
            sleep(250);
            //Pick up a second cone from the stack

            drive.moveSlidesAndTurret(1200, 0.7, 0, 0.3);
            sleep(500);
            drive.moveSlidesAndTurret(3750, 0.8, 440, 0.3);
            drive.followTrajectory(traj6);
            drive.openClaw();
            sleep(350);
            //Deposit a cone to the medium junction

            drive.moveSlidesAndTurret(600, 1.0, 0, 0.35);
            drive.followTrajectory(traj7);
            sleep(350);
            drive.closeClaw();
            sleep(250);
            //Pick up a 3rd cone from the stack

            drive.moveSlidesAndTurret(1200, 0.7, 0, 0.3);
            sleep(500);
            drive.moveSlidesAndTurret(5150, 1.0, -440, 0.3);
            drive.followTrajectory(traj8);
            sleep(500);
            drive.openClaw();
            sleep(350);
            //Deposit a cone to the high junctions

            drive.moveSlidesAndTurret(400, 1.0, 0, 0.3);
            drive.followTrajectory(traj9);
            sleep(800);
            drive.closeClaw();
            sleep(250);
            //Pick up a 4th cone from the stack

            drive.moveSlidesAndTurret(1200, 0.7, 0, 0.3);
            sleep(500);
            drive.moveSlidesAndTurret(5200, 1.0, -440, 0.3);
            drive.followTrajectory(traj10);
            sleep(500);
            drive.openClaw();
            sleep(350);
            //Deposit a 2nd cone to the high junction

            drive.moveSlidesAndTurret(0, 1.0, 0, 0.4);
            drive.followTrajectory(traj11);

             */



            if(tagOfInterest.id == LEFT) {
                drive.followTrajectory(leftTest);
            }

            else if(tagOfInterest.id == MIDDLE) {
                drive.followTrajectory(middleTest);
            }

            else if(tagOfInterest.id == RIGHT) {
                drive.followTrajectory(rightTest);
            }

            drive.moveSlidesAndTurret(0, 0.8, 0, 0.4);
        }
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
