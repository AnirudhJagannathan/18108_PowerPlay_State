package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.StageSwitchingPipeline;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0.3);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0.3);

    public static double LATERAL_MULTIPLIER = 1.4920121658;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public double drive1 = 0.0;
    public double drive2 = 0.0;

    private LinearOpMode opMode;

    public static final double AMAX_POS = 0.50;     // Maximum rotational position ---- Phil Claw: 1.4; GoBilda Claw: 1.4
    public static final double AMIN_POS = 0.25;     // Minimum rotational position ---- Phil Claw: 0.7; GoBilda Claw: 0.61

    public static final double CLAW_OPEN_SETTING = AMAX_POS;
    public static final double CLAW_CLOSED_SETTING = AMIN_POS;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront, slideLeft, slideRight, turret;
    private Servo Claw;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(LinearOpMode opMode, HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.opMode = opMode;

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        Claw = hardwareMap.get(Servo.class, "Claw");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void stop() {
        if (leftFront != null && rightFront != null && leftRear != null && rightRear != null) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            this.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    /* NON RR METHODS */

    public void moveSlidesAndTurret(double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) -slideHeight);
        turret.setTargetPosition(turretPos);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* while (slideLeft.getCurrentPosition() != slideHeight && turret.getCurrentPosition() != turretPos) {

        }

         */

        // move slides
        if (slideLeft.getCurrentPosition() < slideHeight) {
            slideLeft.setPower(slidePower);
            slideRight.setPower(-slidePower);
        }
        else if (slideLeft.getCurrentPosition() > slideHeight) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(slidePower);
        }

        //move turret
        if (turret.getCurrentPosition() > turretPos)
            turret.setPower(turretSpeed);
        else if (turret.getCurrentPosition() < turretPos)
            turret.setPower(turretSpeed);
    }


    public void strafeJunction(boolean leftOrRight,
                                  double maxDriveSpeed, StageSwitchingPipeline pipeline) { //True = left, False = right

        int valMid;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            opMode.sleep(500);

            drive1 = 0.0;
            if (leftOrRight){
                drive2 = maxDriveSpeed;
            }
            else {
                drive2 = -maxDriveSpeed;
            }
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftRear.setPower(rightPower);
            rightRear.setPower(leftPower);

            while (true) {
                valMid = pipeline.getValMid();

                leftFront.setPower(rightPower);
                rightFront.setPower(leftPower);
                leftRear.setPower(leftPower);
                rightRear.setPower(rightPower);

                if (valMid > 100) {
                    stop();
                    break;
                }

                opMode.telemetry.addData("valMid",  pipeline.getValMid());
                opMode.telemetry.addData("Height", pipeline.getRows());
                opMode.telemetry.addData("Width", pipeline.getCols());

                opMode.telemetry.update();
            }
        }
    }

    public void strafeJunctionRoadrunner(StageSwitchingPipeline pipeline, Trajectory traj) { //True = left, False = right

        int localValMid;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            opMode.sleep(500);

            followTrajectoryAsync(traj);

            while (true) {
                localValMid = pipeline.getValMid();
                opMode.telemetry.addData("valMid",  pipeline.getValMid());
                opMode.telemetry.addData("Height", pipeline.getRows());
                opMode.telemetry.addData("Width", pipeline.getCols());
                opMode.telemetry.update();

                if (localValMid > 100) {
                    breakFollowing();
                    setDrivePower(new Pose2d());
                    break;
                }

                opMode.telemetry.addData("valMid",  pipeline.getValMid());
                opMode.telemetry.addData("Height", pipeline.getRows());
                opMode.telemetry.addData("Width", pipeline.getCols());

                opMode.telemetry.update();
            }
            update();
        }
    }

    public void openClaw() {
        Claw.setPosition(CLAW_OPEN_SETTING);
    }

    public void closeClaw() {
        Claw.setPosition(CLAW_CLOSED_SETTING);
    }

    public void moveSlides() {
        double slidePower = Range.clip(opMode.gamepad2.right_stick_y, -1.0, 1.0);

        //Limit how high the slides can go

        if(((slideLeft.getCurrentPosition()) > 5000 && slidePower < 0) ||
                ((slideLeft.getCurrentPosition()) < -150 && slidePower > 0)) {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
        else if (slidePower > 0) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(slidePower);
        }
        else {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(slidePower);
        }

        opMode.telemetry.addData("slidePower", slidePower);
        opMode.telemetry.addData("slideLeftHeight", slideLeft.getCurrentPosition());
        opMode.telemetry.addData("slideRightHeight", slideRight.getCurrentPosition());
    }

    public void moveTurret() throws InterruptedException {
        double turretPower = Range.clip(opMode.gamepad2.left_stick_x, -1.0, 1.0);
        double turretPos = turret.getCurrentPosition();
        opMode.telemetry.addData("turretPos", turretPos);

        if ((turretPos >= 850 && turretPower < -0.01) || (turretPos <= -850 && turretPower > 0.01)) {
            turret.setPower(0);
        }
        else {
            turret.setPower(-0.45 * turretPower);
        }
    }

    public void resetSlides() {
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void mecanumDriving() {
        opMode.resetRuntime();
        double drive = opMode.gamepad1.left_stick_y;
        double strafe = opMode.gamepad1.right_stick_x;
        double turn = opMode.gamepad1.left_stick_x;
        double v1, v2, v3, v4;

        if (opMode.gamepad1.right_bumper) {
            v1 = Range.clip(-drive + strafe + turn, -0.2, 0.2);
            v2 = Range.clip(-drive - strafe - turn, -0.2, 0.2);
            v3 = Range.clip(-drive + strafe - turn, -0.2, 0.2);
            v4 = Range.clip(-drive - strafe + turn, -0.2, 0.2);
        }

        else {
            v1 = Range.clip(-drive + strafe + turn, -0.9, 0.9);
            v2 = Range.clip(-drive - strafe - turn, -0.9, 0.9);
            v3 = Range.clip(-drive + strafe - turn, -0.9, 0.9);
            v4 = Range.clip(-drive - strafe + turn, -0.9, 0.9);
        }
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }
}
