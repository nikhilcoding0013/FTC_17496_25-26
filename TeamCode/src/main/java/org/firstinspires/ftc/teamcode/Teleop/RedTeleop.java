package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PoseStorage;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Config
@TeleOp(name = "RedTeleop")
public class RedTeleop extends LinearOpMode {

    // DRIVE - field level so aimAtTarget can access it
    private MecanumDrive drive;

    // SHOOTER
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // TARGET POINT
    private static final double TARGET_X = 66.0;
    private static final double TARGET_Y =  66.0;
    private static final double CORNER_X = 72.0;
    private static final double CORNER_Y =  72.0;
    boolean autoAimActive = false;
    boolean lastY = false;

    // INTAKE
    private DcMotorEx intake;
    public static double INTAKE_VEL = 1000;

    // HOOD
    private Servo hoodL;
    private Servo hoodR;
    public double hoodPos = 0.0;

    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.65;

    // -------------------------------------------------------
    // Calculates distance from robot to (72, 72),
    // then sets shooter velocity and hood position accordingly.
    // -------------------------------------------------------
    private void aimAtTarget(Pose2d pose) {
        double dx = pose.position.x - CORNER_X;
        double dy = pose.position.y - CORNER_Y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Turn to face target using RoadRunner
        double angleToTarget = Math.atan2(TARGET_Y - pose.position.y, TARGET_X - pose.position.x);
        Actions.runBlocking(
                drive.actionBuilder(pose)
                        .turnTo(angleToTarget)
                        .build()
        );
        Pose2d adjustedPose = drive.localizer.getPose();
        double adjustedAngle = Math.atan2(TARGET_Y - adjustedPose.position.y, TARGET_X - adjustedPose.position.x);
        Actions.runBlocking(
                drive.actionBuilder(adjustedPose)
                        .turnTo(adjustedAngle)
                        .build()
        );

        // Brief outtake before spooling up
        intake.setVelocity(-INTAKE_VEL);
        sleep(200);
        intake.setVelocity(0);

        // Shooter velocity: linear fit to distance
        double targetVel = 5.1198 * distance + 1017.3849;
        shooterLeft.setVelocity(targetVel);
        shooterRight.setVelocity(targetVel);

        // Hood position: exponential fit to distance, clamped to [0.0, 0.65]
        double targetHood = 0.6508 + (-0.6686) * Math.pow(0.9747, distance - 31.0055);
        targetHood = Math.min(Math.max(targetHood, HOOD_MIN), HOOD_MAX);

        hoodL.setPosition(targetHood);
        hoodR.setPosition(targetHood);
        hoodPos = targetHood;

        telemetry.addData("Distance to Target", "%.2f in", distance);
        telemetry.addData("Auto Vel",            "%.1f",   targetVel);
        telemetry.addData("Auto Hood Pos",       "%.3f",   targetHood);
    }

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start at AutoCoordinate
        Pose2d startPose = PoseStorage.currentPose;
        drive = new MecanumDrive(hardwareMap, startPose);

        // SHOOTER
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "LS");
        shooterRight = hardwareMap.get(DcMotorEx.class, "RS");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // INTAKE
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // HOOD
        hoodL = hardwareMap.get(Servo.class, "hoodL");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL.setDirection(Servo.Direction.REVERSE);
        hoodR.setDirection(Servo.Direction.FORWARD);
        hoodL.setPosition(0.0);
        hoodR.setPosition(0.0);

        telemetry.addLine("Blue TeleOp Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            // Required every loop for odometry
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            // DRIVE
            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double turn    = -gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(forward, strafe),
                    turn
            ));

            // Toggle auto aim
            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                autoAimActive = !autoAimActive;

                if (autoAimActive) {
                    aimAtTarget(pose);
                    intake.setVelocity(INTAKE_VEL);
                } else {
                    shooterLeft.setVelocity(0);
                    shooterRight.setVelocity(0);
                    intake.setVelocity(0);
                }
            }
            lastY = currentY;

            // A button - close range preset (blocked during auto aim)
            if (gamepad1.a && !autoAimActive) {
                hoodL.setPosition(0.0);
                hoodR.setPosition(0.0);
                hoodPos = 0.0;
                shooterLeft.setVelocity(1175);
                shooterRight.setVelocity(1175);
            } else if (!autoAimActive) {
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
            }

            // INTAKE (blocked during auto aim)
            if (gamepad1.right_trigger > 0.2) {
                intake.setVelocity(INTAKE_VEL);
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setVelocity(-INTAKE_VEL);
            } else if (!autoAimActive) {
                intake.setVelocity(0);
            }

            // TELEMETRY
            telemetry.addData("X",             "%.1f", pose.position.x);
            telemetry.addData("Y",             "%.1f", pose.position.y);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Auto Aim",      autoAimActive ? "ON" : "OFF");
            telemetry.addData("Close Preset",  gamepad1.a ? "ON" : "OFF");
            telemetry.addData("Shooter Vel",   "%.1f", shooterLeft.getVelocity());
            telemetry.addData("Hood Pos",      "%.3f", hoodPos);
            telemetry.addData("Intake",        intake.getVelocity() > 0 ? "IN" : intake.getVelocity() < 0 ? "OUT" : "OFF");
            telemetry.update();
        }
    }
}
