package org.firstinspires.ftc.teamcode.Auto.newAuto;

import org.firstinspires.ftc.teamcode.AprilTagItems.AprilTag;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Autonomous(name = "BlueAuto Far")
public class BlueAutoFar extends LinearOpMode {

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intake;
    public static double INTAKE_VEL = 1000;
    private Servo hoodL;
    private Servo hoodR;

    private void shootRoutine(double rpm, double hoodPos, boolean muck) {
        if (!muck) {
            intake.setVelocity(-INTAKE_VEL);
            sleep(80);
            intake.setVelocity(0);
        } else {
            shooterLeft.setVelocity(700);
            shooterRight.setVelocity(700);
            sleep(400);
            intake.setVelocity(INTAKE_VEL);
            sleep(300);
            intake.setVelocity(0);
        }
        hoodL.setPosition(hoodPos);
        hoodR.setPosition(hoodPos);
        shooterLeft.setVelocity(rpm);
        shooterRight.setVelocity(rpm);
        sleep(2000);
        intake.setVelocity(INTAKE_VEL);
        sleep(3000);
        intake.setVelocity(0);
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    private void collectRow(int row, MecanumDrive drive) {
        double rowY = -35;

        // Turn to face +Y
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(90))
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(90))
                        .build()
        );

        // Go to row, maintaining 90 degree heading
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToYLinearHeading(rowY, Math.toRadians(90))
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToYLinearHeading(rowY, Math.toRadians(90))
                        .build()
        );

        // Turn to face 180 degrees to align with balls
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(180))
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(180))
                        .build()
        );

        // Drive forward into row slowly with intake running
        intake.setVelocity(INTAKE_VEL);
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(-40,
                                new TranslationalVelConstraint(10.0),
                                new ProfileAccelConstraint(-20, 20))
                        .build()
        );
        intake.setVelocity(0);

        // Back straight out to x=-24
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(-24)
                        .build()
        );

        // Turn to face goal
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(115))
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(115))
                        .build()
        );


        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToY(-52)
                        .build()
        );
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-12, -60, Math.toRadians(114));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        AprilTag vision = new AprilTag(hardwareMap, telemetry);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "LS");
        shooterRight = hardwareMap.get(DcMotorEx.class, "RS");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodL = hardwareMap.get(Servo.class, "hoodL");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL.setDirection(Servo.Direction.REVERSE);
        hoodR.setDirection(Servo.Direction.FORWARD);
        hoodL.setPosition(0.0);
        hoodR.setPosition(0.0);

        telemetry.addLine("Blue Auto Ready");
        telemetry.update();
        waitForStart();

        // Step 1 - shoot
        shootRoutine(1750, 0.65, false);

        // Step 6 - collect motif row
        collectRow(23, drive);

        // Step 7 - shoot
        shootRoutine(1750, 0.65, false);

        // Step 10 - get off of line
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-20, -40), Math.toRadians(90))
                        .build()
        );
    }
}