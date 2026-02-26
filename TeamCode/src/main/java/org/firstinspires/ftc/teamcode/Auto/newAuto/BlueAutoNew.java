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

@Autonomous(name = "BlueAuto NEW")
public class BlueAutoNew extends LinearOpMode {

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intake;
    public static double INTAKE_VEL = 1000;
    private Servo hoodL;
    private Servo hoodR;

    private void shootRoutine(double rpm, double hoodPos, boolean muck) {
        if (!muck) {
            intake.setVelocity(-INTAKE_VEL);
            sleep(200);
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
        double rowY;
        switch (row) {
            case 21: rowY = -35.67; break;
            case 22: rowY = -12.11; break;
            default: rowY =  11.56; break;
        }

        // Turn to face +Y
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(90))
                        .build()
        );

        // Reverse in Y to the row, maintaining 90 degree heading
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

        // Drive forward into row slowly with intake running
        intake.setVelocity(INTAKE_VEL);
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(-48,
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

        // Turn to face +Y
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(90))
                        .build()
        );

        // Spline forward to shooting position, heading aligned to path
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-24, 24), Math.toRadians(90))
                        .build()
        );

        // Turn to shooting angle
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(135))
                        .build()
        );
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-48, 54, Math.toRadians(144.046));
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

        // Step 1 - reverse 7 inches along starting heading
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToXLinearHeading(
                                startPose.position.x + 7 * Math.cos(Math.toRadians(144.046 + 180)),
                                Math.toRadians(144.046)
                        )
                        .build()
        );

        // Step 2 - shoot
        shootRoutine(1170, 0.0, false);

        // Step 3 - reverse to x=-24 maintaining heading
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToXLinearHeading(-24, Math.toRadians(144.046))
                        .build()
        );

        // Step 4 - turn inplace to face motif at 60 degrees
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(60))
                        .build()
        );

        // Step 5 - read motif tag
        int motifId = -1;
        for (int i = 0; i < 5 && motifId == -1; i++) {
            vision.update();
            motifId = vision.getMotifId();
            sleep(30);
        }
        if (motifId == -1) motifId = 22;

        // Step 6 - collect motif row
        collectRow(motifId, drive);

        // Step 7 - shoot
        shootRoutine(1350, 0.38, false);

        // Step 8 - collect above row
        int aboveRow = motifId + 1;
        if (aboveRow > 23) aboveRow = 21;
        collectRow(aboveRow, drive);

        // Step 9 - shoot with muck
        shootRoutine(1350, 0.38, true);

        // Step 10 - get off of line
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-40, -15), Math.toRadians(270))
                        .build()
        );
    }
}