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
import org.firstinspires.ftc.teamcode.PoseStorage;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Autonomous(name = "BlueAuto Close")
public class BlueAutoClose extends LinearOpMode {

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intake;
    public static double INTAKE_VEL = 1000;
    private Servo hoodL;
    private Servo hoodR;

    private static final double SHOOTING_X = -16;
    private static final double SHOOTING_Y = 8;

    private void shootRoutine(double rpm, double hoodPos, boolean muck, double intake_vel) {
        if (!muck) {
            intake.setVelocity(-intake_vel);
            sleep(200);
            intake.setVelocity(0);
        } else {
            shooterLeft.setVelocity(700);
            shooterRight.setVelocity(700);
            sleep(400);
            intake.setVelocity(intake_vel);
            sleep(300);
            intake.setVelocity(0);
        }
        hoodL.setPosition(hoodPos);
        hoodR.setPosition(hoodPos);
        shooterLeft.setVelocity(rpm);
        shooterRight.setVelocity(rpm);
        sleep(2000);
        intake.setVelocity(intake_vel);
        sleep(3000);
        intake.setVelocity(0);
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    private double angleToGoal(Pose2d current) {
        return Math.atan2(72 - current.position.y, -72 - current.position.x);
    }

    private void collectRow(int row, MecanumDrive drive) {
        double rowY;
        double splineInTangent;
        double splineOutTangent;
        switch (row) {
            case 21: rowY = -35.67; splineInTangent = 300; splineOutTangent = 310; break;
            case 22: rowY = -12.11; splineInTangent = 200; splineOutTangent = 350; break;
            default: rowY =  11.56; splineInTangent = 300; splineOutTangent = 310; break;
        }

        // Backwards spline to lineup position
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, rowY), Math.toRadians(splineInTangent))
                        .build()
        );

        // Adjust Y
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToY(rowY)
                        .build()
        );

        // Turn to face 180, readjust
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

        // Drive forward into row slowly with intake
        intake.setVelocity(1500);
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(-42,
                                new TranslationalVelConstraint(9.5),
                                new ProfileAccelConstraint(-20, 20))
                        .build()
        );
        intake.setVelocity(0);

        // Backwards spline back to shooting position
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .setReversed(true)
                        .splineTo(new Vector2d(SHOOTING_X, SHOOTING_Y), Math.toRadians(splineOutTangent))
                        .build()
        );

        // Nudge for valid prior path
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToY(SHOOTING_Y - 0.01)
                        .build()
        );

        // Turn to face (-72, 72), readjust
        double angle = angleToGoal(drive.localizer.getPose());
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(angle)
                        .build()
        );
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-44.6, 49, Math.toRadians(140.5));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

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

        telemetry.addLine("Blue Auto Close Ready");
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
        shootRoutine(1185, 0, false, 800);

        // Step 3 - collect row 23
        collectRow(23, drive);

        // Step 4 - shoot
        shootRoutine(1425, 0.39, false, INTAKE_VEL);

        // Step 5 - collect row 22
        //collectRow(22, drive);

        // Step 6 - shoot
        //shootRoutine(1425, 0.39, false, INTAKE_VEL);

        // Step 7 - get off of line
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-40, 10), Math.toRadians(200))
                        .build()
        );
        PoseStorage.currentPose = drive.localizer.getPose();
    }
}