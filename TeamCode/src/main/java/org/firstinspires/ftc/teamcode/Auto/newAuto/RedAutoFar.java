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

@Autonomous(name = "RedAuto Far")
public class RedAutoFar extends LinearOpMode {

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intake;
    public static double INTAKE_VEL = 800;
    private Servo hoodL;
    private Servo hoodR;

    private void shootRoutine(double rpm, double hoodPos, boolean muck) {
        if (!muck) {
            intake.setVelocity(-INTAKE_VEL);
            sleep(250);
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
        sleep(3750);
        intake.setVelocity(0);
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    private double angleToGoal(Pose2d current) {
        return Math.atan2(72 - current.position.y, 72 - current.position.x);
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(16, -63, Math.toRadians(90));
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

        telemetry.addLine("Blue Auto Far Ready");
        telemetry.update();
        waitForStart();

        // Step 1 - move in +Y to (16, -55)
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToY(-55)
                        .build()
        );

        // Step 2 - turn to face (72, 72), readjust
        double angle1 = angleToGoal(drive.localizer.getPose());
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(angle1)
                        .build()
        );
        angle1 = angleToGoal(drive.localizer.getPose());
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(angle1)
                        .build()
        );

        // Step 3 - shoot
        shootRoutine(1765, 0.65, false);

        // Step 4 - drive forward diagonally at angle1 to (24, -35)
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(24, -35), angle1)
                        .lineToY(-35)
                        .build()
        );

        // Step 5 - turn to heading 0, readjust
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(0))
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(0))
                        .build()
        );

        // Step 6 - drive in X to x=53 with intake
        intake.setVelocity(1500);
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(53,
                                new TranslationalVelConstraint(9.5),
                                new ProfileAccelConstraint(-20, 20))
                        .build()
        );
        intake.setVelocity(0);

        // Step 7 - backwards spline
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .setReversed(true)
                        .splineTo(new Vector2d(30, -57), Math.toRadians(-115))
                        .build()
        );

        // Step 8 - small move so turnTo has valid prior path
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToY(-57)
                        .build()
        );

        // Step 9 - turn to face (72, 72), readjust
        double angle2 = angleToGoal(drive.localizer.getPose());
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(angle2)
                        .build()
        );
        angle2 = angleToGoal(drive.localizer.getPose());
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(angle2)
                        .build()
        );

        // Step 10 - shoot
        shootRoutine(1775, 0.65, false);

        // Step 11 - drive forward diagonally at angle2 to (20, -45)
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(20, -45), angle2)
                        .build()
        );
        PoseStorage.currentPose = drive.localizer.getPose();
    }
}