package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Config
@Autonomous(name = "BlueAutoClose")
public class BlueAutoNew extends LinearOpMode {

    // VISION
    private AprilTag vision;
    
    // SHOOTER
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // INTAKE
    private DcMotorEx intake;
    public static double INTAKE_VEL = 1000;

    // HOOD
    private Servo hoodL;
    private Servo hoodR;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.65;

    // -------------------------------------------------------
    // Outtake briefly, run launchers at 1160, wait 2s,
    // intake for 3s, then stop everything
    // -------------------------------------------------------
    private void shootRoutine() {
        // Brief outtake
        intake.setVelocity(-INTAKE_VEL);
        sleep(200);
        intake.setVelocity(0);

        // Spool up launchers
        shooterLeft.setVelocity(1160);
        shooterRight.setVelocity(1160);
        sleep(2000);

        // Intake
        intake.setVelocity(INTAKE_VEL);
        sleep(3000);

        // Stop everything
        intake.setVelocity(0);
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-49.84, 55.93, Math.toRadians(144.046));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        vision = new AprilTag(hardwareMap, telemetry);
        
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

        telemetry.addLine("Blue Auto Ready");
        telemetry.update();
        waitForStart();

        // Step 1 - drive backwards 7 inches
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                .lineToX(-49.84 + 7 * Math.cos(Math.toRadians(144.046 + 180)))
                .build()
        );

        // Step 2 - shoot routine
        shootRoutine();

        // Step 3 - drive to (-18, 35) at 64 degrees
        Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(-18, 35), Math.toRadians(64))
                .build()
        );

        // Step 4 - read motif tag
        vision.update();
        int motifId = vision.getMotifId();
        telemetry.addData("Motif ID", motifId);
        telemetry.update();
    }
}
