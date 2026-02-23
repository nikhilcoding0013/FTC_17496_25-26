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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;

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
    private void shootRoutine(double rpm, double hoodPos, boolean muck) {
        if(!muck){
            // Brief outtake
            intake.setVelocity(-INTAKE_VEL);
            sleep(200);
            intake.setVelocity(0);
        }
        else{
            // get rid of ball 1
            shooterLeft.setVelocity(700);
            shooterRight.setVelocity(700);
            sleep(400);
            intake.setVelocity(INTAKE_VEL);
            sleep(300);
            intake.setVelocity(0);
        }

        // Angle hood
        hoodL.setPosition(hoodPos);
        hoodR.setPosition(hoodPos);
        
        // Spool up launchers
        shooterLeft.setVelocity(rpm);
        shooterRight.setVelocity(rpm);
        sleep(2000);

        // Intake
        intake.setVelocity(INTAKE_VEL);
        sleep(3000);

        // Stop everything
        intake.setVelocity(0);
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    private void collectRow(int row, MecanumDrive drive) {
        switch (row) {
            case 21:
                Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-24, -35.67), Math.PI)
                        .build()
                );
                break;
            case 22:
                Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-24, -12.11), Math.PI)
                        .build()
                );
                break;
            case 23:
                Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(-24, 11.56), Math.PI)
                        .build()
                );
                break;
        }
    
        // Drive forward slowly with intake running
        Pose2d current = drive.localizer.getPose();
        intake.setVelocity(INTAKE_VEL);
        Actions.runBlocking(
            drive.actionBuilder(current)
                .lineToX(current.position.x - 34,
                    new TranslationalVelConstraint(5.0),
                    new ProfileAccelConstraint(-20, 20))
                .build()
        );
        intake.setVelocity(0);
    
        // Drive back at regular speed and go to (-18, 18) at 135 degrees
        current = drive.localizer.getPose();
        Actions.runBlocking(
            drive.actionBuilder(current)
                .splineTo(new Vector2d(current.position.x + 34, current.position.y), 0)
                .splineTo(new Vector2d(-18, 18), Math.toRadians(135))
                .build()
        );
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
                .splineTo(new Vector2d(
                    -49.84 + 7 * Math.cos(Math.toRadians(144.046 + 180)),
                    55.93 + 7 * Math.sin(Math.toRadians(144.046 + 180))
                ), Math.toRadians(144.046))
                .build()
        );

        // Step 2 - shoot routine
        shootRoutine(1160, 0.0, false);

        // Step 3 - drive to (-18, 35) at 64 degrees
        Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(-18, 35), Math.toRadians(64))
                .build()
        );

        // Step 4 - read motif tag
        int motifId = -1;
        for (int i = 0; i < 5 && motifId == -1; i++) {
            vision.update();
            motifId = vision.getMotifId();
            sleep(30);
        }
        // Precaution
        if (motifId == -1) motifId = 22;
        
        // Step 5 - collect corresponding row and go to shooting position
        collectRow(motifId, drive);

        // Step 6 - shoot routine
        shootRoutine(1365, 0.36, false);

        // Step 7 - collect above row
        int aboveRow = motifId + 1;
        if(aboveRow > 23)
            aboveRow = 21;
        collectRow(aboveRow, drive);

        // Step 8 - shoot routine but get rid of ball 1
        shootRoutine(1365, 0.36, true);

        // Step 9 - get off line
        Pose2d current = drive.localizer.getPose();
        Actions.runBlocking(
            drive.actionBuilder(current)
                .splineTo(new Vector2d(current.position.x + 34, current.position.y), 0)
                .splineTo(new Vector2d(-12, -24), Math.toRadians(135))
                .build()
        );
    }
}
