package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "TeleOp2026FullShootingSequence", group = "Competition")
public class TeleOp2026FullShootingSequence extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor   intakeMotor, sortMotor;
    private Servo     hoodLeft, hoodRight, liftLeft, liftRight;
    private ColorSensor colorSensor;

    // ================= DASHBOARD SORTER VARIABLES =================
    public static double SORT_MOTOR_TICKS_PER_REV = 560.0;
    public static double SORT_GEAR_NUM  = 16.0;
    public static double SORT_GEAR_DEN  = 13.0;
    public static double SORT_DEGREES   = 173.0;
    public static double SORT_POWER     = 0.5;
    private double ticksPerOutputRev;
    private double ticksPerDegree;
    private int    SORT_MOVE_TICKS;

    // ================= DRIVE CONSTANTS =================
    private final double TICKS_PER_REV    = 28.0;
    private final double MAX_RPM          = 7000.0;
    private       double DRIVE_TICKS_PER_SEC;
    private final double DRIVE_DEADZONE   = 0.03;

    // ================= SHARED MOTOR CONSTANTS =================
    static final double COUNTS_PER_REV    = 1378.0;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;

    // ================= DASHBOARD =================
    private FtcDashboard       dashboard;
    private MultipleTelemetry  telemetryDashboard;
    private double             lastTime = 0.0;
    private double             hoodPosX = 0.8;

    // ================= LIFT STATE =================
    private boolean liftActive    = false;
    private double  liftStartTime = 0.0;

    // ================= INDEXER TRACKING =================
    //  indexOrder[slot] — color stored in each physical slot (0=purple,1=green,2=empty)
    //  Ignoring color for now, so shootOrder is always {0,1,2}
    private int[] indexOrder = new int[]{2, 2, 2};
    private int   indexerPos = 0;               // which physical slot is currently at the kicker

    // ================= SHOOTING STATE =================
    private boolean shootingActive = false;
    private boolean lastLB         = false;
    private int     shootState     = 0;
    private int     ballsLaunched  = 0;
    private int     ballsIn        = 0;
    private double  sorterTimer    = 0.0;
    private double  liftTimer      = 0.0;

    // Shoot order — ignored for color right now, just shoot slots 0 → 1 → 2
    private int[] shootOrder = new int[]{0, 1, 2};

    // ================= INTAKE STATE =================
    private boolean lastRB              = false;
    private boolean intakeSequenceActive = false;
    private int     intakeState         = 0;
    private double  intakeTimer         = 0.0;

    // ================= MANUAL SHIFT =================
    private boolean notShifting  = true;
    private double  shiftTimer   = 0.0;

    @Override
    public void runOpMode() {
        // ---- HARDWARE MAP ----
        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        launcherLeft  = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sortMotor   = hardwareMap.get(DcMotor.class, "sortMotor");

        hoodLeft  = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");
        liftLeft  = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // ---- MOTOR SETUP ----
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DRIVE_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

        dashboard          = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // ---- SORTER MATH ----
        ticksPerOutputRev = SORT_MOTOR_TICKS_PER_REV * (SORT_GEAR_NUM / SORT_GEAR_DEN);
        ticksPerDegree    = ticksPerOutputRev / 360.0;
        SORT_MOVE_TICKS   = (int) Math.round(SORT_DEGREES * ticksPerDegree);

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            handleDrive();
            shootingSequence();
            intakeSequence();
            handleLift();
            shiftIndexer();
            handleHoodManual();
            updateTelemetry();
        }
    }

    // ================= DRIVE =================
    private void handleDrive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.15;
        double rx =  gamepad1.right_stick_x;

        if (Math.abs(y)  < DRIVE_DEADZONE) y  = 0;
        if (Math.abs(x)  < DRIVE_DEADZONE) x  = 0;
        if (Math.abs(rx) < DRIVE_DEADZONE) rx = 0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1) { fl /= max; fr /= max; bl /= max; br /= max; }

        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);
    }

    // ================= LIFT =================
    private void handleLift() {
        boolean x = gamepad1.x;
        if (!liftActive && x) {
            liftActive    = true;
            liftStartTime = getRuntime();
            liftLeft.setPosition(0.35);
            liftRight.setPosition(0.65);
        }
        if (liftActive && getRuntime() - liftStartTime >= 1) {
            liftLeft.setPosition(0.99);
            liftRight.setPosition(0.01);
            liftActive = false;
        }
    }

    // ================= MANUAL SHIFT INDEXER =================
    private void shiftIndexer() {
        boolean a = gamepad1.a;
        if (a && notShifting) {
            notShifting = false;
            int moveTicks = (int)(5.0 * COUNTS_PER_DEGREE); // shift 5 degrees
            int target    = sortMotor.getCurrentPosition() + moveTicks;
            sortMotor.setTargetPosition(target);
            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortMotor.setPower(SORT_POWER);
            shiftTimer = getRuntime();
        }
        if (getRuntime() - shiftTimer >= 0.1) {
            notShifting = true;
        }
    }

    // ================= HOOD =================
    private void handleHoodManual() {
        double currentTime = getRuntime();
        double dt          = currentTime - lastTime;
        lastTime           = currentTime;
        double speed       = 0.05;

        if (gamepad1.dpad_up)   hoodPosX -= speed * dt;
        if (gamepad1.dpad_down) hoodPosX += speed * dt;

        hoodPosX = Math.max(0.01, Math.min(0.7, hoodPosX));
        hoodLeft.setPosition(hoodPosX);
        hoodRight.setPosition(1.0 - hoodPosX);
    }

    // ================= SHOOTING (ported from auto shootingAction()) =================
    //
    //  State 0 — Decide which slot to move to; command the sorter move.
    //  State 1 — Wait for sorter to finish moving.
    //  State 2 — Wait 0.35 s settle time, then kick.
    //  State 3 — Wait 0.75 s for kicker to complete, then retract and count.
    //  State 4 — Short pause after shot; loop back to 0 if balls remain, else go to 5.
    //  State 5 — Cleanup: stop flywheels, partial reset shift, mark done.
    //
    private void shootingSequence() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !lastLB && !shootingActive) {
            shootingActive = true;
            shootState     = 0;
            ballsLaunched  = 0;
        }
        lastLB = lb;

        if (!shootingActive) return;

        // Keep flywheels commanded every loop so the velocity PID holds speed
        double vel = (3000.0 / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        switch (shootState) {

            case 0:
                // Only enter logic when the sorter is not already busy
                if (!sortMotor.isBusy()) {
                    if (ballsIn <= 0) {
                        // No balls left — skip straight to cleanup
                        shootState = 5;
                        break;
                    }

                    int targetSlot  = shootOrder[ballsLaunched];          // slot we want at the kicker
                    int stepsNeeded = (targetSlot - indexerPos + 3) % 3;  // modular steps (0, 1, or 2)

                    if (stepsNeeded == 0) {
                        // Already at correct slot — go straight to settle wait
                        sorterTimer = getRuntime();
                        shootState  = 2;
                    } else {
                        int moveTicks = (int)(stepsNeeded * ((SORT_DEGREES / 2.0)) * COUNTS_PER_DEGREE);
                        int target    = sortMotor.getCurrentPosition() + moveTicks;
                        sortMotor.setTargetPosition(target);
                        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sortMotor.setPower(SORT_POWER);
                        indexerPos  = targetSlot;     // update our tracking immediately
                        sorterTimer = getRuntime();
                        shootState  = 1;              // wait for move to finish
                    }
                }
                break;

            case 1:
                // Wait for sorter motor to stop moving
                if (!sortMotor.isBusy()) {
                    sorterTimer = getRuntime();
                    shootState  = 2;
                }
                break;

            case 2:
                // Settle time after sorter stops, then extend kicker
                if (!sortMotor.isBusy() && getRuntime() - sorterTimer >= 0.35) {
                    liftLeft.setPosition(0.15);
                    liftRight.setPosition(0.85);
                    liftTimer  = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:
                // Wait for ball to exit, then retract kicker and record shot
                if (getRuntime() - liftTimer >= 1.00) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    liftTimer     = getRuntime();
                    ballsLaunched++;
                    ballsIn--;
                    shootState = 4;
                }
                break;

            case 4:
                // Short pause — decide whether to fire again or end
                if (ballsIn <= 0 || ballsLaunched >= 3) {
                    shootState = 5;
                } else if (getRuntime() - liftTimer >= 0.75) {
                    shootState = 0;
                }
                break;

            case 5:
                // Cleanup: stop flywheels, do the post-sequence half-step reset
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);

                int moveTicks = (int)((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                int target    = sortMotor.getCurrentPosition() + moveTicks;
                sortMotor.setTargetPosition(target);
                sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sortMotor.setPower(SORT_POWER);
                indexerPos    = (indexerPos + 2) % 3;

                // Reset for next round
                shootingActive = false;
                ballsLaunched  = 0;
                indexOrder     = new int[]{2, 2, 2};
                break;
        }
    }

    // ================= INTAKE =================
    private void intakeSequence() {
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB && !intakeSequenceActive) {
            intakeSequenceActive = true;
            intakeState          = 0;
            ballsIn              = 0;
        }
        lastRB = rb;

        if (!intakeSequenceActive) return;

        switch (intakeState) {

            case 0:
                intakeMotor.setPower(-0.8);
                boolean greenDetected  = (colorSensor.green() > 100);
                boolean purpleDetected = (colorSensor.red() > 55 && colorSensor.blue() > 80);

                if (greenDetected || purpleDetected) {
                    intakeTimer = getRuntime();
                    if (ballsIn >= 0 && ballsIn < 3) {
                        if (greenDetected)  indexOrder[ballsIn] = 1;
                        if (purpleDetected) indexOrder[ballsIn] = 0;
                    }
                    intakeState = 1;
                }

                // Let shooting sequence interrupt intake (e.g. 2-ball mid-sequence)
                if (shootingActive) {
                    intakeSequenceActive = false;
                    intakeMotor.setPower(0);
                }
                break;

            case 1:
                if (getRuntime() - intakeTimer >= 0.75 && !sortMotor.isBusy()) {
                    int moveTicks = (int)((SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                    int target    = sortMotor.getCurrentPosition() + moveTicks;
                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);
                    indexerPos = (indexerPos + 1) % 3;
                    ballsIn++;
                    intakeState = 2;
                }
                break;

            case 2:
                if (!sortMotor.isBusy()) {
                    if (ballsIn >= 3) {
                        intakeState = 3;   // Hand off to cleanup state — NO sleep() here
                    } else {
                        intakeState = 0;   // Keep intaking
                    }
                }
                break;

            case 3:
                // Final quarter-step shift to bring the indexer to launch position
                if (!sortMotor.isBusy()) {
                    int moveTicks = (int)((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                    int target    = sortMotor.getCurrentPosition() + moveTicks;
                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);
                    intakeState = 4;
                }
                break;

            case 4:
                // Wait for that final shift to finish, then cleanly exit RUN_TO_POSITION
                if (!sortMotor.isBusy()) {
                    // Exit RUN_TO_POSITION so isBusy() returns false for the shooting sequence
                    sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    sortMotor.setPower(0);
                    intakeMotor.setPower(0);
                    indexerPos           = (indexerPos + 2) % 3;
                    intakeSequenceActive = false;
                }
                break;
        }
    }

    // ================= TELEMETRY =================
    private void updateTelemetry() {
        telemetryDashboard.addData("Shoot State",    shootState);
        telemetryDashboard.addData("Sorter position", sortMotor.getCurrentPosition());
        telemetryDashboard.addData("Sorter busy",    sortMotor.isBusy());
        telemetryDashboard.addData("Red",            colorSensor.red());
        telemetryDashboard.addData("Green",          colorSensor.green());
        telemetryDashboard.addData("Blue",           colorSensor.blue());
        telemetryDashboard.addData("Balls in",       ballsIn);
        telemetryDashboard.addData("Balls launched", ballsLaunched);
        telemetryDashboard.addData("Ball Order",     indexOrder);
        telemetryDashboard.addData("Indexer pos",    indexerPos);
        telemetryDashboard.addData("Shoot order",    shootOrder);
        telemetryDashboard.update();
    }
}