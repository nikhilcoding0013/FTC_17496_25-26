package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "AngleTesting", group = "Testing")
public class AngleAndRPMTesting extends OpMode {

    double LAUNCHER_RPM = 1150;
    // DASHBOARD TUNABLE
    public static double INTAKE_RPM = 1000;

    // =============================
    // Servos
    // =============================
    Servo leftServo;
    Servo rightServo;

    final double SPEED_SCALE = 0.02;
    // Find MIN_POS and MAX_POS
    final double MIN_POS = 0.0;
    final double MAX_POS = 0.65;

    double leftPos;
    double rightPos;

    // =============================
    // Motors
    // =============================
    DcMotorEx IntakeEx;
    DcMotorEx LSX;
    DcMotorEx RSX;

    boolean launcherOn = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;

    @Override
    public void init() {

        // ===== Servos =====
        leftServo  = hardwareMap.get(Servo.class, "hoodR");
        rightServo = hardwareMap.get(Servo.class, "hoodL");
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftPos = 0.0;
        rightPos = 0.0;

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);

        // ===== Motors =====
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");

        //IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.REVERSE);
        LSX.setDirection(DcMotorEx.Direction.FORWARD);

        IntakeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        // =============================
        // SERVO CONTROL (Triggers)
        // =============================
        double movement = (gamepad1.right_trigger - gamepad1.left_trigger) * SPEED_SCALE;

        leftPos  = Math.min(Math.max(leftPos + movement, MIN_POS), MAX_POS);
        rightPos = Math.min(Math.max(rightPos + movement, MIN_POS), MAX_POS);

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);

        // =============================
        // ADJUST LAUNCHER RPM (B / X)
        // =============================
        if (gamepad1.b && !lastB) {
            LAUNCHER_RPM += 25;
        }

        if (gamepad1.x && !lastX) {
            LAUNCHER_RPM -= 25;
        }

        lastB = gamepad1.b;
        lastX = gamepad1.x;

        // =============================
        // LAUNCHER TOGGLE (A button)
        // =============================
        if (gamepad1.a && !lastA) {
            launcherOn = !launcherOn;
        }
        lastA = gamepad1.a;

        if (launcherOn) {
            LSX.setVelocity(LAUNCHER_RPM);
            RSX.setVelocity(LAUNCHER_RPM);
        } else {
            LSX.setVelocity(0);
            RSX.setVelocity(0);
        }

        // =============================
        // INTAKE (HOLD LEFT BUMPER)
        // =============================
        if (gamepad1.left_bumper) {
            IntakeEx.setVelocity(INTAKE_RPM);
        } else {
            IntakeEx.setVelocity(0);
        }

        // =============================
        // TELEMETRY
        // =============================
        telemetry.addData("Launcher RPM", LAUNCHER_RPM);
        telemetry.addData("LS Actual", "%.0f", LSX.getVelocity());
        telemetry.addData("RS Actual", "%.0f", RSX.getVelocity());
        telemetry.addData("Intake RPM (Dash)", INTAKE_RPM);
        telemetry.addData("Launcher ON", launcherOn);
        telemetry.addData("Servo Pos", "%.2f", leftPos);
        telemetry.addData("Angle", "%.2f", (65 - leftPos * 28.6));
        telemetry.update();
    }
}
