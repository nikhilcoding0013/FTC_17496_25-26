package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "ArmManipulation")
public class ArmManipulation extends LinearOpMode {

    // SERVOS
    private Servo servo0;
    private Servo servo1;
    // MOTORS
    private DcMotorEx motor0;
    private DcMotorEx motor1;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // SERVOS
        servo0 = hardwareMap.get(Servo.class, "hoodR");
        servo0.setDirection(Servo.Direction.FORWARD);
        servo1 = hardwareMap.get(Servo.class, "hoodL");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo0.setPosition(0.5);
        servo1.setPosition(0.5);

        // MOTORS
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("ArmManipulation Ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double totalPos = 1.0; // servo0 + servo1, bounded [0.5, 1.5]
        double diffPos  = 0.0; // servo0 - servo1, bounded [-0.5, 0.5]

        while (opModeIsActive() && !isStopRequested()) {

            final double TOTAL_MIN = 0.5;
            final double TOTAL_MAX = 1.5;
            final double DIFF_MIN = -0.5;
            final double DIFF_MAX =  0.5;
            final double STEP = 0.015;
            final double SPIN_POWER = 0.3;
            final double LIFT_POWER = 0.1;

            // Servo Module Control
            // LEFT STICK Y → both servos same direction
            totalPos += -gamepad1.left_stick_y * STEP;
            // RIGHT STICK X → servos opposite directions
            diffPos += gamepad1.right_stick_x * STEP;
            // CLAMP
            totalPos = Math.min(Math.max(totalPos, TOTAL_MIN), TOTAL_MAX);
            diffPos  = Math.min(Math.max(diffPos,  DIFF_MIN),  DIFF_MAX);
            // APPLY
            servo0.setPosition((totalPos + diffPos) / 2.0);
            servo1.setPosition((totalPos - diffPos) / 2.0);

            // Torque Base Control
            if (gamepad1.dpad_up) {
                motor0.setPower(LIFT_POWER);
                motor1.setPower(LIFT_POWER);
            } else if (gamepad1.dpad_down) {
                motor0.setPower(-LIFT_POWER);
                motor1.setPower(-LIFT_POWER);
            } else if (gamepad1.dpad_right) {
                motor0.setPower(SPIN_POWER);
                motor1.setPower(-SPIN_POWER);
            } else if (gamepad1.dpad_left) {
                motor0.setPower(-SPIN_POWER);
                motor1.setPower(SPIN_POWER);
            } else {
                motor0.setPower(0);
                motor1.setPower(0);
            }

            // TELEMETRY
            telemetry.addData("Total Pos", "%.3f", totalPos);
            telemetry.addData("Diff Pos",  "%.3f", diffPos);
            telemetry.addData("Servo 0",   "%.3f", servo0.getPosition());
            telemetry.addData("Servo 1",   "%.3f", servo1.getPosition());
            telemetry.update();
        }
    }
}