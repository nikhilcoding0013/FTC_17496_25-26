package org.firstinspires.ftc.teamcode.DifferentialRoboticArm;

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
@TeleOp(name = "ArmManipulation JOYSTICK")
public class ArmManipulationJOYSTICK extends LinearOpMode {

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
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ArmManipulation Ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        final double SPIN_POWER = 0.2;
        final double LIFT_POWER = 0.1;
        final double HOLD_POWER = 0.25;

        boolean holding = false;
        int holdPos0 = 0;
        int holdPos1 = 0;

        while (opModeIsActive() && !isStopRequested()) {

            // Step 1: compute tilt from left stick Y
            double tilt = 0.5 + (-gamepad1.left_stick_y * 0.25);
            // Step 2: compute maxSpin from tilt
            double maxSpin = Math.min(1.0 - tilt, tilt);
            // Step 3: compute spin from left stick X
            double spin = gamepad1.left_stick_x * maxSpin;
            // Step 4: compute servo positions
            double s0 = tilt + spin;
            double s1 = tilt - spin;
            // Apply
            servo0.setPosition(s0);
            servo1.setPosition(s1);

            // Motor Base Control
            double liftInput = -gamepad1.right_stick_y;
            double spinInput =  gamepad1.right_stick_x;

            if (liftInput != 0 || spinInput != 0) {
                // Moving
                holding = false;
                motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor0.setPower((liftInput * LIFT_POWER) + (spinInput * SPIN_POWER));
                motor1.setPower((liftInput * LIFT_POWER) - (spinInput * SPIN_POWER));
            } else {
                // Holding — capture position only once when stick is first released
                if (!holding) {
                    holdPos0 = motor0.getCurrentPosition();
                    holdPos1 = motor1.getCurrentPosition();
                    motor0.setTargetPosition(holdPos0);
                    motor1.setTargetPosition(holdPos1);
                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor0.setPower(HOLD_POWER);
                    motor1.setPower(HOLD_POWER);
                    holding = true;
                }
            }

            // TELEMETRY
            telemetry.addData("Tilt",        "%.3f", tilt);
            telemetry.addData("Spin",        "%.3f", spin);
            telemetry.addData("Max Spin",    "%.3f", maxSpin);
            telemetry.addData("Servo 0",     "%.3f", s0);
            telemetry.addData("Servo 1",     "%.3f", s1);
            telemetry.addData("Lift Input",  "%.3f", liftInput);
            telemetry.addData("Spin Input",  "%.3f", spinInput);
            telemetry.addData("Holding",     holding);
            telemetry.addData("Motor 0 Pos", motor0.getCurrentPosition());
            telemetry.addData("Motor 1 Pos", motor1.getCurrentPosition());
            telemetry.update();
        }
    }
}