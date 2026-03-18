package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "ArmManipulation")
public class ArmManipulation extends LinearOpMode {

    // SERVOS
    private Servo servo0;
    private Servo servo1;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // SERVOS
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo0.setPosition(0.5);
        servo1.setPosition(0.5);
        telemetry.addLine("ArmManipulation Ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            // LEFT STICK - both servos spin same direction
            double leftStickY = -gamepad1.left_stick_y;
            double leftServoPos = (leftStickY + 1.0) / 2.0;

            servo0.setPosition(leftServoPos);
            servo1.setPosition(leftServoPos);

            // RIGHT STICK - servos spin opposite directions
            double rightStickX = gamepad1.right_stick_x;
            double servo0Pos = (rightStickX + 1.0) / 2.0;
            double servo1Pos = (-rightStickX + 1.0) / 2.0;

            servo0.setPosition(servo0Pos);
            servo1.setPosition(servo1Pos);

            // TELEMETRY
            telemetry.addData("Left Stick Y",     "%.3f", leftStickY);
            telemetry.addData("Right Stick X",    "%.3f", rightStickX);
            telemetry.addData("Servo 0 Position", "%.3f", servo0.getPosition());
            telemetry.addData("Servo 1 Position", "%.3f", servo1.getPosition());
            telemetry.update();
        }
    }
}