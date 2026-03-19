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
        servo0 = hardwareMap.get(Servo.class, "hoodR");
        servo1 = hardwareMap.get(Servo.class, "hoodL");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo0.setPosition(0.5);
        servo1.setPosition(0.5);
        telemetry.addLine("ArmManipulation Ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            double minPos = 0.25;
            double maxPos = 0.75;

            // DPAD UP/DOWN → both servos same direction
            if (gamepad1.dpad_up) {
                servo0.setPosition(maxPos);
                servo1.setPosition(maxPos);
            } else if (gamepad1.dpad_down) {
                servo0.setPosition(minPos);
                servo1.setPosition(minPos);
            }

            // DPAD LEFT/RIGHT → opposite directions
            if (gamepad1.dpad_right) {
                servo0.setPosition(maxPos);
                servo1.setPosition(minPos);
            } else if (gamepad1.dpad_left) {
                servo0.setPosition(minPos);
                servo1.setPosition(maxPos);
            }

            telemetry.addData("Servo 0", servo0.getPosition());
            telemetry.addData("Servo 1", servo1.getPosition());
            telemetry.update();
        }
    }
}