package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MotorFullSpeed2Sec", group = "Linear Opmode")
public class MotorFullSpeed2Sec extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {

        // Initialize motor (make sure name matches config)
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        if (opModeIsActive()) {

            // Set motor to full speed
            motor.setPower(1.0);

            // Run for 2 seconds
            sleep(2000);

            // Stop motor
            motor.setPower(0.0);
        }
    }
}