package org.firstinspires.ftc.teamcode.DifferentialRoboticArm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@TeleOp(name = "ArmManipulation GESTURE")
public class ArmManipulationGESTURE extends LinearOpMode {

    // SERVOS
    private Servo servo0;
    private Servo servo1;
    // MOTORS
    private DcMotorEx motor0;
    private DcMotorEx motor1;

    public static double SPIN_VEL = 250;
    public static double LIFT_VEL = 250;

    // Gesture input values — written by UDP thread, read by OpMode loop
    private volatile float gestureLeftX  = 0;
    private volatile float gestureLeftY  = 0;
    private volatile float gestureRightX = 0;
    private volatile float gestureRightY = 0;

    private volatile boolean udpRunning = false;
    private DatagramSocket udpSocket;

    private void startUdpReceiver() {
        udpRunning = true;
        Thread udpThread = new Thread(() -> {
            try {
                udpSocket = new DatagramSocket(9000);
                byte[] buf = new byte[16]; // 4 floats x 4 bytes
                DatagramPacket packet = new DatagramPacket(buf, buf.length);
                while (udpRunning) {
                    udpSocket.receive(packet);
                    ByteBuffer bb = ByteBuffer.wrap(buf).order(ByteOrder.LITTLE_ENDIAN);
                    gestureLeftX  = bb.getFloat();
                    gestureLeftY  = bb.getFloat();
                    gestureRightX = bb.getFloat();
                    gestureRightY = bb.getFloat();
                }
            } catch (Exception e) {
                // socket closed on stop — expected
            }
        });
        udpThread.setDaemon(true);
        udpThread.start();
    }

    private void stopUdpReceiver() {
        udpRunning = false;
        if (udpSocket != null && !udpSocket.isClosed()) {
            udpSocket.close();
        }
    }

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
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start UDP receiver before waitForStart so it's ready immediately
        startUdpReceiver();

        telemetry.addLine("ArmManipulation GESTURE Ready");
        telemetry.addLine("Waiting for gesture input on port 9000...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            // Servo Module Control — left hand replaces left stick
            double tilt    = 0.5 + (-gestureLeftY * 0.25);
            double maxSpin = Math.min(1.0 - tilt, tilt);
            double spin    = gestureLeftX * maxSpin;
            double s0      = tilt + spin;
            double s1      = tilt - spin;
            servo0.setPosition(s0);
            servo1.setPosition(s1);

            // Motor Base Control — right hand replaces right stick
            double liftInput = -gestureRightY;
            double spinInput =  gestureRightX;
            motor0.setVelocity((liftInput * LIFT_VEL) + (spinInput * SPIN_VEL));
            motor1.setVelocity((liftInput * LIFT_VEL) - (spinInput * SPIN_VEL));

            // TELEMETRY
            telemetry.addData("Gesture LX",  "%.3f", gestureLeftX);
            telemetry.addData("Gesture LY",  "%.3f", gestureLeftY);
            telemetry.addData("Gesture RX",  "%.3f", gestureRightX);
            telemetry.addData("Gesture RY",  "%.3f", gestureRightY);
            telemetry.addData("Servo 0",     "%.3f", s0);
            telemetry.addData("Servo 1",     "%.3f", s1);
            telemetry.addData("Motor 0 Pos", motor0.getCurrentPosition());
            telemetry.addData("Motor 1 Pos", motor1.getCurrentPosition());
            telemetry.update();
        }

        stopUdpReceiver();
    }
}