package org.firstinspires.ftc.teamcode.Auto.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "TEACH Path (Drag + Odo)", group = "TEST")
public class TeachPathOPMode extends LinearOpMode {

    // Where to save on Control Hub
    public static String FILE = "/sdcard/FIRST/paths/blue_row_sweep.csv";

    // Sampling
    public static int SAMPLE_MS = 20;           // 50 Hz
    public static double DOWNSAMPLE_DIST = 0.5; // inches
    public static double DOWNSAMPLE_DEG  = 2.0; // degrees

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-48, 54, Math.toRadians(144.046));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("TEACH MODE");
        telemetry.addLine("1) Put robot at correct start pose.");
        telemetry.addLine("2) Press START. Then DRAG/PUSH robot by hand.");
        telemetry.addLine("3) Press (A) to STOP + SAVE.");
        telemetry.addLine("If you don't have a gamepad connected, use STOP button to end (won't save).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        long t0 = System.nanoTime();
        List<PathRecorder.Sample> samples = new ArrayList<>();

        while (opModeIsActive()) {
            // Update localization continuously (important)
            drive.localizer.update();

            Pose2d p = drive.localizer.getPose();
            double t = (System.nanoTime() - t0) / 1e9;

            samples.add(new PathRecorder.Sample(t, p.position.x, p.position.y, p.heading.toDouble()));

            telemetry.addData("File", FILE);
            telemetry.addData("Samples", samples.size());
            telemetry.addData("X", "%.2f", p.position.x);
            telemetry.addData("Y", "%.2f", p.position.y);
            telemetry.addData("HeadingDeg", "%.1f", Math.toDegrees(p.heading.toDouble()));
            telemetry.addLine("Press (A) to save and stop.");
            telemetry.update();

            if (gamepad1.a) break;

            sleep(SAMPLE_MS);
        }

        try {
            List<PathRecorder.Sample> ds = PathRecorder.downsample(
                    samples,
                    DOWNSAMPLE_DIST,
                    Math.toRadians(DOWNSAMPLE_DEG)
            );
            PathRecorder.save(FILE, ds);
            telemetry.addLine("SAVED ✅");
            telemetry.addData("Downsampled", ds.size());
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("SAVE FAILED ❌");
            telemetry.addData("Err", e.getMessage());
            telemetry.update();
        }

        sleep(1500);
    }
}