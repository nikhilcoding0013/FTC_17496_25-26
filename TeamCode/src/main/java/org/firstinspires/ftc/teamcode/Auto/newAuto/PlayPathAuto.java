package org.firstinspires.ftc.teamcode.Auto.newAuto;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "AUTO Play Taught Path (SMOOTH v4 - No Turn Creep)", group = "TEST")
public class PlayPathAuto extends LinearOpMode {

    public static String FILE_NAME = "blue_row_sweep.csv";

    // ---------- SMOOTHING (start with these) ----------
    // These kill “micro-corrections” that look like jerk/creep.
    public static double MIN_WAYPOINT_DIST_IN = 8.0;
    public static double MIN_WAYPOINT_TURN_DEG = 20.0;
    public static int MAX_WAYPOINTS = 30;

    // Only apply heading changes at big turns
    public static double APPLY_TURN_DEG = 25.0;

    // Deadbands: do not do tiny translation commands
    public static double AXIS_DEADBAND_IN = 2.0;

    // After turning, require EVEN MORE translation to move (prevents tiny forward creep)
    public static double POST_TURN_DEADBAND_IN = 3.0;

    // Consistent move axis order (less back/forth)
    public static boolean MOVE_Y_THEN_X = true;

    // Spin-in-place handling (for taught “rotate without moving” moments)
    public static boolean INSERT_TURN_IF_LOW_TRANSLATION = true;
    public static double LOW_TRANSLATION_IN = 1.0;
    public static double TURN_ONLY_THRESHOLD_DEG = 15.0;

    // Optional: go to first recorded point before playing
    public static boolean SNAP_TO_FIRST_POINT = false;

    private String fullPath() {
        return Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/paths/" + FILE_NAME;
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-48, 54, Math.toRadians(144.046));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        List<PathRecorder.Sample> raw;
        try {
            raw = PathRecorder.load(fullPath());
        } catch (Exception e) {
            telemetry.addLine("LOAD FAILED ❌");
            telemetry.addData("Path", fullPath());
            telemetry.addData("Err", e.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

        if (raw.size() < 2) {
            telemetry.addLine("Not enough points to play.");
            telemetry.update();
            sleep(2000);
            return;
        }

        List<PathRecorder.Sample> pts = downsampleForPlayback(
                raw,
                MIN_WAYPOINT_DIST_IN,
                Math.toRadians(MIN_WAYPOINT_TURN_DEG),
                MAX_WAYPOINTS
        );

        telemetry.addLine("Loaded path ✅");
        telemetry.addData("Raw", raw.size());
        telemetry.addData("Pts", pts.size());
        telemetry.addData("Path", fullPath());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder builder = drive.actionBuilder(drive.localizer.getPose());

        if (SNAP_TO_FIRST_POINT && pts.size() > 0) {
            PathRecorder.Sample s0 = pts.get(0);
            builder = builder.lineToX(s0.x).lineToY(s0.y).turnTo(s0.h);
        }

        // Track our assumed pose as we append actions
        double curX = drive.localizer.getPose().position.x;
        double curY = drive.localizer.getPose().position.y;
        double curH = drive.localizer.getPose().heading.toDouble();

        for (int i = 1; i < pts.size(); i++) {
            PathRecorder.Sample prev = pts.get(i - 1);
            PathRecorder.Sample now  = pts.get(i);

            double segDx = now.x - prev.x;
            double segDy = now.y - prev.y;
            double segDist = Math.hypot(segDx, segDy);

            double dx = now.x - curX;
            double dy = now.y - curY;

            double dHeadDegFromCur = Math.toDegrees(angleWrap(now.h - curH));
            double dHeadDegSeg = Math.toDegrees(angleWrap(now.h - prev.h));

            // 1) If you basically spun in place during teach mode, do a pure turn
            if (INSERT_TURN_IF_LOW_TRANSLATION
                    && segDist < LOW_TRANSLATION_IN
                    && Math.abs(dHeadDegFromCur) >= TURN_ONLY_THRESHOLD_DEG) {
                builder = builder.turnTo(now.h);
                curH = now.h;
                continue;
            }

            // 2) Translate to the waypoint (consistent axis order + deadband)
            if (MOVE_Y_THEN_X) {
                if (Math.abs(dy) >= AXIS_DEADBAND_IN) {
                    builder = builder.lineToY(now.y);
                    curY = now.y;
                }
                if (Math.abs(dx) >= AXIS_DEADBAND_IN) {
                    builder = builder.lineToX(now.x);
                    curX = now.x;
                }
            } else {
                if (Math.abs(dx) >= AXIS_DEADBAND_IN) {
                    builder = builder.lineToX(now.x);
                    curX = now.x;
                }
                if (Math.abs(dy) >= AXIS_DEADBAND_IN) {
                    builder = builder.lineToY(now.y);
                    curY = now.y;
                }
            }

            // 3) Apply turn ONLY at major turns.
            //    AND after turning, DO NOT allow tiny translation corrections (the creep fix).
            if (Math.abs(dHeadDegSeg) >= APPLY_TURN_DEG) {
                builder = builder.turnTo(now.h);
                curH = now.h;

                // After turn, only move if we're still FAR from the waypoint (prevents tiny forward move)
                double ndx = now.x - curX;
                double ndy = now.y - curY;

                if (MOVE_Y_THEN_X) {
                    if (Math.abs(ndy) >= POST_TURN_DEADBAND_IN) {
                        builder = builder.lineToY(now.y);
                        curY = now.y;
                    }
                    if (Math.abs(ndx) >= POST_TURN_DEADBAND_IN) {
                        builder = builder.lineToX(now.x);
                        curX = now.x;
                    }
                } else {
                    if (Math.abs(ndx) >= POST_TURN_DEADBAND_IN) {
                        builder = builder.lineToX(now.x);
                        curX = now.x;
                    }
                    if (Math.abs(ndy) >= POST_TURN_DEADBAND_IN) {
                        builder = builder.lineToY(now.y);
                        curY = now.y;
                    }
                }
            }
        }

        Actions.runBlocking(builder.build());
    }

    private static List<PathRecorder.Sample> downsampleForPlayback(List<PathRecorder.Sample> in,
                                                                   double minDist,
                                                                   double minHeadingRad,
                                                                   int maxPts) {
        if (in.isEmpty()) return in;

        List<PathRecorder.Sample> out = new ArrayList<>();
        out.add(in.get(0));
        PathRecorder.Sample last = in.get(0);

        for (int i = 1; i < in.size(); i++) {
            PathRecorder.Sample cur = in.get(i);

            double dx = cur.x - last.x;
            double dy = cur.y - last.y;
            double dist = Math.hypot(dx, dy);

            double dh = Math.abs(angleWrap(cur.h - last.h));

            if (dist >= minDist || dh >= minHeadingRad) {
                out.add(cur);
                last = cur;
                if (out.size() >= maxPts) break;
            }
        }

        // Ensure final point included
        PathRecorder.Sample end = in.get(in.size() - 1);
        PathRecorder.Sample lastOut = out.get(out.size() - 1);

        double endDist = Math.hypot(end.x - lastOut.x, end.y - lastOut.y);
        double endDh = Math.abs(angleWrap(end.h - lastOut.h));

        if (endDist > 1e-6 || endDh > 1e-6) {
            if (out.size() < maxPts) out.add(end);
            else out.set(out.size() - 1, end);
        }

        return out;
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}