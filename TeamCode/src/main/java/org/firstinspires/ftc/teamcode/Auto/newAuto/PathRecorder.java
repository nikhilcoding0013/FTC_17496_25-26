package org.firstinspires.ftc.teamcode.Auto.newAuto;

import com.acmerobotics.roadrunner.Pose2d;

import java.io.*;
import java.util.*;

public class PathRecorder {
    public static class Sample {
        public final double t;   // seconds since start
        public final double x;
        public final double y;
        public final double h;   // radians
        public Sample(double t, double x, double y, double h) {
            this.t = t; this.x = x; this.y = y; this.h = h;
        }
    }

    public static void save(String filename, List<Sample> samples) throws IOException {
        File f = new File(filename);
        f.getParentFile().mkdirs();
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(f))) {
            bw.write("t,x,y,h\n");
            for (Sample s : samples) {
                bw.write(String.format(Locale.US, "%.6f,%.6f,%.6f,%.6f\n", s.t, s.x, s.y, s.h));
            }
        }
    }

    public static List<Sample> load(String filename) throws IOException {
        List<Sample> out = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(new File(filename)))) {
            String line = br.readLine(); // header
            while ((line = br.readLine()) != null) {
                String[] p = line.trim().split(",");
                if (p.length < 4) continue;
                double t = Double.parseDouble(p[0]);
                double x = Double.parseDouble(p[1]);
                double y = Double.parseDouble(p[2]);
                double h = Double.parseDouble(p[3]);
                out.add(new Sample(t, x, y, h));
            }
        }
        return out;
    }

    public static Pose2d toPose(Sample s) {
        return new Pose2d(s.x, s.y, s.h);
    }

    // Downsample to reduce RR complexity (keeps turns)
    public static List<Sample> downsample(List<Sample> in, double minDist, double minHeadingRad) {
        if (in.isEmpty()) return in;
        List<Sample> out = new ArrayList<>();
        out.add(in.get(0));
        Sample last = in.get(0);

        for (int i = 1; i < in.size(); i++) {
            Sample cur = in.get(i);
            double dx = cur.x - last.x;
            double dy = cur.y - last.y;
            double dist = Math.hypot(dx, dy);
            double dh = angleWrap(cur.h - last.h);
            if (dist >= minDist || Math.abs(dh) >= minHeadingRad) {
                out.add(cur);
                last = cur;
            }
        }
        // ensure final point
        Sample end = in.get(in.size() - 1);
        if (out.get(out.size() - 1) != end) out.add(end);
        return out;
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}