package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class MeepMeepViz {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double startX = -49.84;
        double startY = 55.93;
        double startHeading = Math.toRadians(144.046);

        // Simulating motifId = 22, aboveRow = 23
        bot.runAction(bot.getDrive().actionBuilder(
                        new Pose2d(startX, startY, startHeading))

                // Step 1 - drive backwards 7 inches
                .setReversed(true)
                .splineTo(new Vector2d(
                        startX + 7 * Math.cos(Math.toRadians(144.046 + 180)),
                        startY + 7 * Math.sin(Math.toRadians(144.046 + 180))
                ), Math.toRadians(144.046 + 180))
                .setReversed(false)

                // Step 3 - drive to (-18, 35) at 64 degrees
                .splineToLinearHeading(new Pose2d(-18, 35, Math.toRadians(64)), Math.toRadians(64))

                // Step 5 - collectRow(22)
                .splineToLinearHeading(new Pose2d(-24, -12.11, Math.PI), Math.PI)
                .lineToX(-50, new TranslationalVelConstraint(8.0), new ProfileAccelConstraint(-20, 20))
                // safe intermediate to avoid row 23
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-18, -5, Math.toRadians(135)), Math.toRadians(135 + 180))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-18, 18, Math.toRadians(135)), Math.toRadians(135))

                // Step 7 - collectRow(23)
                .splineToLinearHeading(new Pose2d(-24, 11.56, Math.PI), Math.PI)
                .lineToX(-50, new TranslationalVelConstraint(8.0), new ProfileAccelConstraint(-20, 20))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-18, 18, Math.toRadians(135)), Math.toRadians(135 + 180))
                .setReversed(false)

                // Step 9 - get off line
                .strafeTo(new Vector2d(-40, -15))

                .build()
        );

        BufferedImage img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/src/main/res/DecodeField.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}