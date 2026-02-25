package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class BlueAutoNewMeepMeep {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-48, 54, Math.toRadians(144.046));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // Step 1 - drive backwards 7 inches
                .setReversed(true)
                .splineTo(new Vector2d(
                        -49.84 + 7 * Math.cos(Math.toRadians(144.046 + 180)),
                        55.93 + 7 * Math.sin(Math.toRadians(144.046 + 180))
                ), Math.toRadians(144.046 + 180))
                .setReversed(false)

                // Step 3 - drive to (-18, 35) at 64 degrees
                .splineTo(new Vector2d(-18, 35), Math.toRadians(64))

                // Step 5 - collectRow (using motifId = 22 as default)
                .splineTo(new Vector2d(-24, -12.11), Math.PI)
                .lineToX(-24 - 25,
                        new TranslationalVelConstraint(8.0),
                        new ProfileAccelConstraint(-20, 20))
                .setReversed(true)
                .splineTo(new Vector2d(-18, -5), Math.toRadians(135 + 180))
                .setReversed(false)
                .splineTo(new Vector2d(-18, 18), Math.toRadians(135))

                // Step 7 - collectRow aboveRow (motifId + 1 = 23)
                .splineTo(new Vector2d(-24, 11.56), Math.PI)
                .lineToX(-24 - 25,
                        new TranslationalVelConstraint(8.0),
                        new ProfileAccelConstraint(-20, 20))
                .setReversed(true)
                .splineTo(new Vector2d(-18, 18), Math.toRadians(135 + 180))
                .setReversed(false)

                // Step 9 - get off line
                .splineTo(new Vector2d(-40, -15), Math.toRadians(270))
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
                .addEntity(myBot)
                .start();
    }
}