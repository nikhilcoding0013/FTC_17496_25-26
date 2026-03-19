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

public class BlueAutoFarMeepMeep {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-16, -63, Math.toRadians(90));

        double angle1 = Math.atan2(72 - (-55), -72 - (-16));
        double angle2 = Math.atan2(72 - (-50), -72 - (-16));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)

                        // Step 1 - move in +Y to (-16, -55)
                        .lineToY(-55)

                        // Step 2 - turn to face (-72, 72)
                        .turnTo(angle1)

                        // Step 4 - drive forward diagonally at angle1 to y=-35
                        .splineTo(new Vector2d(-24, -35), angle1)

                        // Step 5 - turn to heading 180
                        .turnTo(Math.toRadians(180))

                        // Step 6 - drive in -X to x=-52
                        .lineToX(-52,
                                new TranslationalVelConstraint(10.0),
                                new ProfileAccelConstraint(-20, 20))

                        // Step 7 - backwards spline to (-16, -50), heading tangent to path
                        .setReversed(true)
                        .splineTo(new Vector2d(-16, -55), Math.toRadians(295))
                        .setReversed(false)

                        // small move so turnTo has a valid prior path
                        .lineToY(-55)

                        // Step 8 - turn to face (-72, 72)
                        .turnTo(angle2)

                        // Step 10 - drive forward diagonally at angle2 to y=-45
                        .splineTo(new Vector2d(-20, -45), angle2)

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