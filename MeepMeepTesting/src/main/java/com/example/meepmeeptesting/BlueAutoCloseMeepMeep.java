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

public class BlueAutoCloseMeepMeep {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-48, 54, Math.toRadians(144.046));

        double row23Y = 11.56;
        double row22Y = -12.11;
        double shootingX = -16;
        double shootingY = 8;
        double goalAngle = Math.atan2(72 - shootingY, -72 - shootingX);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)

                        // Step 1 - reverse 7 inches along starting heading
                        .lineToXLinearHeading(
                                startPose.position.x + 7 * Math.cos(Math.toRadians(144.046 + 180)),
                                Math.toRadians(144.046)
                        )

                        // collect row 23 - backwards spline to lineup
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, row23Y), Math.toRadians(300))
                        .setReversed(false)

                        // adjust Y
                        .lineToY(row23Y)

                        // turn to 180, drive into row
                        .turnTo(Math.toRadians(180))
                        .lineToX(-52,
                                new TranslationalVelConstraint(10.0),
                                new ProfileAccelConstraint(-20, 20))

                        // backwards spline back to shooting position
                        .setReversed(true)
                        .splineTo(new Vector2d(shootingX, shootingY), Math.toRadians(310))
                        .setReversed(false)

                        // nudge so turnTo has valid prior path
                        .lineToY(shootingY - 0.01)

                        // turn to face (-72, 72)
                        .turnTo(goalAngle)

                        /* collect row 22 - backwards spline to lineup
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, row22Y), Math.toRadians(200))
                        .setReversed(false)

                        // adjust Y
                        .lineToY(row22Y)

                        // turn to 180, drive into row
                        .turnTo(Math.toRadians(180))
                        .lineToX(-52,
                                new TranslationalVelConstraint(10.0),
                                new ProfileAccelConstraint(-20, 20))

                        // backwards spline back to shooting position
                        .setReversed(true)
                        .splineTo(new Vector2d(shootingX, shootingY), Math.toRadians(350))
                        .setReversed(false)

                        // nudge so turnTo has valid prior path
                        .lineToY(shootingY - 0.01)

                        // turn to face (-72, 72)
                        .turnTo(goalAngle)
                        */
                        // get off line
                        .splineTo(new Vector2d(-40, 10), Math.toRadians(200))

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