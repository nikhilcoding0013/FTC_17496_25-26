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

        double motifRowY = -12.11;
        double aboveRowY = 11.56;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)

                        // Step 2 - reverse 7 inches along starting heading
                        .lineToXLinearHeading(
                                startPose.position.x + 7 * Math.cos(Math.toRadians(144.046 + 180)),
                                Math.toRadians(144.046)
                        )

                        // Step 4 - reverse to x = -24 maintaining heading
                        .lineToXLinearHeading(-24, Math.toRadians(144.046))

                        // Step 5 - turn inplace to face motif at 60 degrees
                        .turnTo(Math.toRadians(60))

                        // Step 7 - turn inplace to 90 degrees
                        .turnTo(Math.toRadians(90))

                        // Step 8 - reverse in Y to row 22, then turn to 180
                        .lineToYLinearHeading(motifRowY, Math.toRadians(90))
                        .turnTo(Math.toRadians(180))

                        // Step 9 - drive forward into row to x=-48
                        .lineToX(-48,
                                new TranslationalVelConstraint(10.0),
                                new ProfileAccelConstraint(-20, 20))

                        // Step 10 - back straight out to x=-24
                        .lineToX(-24)

                        // Step 11 - turn to 90, spline to (-24, 24) heading aligned to path
                        .turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(-24, 24), Math.toRadians(90))

                        // Step 12 - turn in place to shooting angle
                        .turnTo(Math.toRadians(135))

                        // Step 14 - collect row 23
                        .turnTo(Math.toRadians(90))
                        .lineToYLinearHeading(aboveRowY, Math.toRadians(90))
                        .turnTo(Math.toRadians(180))
                        .lineToX(-48,
                                new TranslationalVelConstraint(10.0),
                                new ProfileAccelConstraint(-20, 20))
                        .lineToX(-24)
                        .turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(-24, 24), Math.toRadians(90))

                        // Step 12b - turn in place to shooting angle
                        .turnTo(Math.toRadians(135))

                        // Step 16 - get off line
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