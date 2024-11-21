package com.example.meepmeeptesting;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.TrajectoryBuilder;

public class Meep{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12.8, -62, 90))
                .splineTo(new Vector2d(-33,-35.7), Math.toRadians(120))
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                        .turnTo(240)
                        .waitSeconds(1)
                .splineTo(new Vector2d(-47, -33), Math.toRadians(90))
                        .waitSeconds(1)
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                        .turnTo(240)
                        .waitSeconds(1)
                .splineTo(new Vector2d(-57., -34), Math.toRadians(90))
                        .waitSeconds(1)
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .turnTo(240)
                        .waitSeconds(1)
                .splineTo(new Vector2d(-35, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-54, -25), Math.toRadians(180))
                        .waitSeconds(1)
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))

                .turnTo(240)



                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}