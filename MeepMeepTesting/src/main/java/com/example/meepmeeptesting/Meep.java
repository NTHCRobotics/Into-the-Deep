package com.example.meepmeeptesting;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.SleepAction;


public class Meep{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -62.5, 180))
                        .setReversed(true)
                . splineTo(new Vector2d(-55, -58), Math.toRadians(225))
                .setReversed(false)
                .splineTo(new Vector2d(-47.8, -31.9), Math.toRadians(90))
                .setReversed(true)
              //  .waitSeconds(1)
                .splineTo(new Vector2d( -55, -58), Math.toRadians(225))
                        .setReversed(false)
                .splineTo(new Vector2d(-56.6,-36.6), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(-55,-58), Math.toRadians(225))

                .setReversed(false)
                        .splineTo(new Vector2d(-68, -36), Math.toRadians(90))

                        .setReversed(true)
                        .splineTo(new Vector2d(-55, -58), Math.toRadians(225))
                        .setReversed(false)
                        .splineTo(new Vector2d(-32, -11), Math.toRadians(90))
                        .setReversed(true)
                                .splineTo(new Vector2d(-19, -10.4), Math.toRadians(225))










                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}