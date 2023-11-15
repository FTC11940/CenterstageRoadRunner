package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -62, (Math.toRadians(90)) ))
                                .waitSeconds(0.5)// pause
                                //.turn(Math.toRadians(90)) // Turn towards the center of the field
                                .forward(20) // Forward toward spike Mark
                                .waitSeconds(1)
                                .turn(Math.toRadians(-90))
                                .strafeLeft(6)
                                .lineToLinearHeading(new Pose2d(48,-36, Math.toRadians(0)) )
                                // .forward(80)
                                .waitSeconds(1)
                                .back(5)
                                // .strafeRight(20)
                                // .forward(15)
                                .splineTo(new Vector2d(60,-58),Math.toRadians(0) )
                                .waitSeconds(1)


//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))

                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
