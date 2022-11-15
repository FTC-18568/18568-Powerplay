package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepStraightToSignal {
    public static int startingX;
    public static int startingY;
    public static int startingPos; //blue left = 1, blue right = 2, red left = 3, red right = 4;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        startingX = 0;
        startingY = 0;
        startingPos = 2;


        switch(startingPos) {
            case 1:
                startingX = 36;
                startingY = 62;
                break;
            case 2:
                startingX = -36;
                startingY = 62;
                break;


        }

        //https://firstinspiresst01.blob.core.windows.net/first-energize-ftc/game-manual-part-2-traditional.pdf

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startingX, startingY, Math.toRadians(-90) )) //starting position (x,y)


                                //blue location 1 (left team)
//                                .splineTo(new Vector2d(36, 25), Math.toRadians(270))
//                                .splineToSplineHeading(new Pose2d(55, 11, Math.toRadians(200)), Math.toRadians(0)) //first radian is way it faces
//                                .waitSeconds(1.5)
//                                .splineTo(new Vector2d(58, 36), Math.toRadians(90))

//
//                                //blue location 2 (left team)
//                                .splineTo(new Vector2d(36, 25), Math.toRadians(270))
//                                .splineToSplineHeading(new Pose2d(55, 11, Math.toRadians(200)), Math.toRadians(0))
//                                .waitSeconds(1.5)
//                                .splineTo(new Vector2d(35, 36), Math.toRadians(90))



//                                //blue location 3 (left team)
//                               .splineTo(new Vector2d(36, 25), Math.toRadians(270))
//                                .splineToSplineHeading(new Pose2d(55, 11, Math.toRadians(200)), Math.toRadians(0))
//                                .waitSeconds(1.5)
//                                .splineTo(new Vector2d(11, 36), Math.toRadians(90))

                                    //blue location 3 (right team)

                              //  .splineTo(new Vector2d(-58, 36), Math.toRadians(270))


//
//
//                                //blue location 2 (right team)

                                .splineTo(new Vector2d(-35, 36), Math.toRadians(270))
//
//                                //blue location 1 (right team)
                               // .splineTo(new Vector2d(-11, 36), Math.toRadians(270))
//






                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}