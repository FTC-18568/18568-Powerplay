package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static int startingX;
    public static int startingY;
    public static int startingPos; //blue left = 1, blue right = 2, red left = 3, red right = 4;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        startingX = 0;
        startingY = 0;
        startingPos = 2;


        switch(startingPos) {
            case 1:
                startingX = 35;
                startingY = 62;
                break;
            case 2:
                startingX = -35;
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


                                //.waitSeconds(1) - stay still
                                //each tile is 23.33 width
                                //hyptonouse length = 33


                                //blue location 1 (left team)
                                .forward(39)
                                .turn(Math.toRadians(65))
                                .forward(30)
                                .turn(Math.toRadians(205))
                                .turn(Math.toRadians(-90))
                                .forward(24)
                                .waitSeconds(2)
//
//
//                                //blue location 2 (left team)
//                                .forward(39)
//                                .turn(Math.toRadians(65))
//                                .forward(30)
//                                .turn(Math.toRadians(205))
//                                .turn(Math.toRadians(-43))
//                                .forward(36)
//                                .waitSeconds(2)
//
//                                //blue location 3 (left team)
//                                .forward(39)
//                                .turn(Math.toRadians(65))
//                                .forward(30)
//                                .turn(Math.toRadians(205))
//                                .turn(Math.toRadians(-25))
//                                .forward(55)
//                                .waitSeconds(2)

                                    //blue location 3 (right team)
//                                .forward(39)
//                                .turn(Math.toRadians(-65))
//                                .forward(30)
//                                .turn(Math.toRadians(-205))
//                                .turn(Math.toRadians(90))
//                                .forward(24)
//                                .waitSeconds(2)
//
//
//                                //blue location 2 (right team)
//                                .forward(39)
//                                .turn(Math.toRadians(-65))
//                                .forward(30)
//                                .turn(Math.toRadians(-205))
//                                .turn(Math.toRadians(43))
//                                .forward(36)
//                                .waitSeconds(2)
//
//                                //blue location 1 (right team)
//                                .forward(39)
//                                .turn(Math.toRadians(-65))
//                                .forward(30)
//                                .turn(Math.toRadians(-205))
//                                .turn(Math.toRadians(25))
//                                .forward(55)
//                                .waitSeconds(2)






                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}