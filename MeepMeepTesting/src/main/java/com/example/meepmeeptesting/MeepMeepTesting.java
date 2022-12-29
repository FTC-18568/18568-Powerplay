package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static int startingX;
    public static int startingY;
    public static int startingPos; //blue left = 1, blue right = 2, red left = 3, red right = 4;
    public static int startingAngle;



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        startingX = 0;
        startingY = 0;
        startingAngle = 0;
        startingPos = 1;



        switch(startingPos) {
            case 1: //blue left
                //Log.d("myTag", "This is my message 1");
                startingX = 36;
                startingY = 62;
                startingAngle = 270;
                break;
            case 2: //blue right
                //Log.d("myTag", "This is my message");
                startingX = -36;
                startingY = 62;
                startingAngle = 270;
                break;
            case 3: //red left
                startingX = -36;
                startingY = -62;
                startingAngle = 90;
                break;
            case 4: //red right
                startingX = 36;
                startingY = -62;
                startingAngle = 90;
                break;


        }

        //https://firstinspiresst01.blob.core.windows.net/first-energize-ftc/game-manual-part-2-traditional.pdf

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 18)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(startingX, startingY, Math.toRadians(startingAngle))) //starting position (x,y)

                                        //Drive to the high goal from blue left
                                        .forward(30)
                                        .splineTo(new Vector2d(20, 13), Math.toRadians(150))
                                        .waitSeconds(2)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(56, 13), Math.toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(20, 13), Math.toRadians(150))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(56, 13), Math.toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(20, 13), Math.toRadians(150))



                                        //blue location 1 (left team)
//                                .splineTo(new Vector2d(52, 59), Math.toRadians(0))
//                                .splineTo(new Vector2d(61, 36), Math.toRadians(270))


//                                //blue location 2 (left team)

//                                .splineTo(new Vector2d(36, 36), Math.toRadians(270))


//                                //blue location 3 (left team)

//                                .splineTo(new Vector2d(20, 59), Math.toRadians(180))
//                                .splineTo(new Vector2d(13, 36), Math.toRadians(270))


//        ----------------------------------------------------------------------------------------------

                                        //blue location 3 (right team)

//                                .splineTo(new Vector2d(-52, 59), Math.toRadians(180))
//                                .splineTo(new Vector2d(-61, 36), Math.toRadians(270))


//                                //blue location 2 (right team)

//                                .splineTo(new Vector2d(-36, 36), Math.toRadians(270))

//                                //blue location 1 (right team)
//                                .splineTo(new Vector2d(-20, 59), Math.toRadians(0))
//                                .splineTo(new Vector2d(-13, 36), Math.toRadians(270))
//

// -----------------------------------------------------------------------------------------------

                                        //red location 1 (left team)
//                                .splineTo(new Vector2d(-52, -59), Math.toRadians(180))
//                                .splineTo(new Vector2d(-61, -36), Math.toRadians(90))


//                                //red location 2 (left team)

//                                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))


//                                //red location 3 (left team)

//                                .splineTo(new Vector2d(-20, -59), Math.toRadians(0))
//                                .splineTo(new Vector2d(-13, -36), Math.toRadians(90))


//----------------------------------------------------------------------------------------

                                        //red location 1 (right team)
//                                .splineTo(new Vector2d(20, -59), Math.toRadians(180))
//                                .splineTo(new Vector2d(13, -36), Math.toRadians(90))


//                                //red location 2 (right team)

//                                .splineTo(new Vector2d(36, -36), Math.toRadians(90))


//                                //red location 3 (right team)

//                                .splineTo(new Vector2d(52, -59), Math.toRadians(0))
//                                .splineTo(new Vector2d(61, -36), Math.toRadians(90))


                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}