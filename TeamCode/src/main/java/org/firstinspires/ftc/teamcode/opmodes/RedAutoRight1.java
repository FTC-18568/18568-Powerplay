package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
//BLUEAUTORIGHT1 - STRAIGHT TO SIGNAL LOCATION
public class RedAutoRight1 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int tagid;
    Trajectory myTrajectory;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // I have been told that the default values are sufficient for detection.
    // The calibrated values are in res/xml/teamwebcamcalibrations.xml
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.035;


    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);



        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    tagid = tag.id;
                    telemetry.addLine(tagid);
                }
            }
            else
            {
                telemetry.addLine("No Tags Detected");

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        switch(tagid) {
            case 1:
                myTrajectory = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(20, -59), Math.toRadians(180))
                        .splineTo(new Vector2d(13, -36), Math.toRadians(90))
                        .build();

                break;
            case 2:
                myTrajectory = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                        .build();
                break;
            case 3:
                myTrajectory = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(52, -59), Math.toRadians(0))
                        .splineTo(new Vector2d(60, -36), Math.toRadians(90))
                        .build();
                break;
        }

        drive.followTrajectory(myTrajectory);



        /* Wait until program ends */
        sleep(30000);
    }





}