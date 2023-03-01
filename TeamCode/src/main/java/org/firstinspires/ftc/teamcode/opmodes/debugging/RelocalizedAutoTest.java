package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous
public class RelocalizedAutoTest extends LinearOpMode
{
    OpenCvCamera camera;
    Trajectory startToPole;
    Trajectory poleToCone;

    PoleDetectionPipeline poleDetectionPipeline;

    public DcMotorEx motorFrontLeft = null;
    public DcMotorEx motorFrontRight = null;
    public DcMotorEx motorBackLeft = null;
    public DcMotorEx motorBackRight = null;

    private DistanceSensor distanceSensor = null;

    private int motorVelocity;

    private double sensorOffset;

    private BNO055IMU imu;

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

        // Define and Initialize Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorVelocity = 300;

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");

        sensorOffset = 139.7;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        poleDetectionPipeline = new PoleDetectionPipeline();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36, 62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //Drive to the high goal from blue left starting pose
        startToPole = drive.trajectoryBuilder(startPose)
                .forward(40)
                .splineTo(new Vector2d(35, 8), Math.toRadians(215))
                .build();

        camera.setPipeline(poleDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pole X", poleDetectionPipeline.poleX);
            telemetry.addData("Pole Y", poleDetectionPipeline.poleY);
            telemetry.addData("Pole Width", poleDetectionPipeline.poleWidth);
            telemetry.addData("Pole Center", (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0)));
            telemetry.addData("Distance sensor", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();




        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


        drive.followTrajectory(startToPole);

        /* Wait until program ends */
        sleep(30000);
    }





}