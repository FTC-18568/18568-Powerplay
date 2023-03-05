package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    Trajectory coneToPole;

    PoleDetectionPipeline poleDetectionPipeline;

    public DcMotorEx motorFrontLeft = null;
    public DcMotorEx motorFrontRight = null;
    public DcMotorEx motorBackLeft = null;
    public DcMotorEx motorBackRight = null;

    private DistanceSensor distanceSensor = null;

    private int motorVelocity;

    private double sensorOffset;

    private Pose2d relocalizedPose;

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

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                .splineTo(new Vector2d(35, 10), Math.toRadians(215))
                .build();

        poleToCone = null;

        coneToPole = drive.trajectoryBuilder(new Pose2d(56, 12, Math.toRadians(180)), false)
                .forward(5)
                .splineTo(new Vector2d(35, 10), Math.toRadians(215))
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


        //Drive to pole
        drive.followTrajectory(startToPole);
        sleep(2000);

        //Align the robot and get new position
        align();
        sleep(1000);

        relocalizedPose = relocalize();
        drive.setPoseEstimate(relocalizedPose);
        poleToCone = drive.trajectoryBuilder(relocalizedPose, true)
                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                .build();

        sleep(1000);

        //Drive to cones
        drive.followTrajectory(poleToCone);

        sleep(1000);
        cycle(drive);
        sleep(1000);
        cycle(drive);
        sleep(1000);
        cycle(drive);


        /* Wait until program ends */
        sleep(30000);
    }

    public void cycle(SampleMecanumDrive drive) {
        //Drive to pole
        drive.setPoseEstimate(new Pose2d(56, 12, Math.toRadians(180)));
        drive.followTrajectory(coneToPole);
        sleep(2000);

        //Align the robot and get new position
        align();
        sleep(1000);

        relocalizedPose = relocalize();
        drive.setPoseEstimate(relocalizedPose);
        poleToCone = drive.trajectoryBuilder(relocalizedPose, true)
                .splineTo(new Vector2d(56, 13), Math.toRadians(0))
                .build();

        //Drive to cones
        drive.followTrajectory(poleToCone);
    }

    //Get new position
    public Pose2d relocalize() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM)+sensorOffset;
        double angle = Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

        double x_offset = distance*Math.sin(Math.toRadians(angle));
        double y_offset = distance*Math.cos(Math.toRadians(angle));

        double finalX = 23.4 + x_offset/25.4;
        double finalY = y_offset/25.4;

        Pose2d finalPose = new Pose2d(finalX, finalY, Math.toRadians(270-angle));

        telemetry.addData("X Offset", x_offset);
        telemetry.addData("Y Offset", y_offset);
        telemetry.addData("Distance", distance);
        telemetry.update();

        return finalPose;
    }

    public void align() {
        while (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) < 375
                || poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) > 425 && opModeIsActive()) {

            telemetry.addData("Pole X", poleDetectionPipeline.poleX);
            telemetry.addData("Pole Y", poleDetectionPipeline.poleY);
            telemetry.addData("Pole Width", poleDetectionPipeline.poleWidth);
            telemetry.addData("Pole Center", (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0)));
            telemetry.update();
            if (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) < 375) {
                motorFrontRight.setVelocity(motorVelocity);
                motorBackRight.setVelocity(-motorVelocity);
                motorFrontLeft.setVelocity(-motorVelocity);
                motorBackLeft.setVelocity(motorVelocity);
            } else if (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) > 425) {
                motorFrontRight.setVelocity(-motorVelocity);
                motorBackRight.setVelocity(motorVelocity);
                motorFrontLeft.setVelocity(motorVelocity);
                motorBackLeft.setVelocity(-motorVelocity);
            }
        }
        motorFrontRight.setVelocity(0);
        motorBackRight.setVelocity(0);
        motorFrontLeft.setVelocity(0);
        motorBackLeft.setVelocity(0);

        while (distanceSensor.getDistance(DistanceUnit.MM) < 85 || distanceSensor.getDistance(DistanceUnit.MM) > 95 && opModeIsActive()) {
            if (distanceSensor.getDistance(DistanceUnit.MM) < 85) {
                motorFrontRight.setVelocity(-motorVelocity);
                motorBackRight.setVelocity(-motorVelocity);
                motorFrontLeft.setVelocity(-motorVelocity);
                motorBackLeft.setVelocity(-motorVelocity);
            } else if (distanceSensor.getDistance(DistanceUnit.MM) > 95) {
                motorFrontRight.setVelocity(motorVelocity);
                motorBackRight.setVelocity(motorVelocity);
                motorFrontLeft.setVelocity(motorVelocity);
                motorBackLeft.setVelocity(motorVelocity);
            }
            telemetry.addData("Distance sensor", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        motorFrontRight.setVelocity(0);
        motorBackRight.setVelocity(0);
        motorFrontLeft.setVelocity(0);
        motorBackLeft.setVelocity(0);
    }





}