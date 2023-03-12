package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.CombinedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous
public class BlueAutoLeft2 extends LinearOpMode
{
    OpenCvCamera camera;
    Trajectory startToPole;
    TrajectorySequence poleToDrop;
    TrajectorySequence dropToCone;
    Trajectory coneToPole;

    CombinedPipeline combinedPipeline;

    public DcMotorEx motorFrontLeft = null;
    public DcMotorEx motorFrontRight = null;
    public DcMotorEx motorBackLeft = null;
    public DcMotorEx motorBackRight = null;

    public DcMotorEx slideL = null;
    public DcMotorEx slideR = null;

    private DistanceSensor distanceSensor = null;

    private int motorVelocity;

    private double sensorOffset;

    private Pose2d relocalizedPose;

    private Vector2d conePosition;

    private BNO055IMU imu;

    private enum State {
        preload_cone,
        stack_cone_0,
        stack_cone_1,
        IDLE,

    }

    public Servo servoL = null;
    public Servo servoR = null;

    public Servo v4bL = null;
    public Servo v4bR = null;


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

        sensorOffset = 266.7;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.035;


        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        combinedPipeline = new CombinedPipeline(tagsize, fx, fy, cx, cy);
        combinedPipeline.currentState = CombinedPipeline.TagOrPole.POLE;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36, 62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        conePosition = new Vector2d(56, 13);

        //Drive to the high goal from blue left starting pose
        startToPole = drive.trajectoryBuilder(startPose)
                .forward(40)
                .splineTo(new Vector2d(35, 10), Math.toRadians(215))
                .build();

        coneToPole = drive.trajectoryBuilder(new Pose2d(56, 12, Math.toRadians(180)), false)
                .forward(5)
                .splineTo(new Vector2d(35, 10), Math.toRadians(215))
                .build();

        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");



        v4bL = hardwareMap.get(Servo.class, "v4BL");
        v4bR = hardwareMap.get(Servo.class, "v4BR");
        v4bUp();

        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        closeClaw();

        camera.setPipeline(combinedPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        State currentState = State.preload_cone;

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pole X", combinedPipeline.poleX);
            telemetry.addData("Pole Y", combinedPipeline.poleY);
            telemetry.addData("Pole Width", combinedPipeline.poleWidth);
            telemetry.addData("Pole Center", (combinedPipeline.poleX + (combinedPipeline.poleWidth / 2.0)));
            telemetry.addData("Distance sensor", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();




        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        while (opModeIsActive()) {
            switch (currentState) {
                case IDLE:
                    sleep(30000);
                    break;
                case preload_cone:
                    //Drive to pole
                    drive.followTrajectory(startToPole);
                    sleep(2000);

                    //Align the robot and get new position
                    align();
                    sleep(1000);

                    relocalizedPose = relocalize();
                    drive.setPoseEstimate(relocalizedPose);

                    poleToDrop = drive.trajectorySequenceBuilder(relocalizedPose)
                            .back(2.5)
                            .build();

                    dropToCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .setReversed(true)
                            .waitSeconds(2)
                            .splineTo(new Vector2d(56.5, 14), Math.toRadians(0))
                            .build();

                    //Drive to cones
                    drive.followTrajectorySequence(poleToDrop);

                    v4bUp();
                    slideUp(2200);
                    openClaw();
                    sleep(500);
                    v4bDown();

                    currentState = State.IDLE;
                    break;
                case stack_cone_0:
//                poleToCone = drive.trajectoryBuilder(relocalizedPose, true)
//                        .splineTo(conePosition, 0)
//                        .build();
//
//                //Drive to cones
//                drive.followTrajectory(poleToCone);
//
//                pickup_stack_cone_0();
//
//                drive.followTrajectory(coneToPole);
//                sleep(2000);
//
//                //Align the robot and get new position
//                align();
//                sleep(1000);
//
//                relocalizedPose = relocalize();
//                drive.setPoseEstimate(relocalizedPose);
//
//                currentState = State.stack_cone_1;
                    break;
                case stack_cone_1:
                    break;
            }

        }

        /* Wait until program ends */
        sleep(30000);
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
        while ((combinedPipeline.poleX + (combinedPipeline.poleWidth / 2.0) < 385
                || combinedPipeline.poleX + (combinedPipeline.poleWidth / 2.0) > 415) && opModeIsActive()) {

            telemetry.addData("Pole X", combinedPipeline.poleX);
            telemetry.addData("Pole Y", combinedPipeline.poleY);
            telemetry.addData("Pole Width", combinedPipeline.poleWidth);
            telemetry.addData("Pole Center", (combinedPipeline.poleX + (combinedPipeline.poleWidth / 2.0)));
            telemetry.update();
            if (combinedPipeline.poleX + (combinedPipeline.poleWidth / 2.0) < 385) {
                motorFrontRight.setVelocity(motorVelocity);
                motorBackRight.setVelocity(-motorVelocity);
                motorFrontLeft.setVelocity(-motorVelocity);
                motorBackLeft.setVelocity(motorVelocity);
            } else if (combinedPipeline.poleX + (combinedPipeline.poleWidth / 2.0) > 415) {
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

        while ((distanceSensor.getDistance(DistanceUnit.MM) < 85 || distanceSensor.getDistance(DistanceUnit.MM) > 95) && opModeIsActive()) {
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

    public void pickup_stack_cone_0() {
        //Pickup the cone
    }

    public void openClaw() {
        servoL.setPosition(0.02);
        servoR.setPosition(0.25);
    }

    public void closeClaw() {
        servoL.setPosition(0.15);
        servoR.setPosition(0.1);
    }

    public void slideUp(int slideTarget) {
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        slideL.setTargetPosition(slideTarget);
        slideR.setTargetPosition(-slideTarget);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideL.setPower(0.95);
        slideR.setPower(-0.95);
        while (slideL.isBusy()) {
            telemetry.addData("Slide L position", slideL.getCurrentPosition());
            telemetry.addData("Slide L current", slideL.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    public void v4bUp() {
        v4bL.setPosition(0.93);
        v4bR.setPosition(0);
    }

    public void v4bDown() {
        v4bL.setPosition(0.2);
        v4bR.setPosition(0.73);
    }



}