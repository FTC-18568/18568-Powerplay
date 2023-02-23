package org.firstinspires.ftc.teamcode.opmodes.debugging;

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
import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Relocalizer extends LinearOpMode {
    OpenCvCamera camera;
    PoleDetectionPipeline poleDetectionPipeline;

    /* Declare OpMode members. */
    public DcMotorEx motorFrontLeft = null;
    public DcMotorEx motorFrontRight = null;
    public DcMotorEx motorBackLeft = null;
    public DcMotorEx motorBackRight = null;

    private DistanceSensor distanceSensor = null;

    private int motorVelocity;

    private double sensorOffset;

    private BNO055IMU imu;


    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorVelocity = 300;

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");

        sensorOffset = 60.0;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        poleDetectionPipeline = new PoleDetectionPipeline();

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

            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */
        }

        sleep(10000);

        while (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) < 350
                || poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) > 450 && opModeIsActive()) {

            telemetry.addData("Pole X", poleDetectionPipeline.poleX);
            telemetry.addData("Pole Y", poleDetectionPipeline.poleY);
            telemetry.addData("Pole Width", poleDetectionPipeline.poleWidth);
            telemetry.addData("Pole Center", (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0)));
            telemetry.update();
            if (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) < 350) {
                motorFrontRight.setVelocity(-motorVelocity);
                motorBackRight.setVelocity(motorVelocity);
                motorFrontLeft.setVelocity(motorVelocity);
                motorBackLeft.setVelocity(-motorVelocity);
            } else if (poleDetectionPipeline.poleX + (poleDetectionPipeline.poleWidth / 2.0) > 450) {
                motorFrontRight.setVelocity(motorVelocity);
                motorBackRight.setVelocity(-motorVelocity);
                motorFrontLeft.setVelocity(-motorVelocity);
                motorBackLeft.setVelocity(motorVelocity);
            }
        }
        motorFrontRight.setVelocity(0);
        motorBackRight.setVelocity(0);
        motorFrontLeft.setVelocity(0);
        motorBackLeft.setVelocity(0);
        while (distanceSensor.getDistance(DistanceUnit.MM) < 395 || distanceSensor.getDistance(DistanceUnit.MM) > 405 && opModeIsActive()) {
            if (distanceSensor.getDistance(DistanceUnit.MM) < 395) {
                motorFrontRight.setVelocity(motorVelocity);
                motorBackRight.setVelocity(motorVelocity);
                motorFrontLeft.setVelocity(motorVelocity);
                motorBackLeft.setVelocity(motorVelocity);
            } else if (distanceSensor.getDistance(DistanceUnit.MM) > 405) {
                motorFrontRight.setVelocity(-motorVelocity);
                motorBackRight.setVelocity(-motorVelocity);
                motorFrontLeft.setVelocity(-motorVelocity);
                motorBackLeft.setVelocity(-motorVelocity);
            }
            telemetry.addData("Distance sensor", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        motorFrontRight.setVelocity(0);
        motorBackRight.setVelocity(0);
        motorFrontLeft.setVelocity(0);
        motorBackLeft.setVelocity(0);

        double distance = distanceSensor.getDistance(DistanceUnit.MM)-sensorOffset;
        double angle = Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

        double x_offset = distance*Math.sin(Math.toRadians(angle));
        double y_offset = distance*Math.cos(Math.toRadians(angle));

        telemetry.addData("X Offset", x_offset);
        telemetry.addData("Y Offset", y_offset);
        telemetry.update();


        sleep(30000);


    }


}



