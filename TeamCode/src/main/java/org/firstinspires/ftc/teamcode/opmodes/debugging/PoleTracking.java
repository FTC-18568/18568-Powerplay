package org.firstinspires.ftc.teamcode.opmodes.debugging;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class PoleTracking extends LinearOpMode
{
    OpenCvCamera camera;
    PoleDetectionPipeline poleDetectionPipeline;

    /* Declare OpMode members. */
    public DcMotorEx  motorFrontLeft   = null;
    public DcMotorEx  motorFrontRight  = null;
    public DcMotorEx  motorBackLeft    = null;
    public DcMotorEx  motorBackRight    = null;

    DistanceSensor distanceSensor = null;

    private int motorVelocity;


    @Override
    public void runOpMode()
    {
        // Define and Initialize Motors
        motorFrontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorVelocity = 300;

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        poleDetectionPipeline = new PoleDetectionPipeline();

        camera.setPipeline(poleDetectionPipeline);
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

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pole X", poleDetectionPipeline.poleX);
            telemetry.addData("Pole Y", poleDetectionPipeline.poleY);
            telemetry.addData("Pole Width", poleDetectionPipeline.poleWidth);
            telemetry.addData("Pole Center", (poleDetectionPipeline.poleX+(poleDetectionPipeline.poleWidth/2.0)));
            telemetry.addData("Distance sensor", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        }

        while(opModeIsActive()) {
            telemetry.addData("Pole X", poleDetectionPipeline.poleX);
            telemetry.addData("Pole Y", poleDetectionPipeline.poleY);
            telemetry.addData("Pole Width", poleDetectionPipeline.poleWidth);
            telemetry.addData("Pole Center", (poleDetectionPipeline.poleX+(poleDetectionPipeline.poleWidth/2.0)));
            telemetry.update();
            if (poleDetectionPipeline.poleX+(poleDetectionPipeline.poleWidth/2.0) < 350) {
                motorFrontRight.setVelocity(-motorVelocity);
                motorBackRight.setVelocity(motorVelocity);
                motorFrontLeft.setVelocity(motorVelocity);
                motorBackLeft.setVelocity(-motorVelocity);
            }
            else if (poleDetectionPipeline.poleX+(poleDetectionPipeline.poleWidth/2.0) > 450) {
                motorFrontRight.setVelocity(motorVelocity);
                motorBackRight.setVelocity(-motorVelocity);
                motorFrontLeft.setVelocity(-motorVelocity);
                motorBackLeft.setVelocity(motorVelocity);
            }
            else {
                motorFrontRight.setVelocity(0);
                motorBackRight.setVelocity(0);
                motorFrontLeft.setVelocity(0);
                motorBackLeft.setVelocity(0);
            }
        }

    }





}