/*
Note: Control hub wifi password - 18568-Controlhub

 */

package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="V4B Calibrator", group="PowerPlay")
public class V4bCalibration extends LinearOpMode {

    public DcMotorEx slideL = null;
    public DcMotorEx slideR = null;

    public Servo servoL = null;
    public Servo servoR = null;

    public Servo v4bL = null;
    public Servo v4bR = null;

    private int slideTarget;
    private double v4bL_position;
    private double v4br_position;

    @Override
    public void runOpMode() {
        //Initialization Period

        // Define and Initialize Motors
        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");

        v4bL = hardwareMap.get(Servo.class, "v4BL");
        v4bR = hardwareMap.get(Servo.class, "v4BR");


        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        slideTarget = 0;

        v4bL_position = 0.5;
        v4br_position = 0.43;


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                slideTarget+=10;
                slideL.setPower(0.2);
                slideR.setPower(-0.2);
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                slideTarget-=10;
                slideL.setPower(0.2);
                slideR.setPower(-0.2);
                sleep(100);
            }

            //Open Claw
            if (gamepad1.x) {
                openClaw();

            }
            //Close Claw
            if (gamepad1.a) {
                closeClaw();
            }

            if (gamepad1.y) {
                v4bL_position+=0.05;
                v4br_position-=0.05;
                sleep(100);
            }
            if (gamepad1.b) {
                v4bL_position-=0.05;
                v4br_position+=0.05;
                sleep(100);
            }

            v4bL.setPosition(v4bL_position);
            v4bR.setPosition(v4br_position);

            slideL.setTargetPosition(slideTarget);
            slideR.setTargetPosition(-slideTarget);
            slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("slide position", slideTarget);
            telemetry.addData("v4b L", v4bL_position);
            telemetry.addData("v4b R", v4br_position);
            telemetry.update();
        }
    }

    public void openClaw() {
        servoL.setPosition(0.02);
        servoR.setPosition(0.25);
    }

    public void closeClaw() {
        servoL.setPosition(0.15);
        servoR.setPosition(0.1);
    }
}