/*
Note: Control hub wifi password - 18568-Controlhub

 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="PowerPlay TeleOp", group="PowerPlay")
public class PowerPlayTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  motorFrontLeft   = null; //frontLeft backLeft
    public DcMotor  motorFrontRight  = null;
    public DcMotor  motorBackLeft    = null;
    public DcMotor  motorBackRight    = null;

    public DcMotorEx slideL = null;
    public DcMotorEx slideR = null;

    public Servo servoL = null;
    public Servo servoR = null;


    private BNO055IMU imu;

    private boolean clawOpen;

    private double vertical;
    private double horizontal;
    private double pivot;


    @Override
    public void runOpMode() {
        //Initialization Period

        // Define and Initialize Motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft"); //motorFrontLeft
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");

        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");

        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        servoL.setPosition(0.0);
        servoR.setPosition(0.25);

        clawOpen = true;

        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Zero power behavior", slideL.getZeroPowerBehavior());

        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertical = 0;
        horizontal = 0;
        pivot = 0;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive
            if (gamepad1.right_trigger>0.7) {
                motorFrontRight.setPower(-0.1);
                motorBackRight.setPower(-0.1);
                motorFrontLeft.setPower(-0.1);
                motorBackLeft.setPower(-0.1);
            } else if (gamepad1.left_trigger>0.7) {
                motorFrontRight.setPower(0.1);
                motorBackRight.setPower(0.1);
                motorFrontLeft.setPower(0.1);
                motorBackLeft.setPower(0.1);
            } else if (gamepad1.right_bumper) {
                motorFrontLeft.setPower(-0.1);
                motorFrontRight.setPower(0.1);
                motorBackLeft.setPower(-0.1);
                motorBackRight.setPower(0.1);
            } else if (gamepad1.right_bumper) {
                motorFrontLeft.setPower(0.1);
                motorFrontRight.setPower(-0.1);
                motorBackLeft.setPower(0.1);
                motorBackRight.setPower(-0.1);
            } else {
                //Set vertical by threshold
                if (gamepad1.left_stick_y > -0.3 && gamepad1.left_stick_y < 0.3) {
                    vertical = 0.5 * gamepad1.left_stick_y;
                } else if (gamepad1.left_stick_y > -0.5 && gamepad1.left_stick_y < 0.5) {
                    vertical = 0.7 * gamepad1.left_stick_y;
                } else {
                    vertical = gamepad1.left_stick_y;
                }

                horizontal = -gamepad1.left_stick_x;

                //Set pivot by threshold
                if (gamepad1.right_stick_x > -0.3 && gamepad1.right_stick_x < 0.3) {
                    pivot = -0.3 * gamepad1.right_stick_x;
                } else if (gamepad1.right_stick_x > -0.5 && gamepad1.right_stick_x < 0.5) {
                    pivot = -0.5 * gamepad1.right_stick_x;
                } else {
                    pivot = -gamepad1.right_stick_x;
                }

                motorFrontRight.setPower(vertical - pivot - horizontal);
                motorBackRight.setPower(vertical - pivot + horizontal);
                motorFrontLeft.setPower(vertical + pivot + horizontal);
                motorBackLeft.setPower(vertical + pivot - horizontal);
            }




            telemetry.addData("pivot input: ", gamepad1.right_stick_x);

//            telemetry.addData("vertical: ", vertical);
//            telemetry.addData("horizontal: ", horizontal);
//            telemetry.addData("pivot: ", pivot);
//
//            telemetry.addData("frontLeft: ", vertical + pivot + horizontal);
//            telemetry.addData("frontRight: ", vertical - pivot - horizontal);
//            telemetry.addData("backLeft: ", vertical + pivot - horizontal);
//            telemetry.addData("backright: ", vertical - pivot + horizontal);
//
//            telemetry.update();


            //Open Claw
            if (gamepad1.x) {
                servoL.setPosition(0.0);
                servoR.setPosition(0.25);
                clawOpen = true;
            }
            //Close Claw
            if (gamepad1.a) {
                servoL.setPosition(0.15);
                servoR.setPosition(0.1);
                clawOpen = false;
            }


            //Raise for high goal
            if (gamepad1.dpad_up && !clawOpen) {
                slideUp(2650);
            }
            if (gamepad1.dpad_down) {
                slideDown(0);
            }
            //Raise for medium goal
            if (gamepad1.dpad_right && !clawOpen) {
                slideUp(2000);
            }
            //Raise for ground junction
            if (gamepad1.y && !clawOpen) {
                slideUp(200);
            }

            //Overextension failsafe
            if (slideL.getCurrentPosition() > 2750) {
                slideL.setPower(0);
                slideR.setPower(0);
            }

            //Anti-tipping failsafe
            if (slideL.getCurrentPosition()>1500) {
                telemetry.addData("IMU Value: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
                if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle<-4) {
                    slideDown(1500);
                }
            }


            //Print slide encoder data
//            telemetry.addData("Slide L: ", slideL.getCurrentPosition());
//            telemetry.addData("Slide R: ", slideR.getCurrentPosition());
//            telemetry.update();



        }
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
//        slideL.setVelocity(800);
//        slideR.setVelocity(-800);
        slideL.setPower(0.8);
        slideR.setPower(-0.8);
        while (slideL.isBusy()) {
            telemetry.addData("Slide L position", slideL.getCurrentPosition());
            telemetry.addData("Slide L velcoty", slideL.getVelocity());
            telemetry.update();
        }
    }

    public void slideDown(int slideTarget) {
        servoL.setPosition(0.17);
        servoR.setPosition(0.05);
        sleep(400);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        slideL.setTargetPosition(slideTarget);
        slideR.setTargetPosition(-slideTarget);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideL.setVelocity(-600);
//        slideR.setVelocity(600);
        slideL.setPower(-0.6);
        slideR.setPower(0.6);
        while (slideL.isBusy()) {
            telemetry.addData("Slide L position", slideL.getCurrentPosition());
            //telemetry.addData("Slide L velcoty", slideL.getVelocity());
            telemetry.update();
        }
        servoL.setPosition(0.02);
        servoR.setPosition(0.2);
        clawOpen = true;
    }
}
