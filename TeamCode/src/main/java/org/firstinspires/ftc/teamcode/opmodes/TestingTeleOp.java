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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Testing TeleOp", group="PowerPlay")
public class TestingTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  motorFrontLeft   = null;
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

    private int slideTarget;

    private double slowPower;
    private double drivePower;


    @Override
    public void runOpMode() {
        //Initialization Period

        // Define and Initialize Motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");

        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");

        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        openClaw();

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

        slowPower = 0.25;
        drivePower = 0.8;

        slideTarget = 0;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive
            if (gamepad1.right_trigger>0.7) {
                motorFrontRight.setPower(-slowPower);
                motorBackRight.setPower(-slowPower);
                motorFrontLeft.setPower(-slowPower);
                motorBackLeft.setPower(-slowPower);
            } else if (gamepad1.left_trigger>0.7) {
                motorFrontRight.setPower(slowPower);
                motorBackRight.setPower(slowPower);
                motorFrontLeft.setPower(slowPower);
                motorBackLeft.setPower(slowPower);
            } else if (gamepad1.right_bumper) {
                motorFrontLeft.setPower(-0.1);
                motorFrontRight.setPower(0.1);
                motorBackLeft.setPower(-0.1);
                motorBackRight.setPower(0.1);
            } else if (gamepad1.left_bumper) {
                motorFrontLeft.setPower(0.1);
                motorFrontRight.setPower(-0.1);
                motorBackLeft.setPower(0.1);
                motorBackRight.setPower(-0.1);
            } else {
                vertical = drivePower * gamepad1.left_stick_y;

                horizontal = -drivePower * gamepad1.left_stick_x;

                pivot = -drivePower * gamepad1.right_stick_x;

                motorFrontRight.setPower(vertical - pivot - horizontal);
                motorBackRight.setPower(vertical - pivot + horizontal);
                motorFrontLeft.setPower(vertical + pivot + horizontal);
                motorBackLeft.setPower(vertical + pivot - horizontal);
            }


            //Open Claw
            if (gamepad1.x) {
                openClaw();
            }
            //Close Claw
            if (gamepad1.a) {
                closeClaw();
            }


            if (!clawOpen) {
                //Raise for high goal
                if (gamepad1.dpad_up) {
                    slideTarget = 2650;
                    slideL.setPower(0.95);
                    slideR.setPower(-0.95);
                }
                //Raise for medium goal/low goal
                if (gamepad1.dpad_right) {
                    slideTarget = 1900;
                    slideL.setPower(0.95);
                    slideR.setPower(-0.95);
                }
                //Raise for ground junction
                if (gamepad1.y) {
                    slideTarget = 200;
                    slideL.setPower(0.95);
                    slideR.setPower(-0.95);
                }
            }
            if (gamepad1.dpad_down) {
                slideTarget = 5;
                slideL.setPower(-0.7);
                slideR.setPower(0.7);

            }

            slideL.setTargetPosition(slideTarget);
            slideR.setTargetPosition(-slideTarget);
            slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //Overextension failsafe
            if (slideL.getCurrentPosition() > 2750) {
                slideDown(1500, false);
            }

            //Anti-tipping failsafe
            if (slideL.getCurrentPosition()>1500) {
                telemetry.addData("IMU Value: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
                if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle<-4) {
                    slideDown(1500, false);
                }
            }




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
        slideL.setPower(0.95);
        slideR.setPower(-0.95);
        while (slideL.isBusy()) {
            telemetry.addData("Slide L position", slideL.getCurrentPosition());
            telemetry.addData("Slide L current", slideL.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    public void slideDown(int slideTarget, boolean openClaw) {
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        closeClaw();
        //Only delay if at low position to prevent snagging on sides of robot
        if (slideL.getCurrentPosition()<=500) {
            sleep(600);
        }
        slideL.setTargetPosition(slideTarget);
        slideR.setTargetPosition(-slideTarget);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideL.setPower(-0.7);
        slideR.setPower(0.7);
        while (slideL.isBusy()) {
            telemetry.addData("Slide L position", slideL.getCurrentPosition());
            telemetry.update();
        }
        if (openClaw) {
            openClaw();
        }
    }

    public void openClaw() {
        clawOpen = true;
        servoL.setPosition(0.02);
        servoR.setPosition(0.25);
    }

    public void closeClaw() {
        servoL.setPosition(0.15);
        servoR.setPosition(0.1);
        clawOpen = false;
    }
}
