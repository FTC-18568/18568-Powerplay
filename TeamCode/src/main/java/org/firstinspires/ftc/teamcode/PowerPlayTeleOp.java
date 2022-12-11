/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PowerPlay TeleOp", group="PowerPlay")
public class PowerPlayTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  motorFrontLeft   = null; //frontLeft backLeft
    public DcMotor  motorFrontRight  = null;
    public DcMotor  motorBackLeft    = null;
    public DcMotor  motorBackRight    = null;

    public DcMotor slideL = null;
    public DcMotor slideR = null;

    public Servo servoL = null;
    public Servo servoR = null;

    private double p;
    private double i;
    private double d;
    private double f;

    private boolean clawOpen;


    @Override
    public void runOpMode() {
        //Initialization Period

        // Define and Initialize Motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft"); //motorFrontLeft
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");

        slideL = hardwareMap.get(DcMotor.class, "slideL");
        slideR = hardwareMap.get(DcMotor.class, "slideR");

        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        servoL.setPosition(0.02);
        servoR.setPosition(0.2);

        clawOpen = true;

        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Zero power behavior", slideL.getZeroPowerBehavior());

        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        p = 1.622128713;
        i = 0.1*p;
        d = 0;
        f = p*10;

//        slideL.setVelocityPIDFCoefficients(p, i, d, f);
//        slideR.setVelocityPIDFCoefficients(p, i, d, f);
//
//        slideL.setPositionPIDFCoefficients(5.0);
//        slideR.setPositionPIDFCoefficients(5.0);



        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive
            double vertical = 0.7 * gamepad1.left_stick_y;
            double horizontal = -0.7 * gamepad1.left_stick_x;
            double pivot = -0.7 * gamepad1.right_stick_x;

            motorFrontRight.setPower(vertical - pivot - horizontal);
            motorBackRight.setPower(vertical - pivot + horizontal);
            motorFrontLeft.setPower(vertical + pivot + horizontal);
            motorBackLeft.setPower(vertical + pivot - horizontal);

            //Open Claw
            if (gamepad1.x) {
                servoL.setPosition(0.02);
                servoR.setPosition(0.2);
                clawOpen = true;
            }
            //Close Claw
            if (gamepad1.a) {
                servoL.setPosition(0.17);
                servoR.setPosition(0.05);
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
            if (slideL.getCurrentPosition()>2750) {
                slideL.setPower(0);
                slideR.setPower(0);
            }



            //Print slide encoder data
            telemetry.addData("Slide L: ", slideL.getCurrentPosition());
            telemetry.addData("Slide R: ", slideR.getCurrentPosition());
            telemetry.update();

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
        slideL.setPower(-0.8);
        while (slideL.isBusy()) {
            telemetry.addData("Slide L position", slideL.getCurrentPosition());
            //telemetry.addData("Slide L velcoty", slideL.getVelocity());
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
        slideL.setPower(0.6);
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
