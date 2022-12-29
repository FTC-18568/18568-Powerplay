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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PowerPlay Calibration", group="PowerPlay")
public class Calibration extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  motorFrontLeft   = null; //frontLeft backLeft
    public DcMotor  motorFrontRight  = null;
    public DcMotor  motorBackLeft    = null;
    public DcMotor  motorBackRight    = null;
    public Servo servoL = null;
    public Servo servoR = null;
    public DcMotorEx slideL = null;
    public DcMotorEx slideR = null;
    double currentVelocity;
    double maxVelocity;


    @Override
    public void runOpMode() {
        //Initialization Period

        // Define and Initialize Motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft"); //motorFrontLeft
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");

        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        currentVelocity = 0;
        maxVelocity = 0;



        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            currentVelocity = slideL.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }


            if (gamepad1.dpad_up) {
                slideL.setPower(0.8);
                slideR.setPower(-0.8);
            } else if (gamepad1.dpad_down) {
                slideL.setPower(-0.2);
                slideR.setPower(0.2);
            } else {
                slideL.setPower(0);
                slideR.setPower(0);
            }

            //open
            if (gamepad1.a) {
                servoL.setPosition(0.01);
            }

            //close
            if (gamepad1.x) {
                servoL.setPosition(0.17);
            }

            //open
            if (gamepad1.y) {
                servoR.setPosition(0.2);
            }

            //close
            if (gamepad1.b) {
                servoR.setPosition(0.05);
            }

            telemetry.addData("Max velocity", maxVelocity);
            telemetry.addData("current velocity", currentVelocity);
            telemetry.update();



        }
    }
}
