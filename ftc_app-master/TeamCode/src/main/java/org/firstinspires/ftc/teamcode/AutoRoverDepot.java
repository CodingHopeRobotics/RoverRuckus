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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
Autonomous with Depot and Crater
 */
@Autonomous(name="AutoRoverDepot", group="Linear Opmode")
//@Disabled
public class AutoRoverDepot extends LinearOpMode{

    private Servo hookServo = null;  //Team Marker Drop
    private DcMotor leftDrive = null;  //2 Wheel Drive
    private DcMotor rightDrive = null;  //2 Wheel Drive
    private DcMotor leftLift = null;  //2 Arm Lift
    private DcMotor rightLift = null;  // 2 Arm Lift

    @Override
    public void runOpMode() {

        //Marker Servo
        hookServo = hardwareMap.get(Servo.class, "hookServo");
        //2 Wheel Drive
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        //invert wheel motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //2 Arm Lift
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        //invert lift
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        hookServo.setPosition(0);

        double liftPower = 0.2;
        double drivePower = 0.5;
        double turnPower = 0.2;

        while (opModeIsActive()) {
            /*
            Claim Depot
             */
            //raise lift
            leftLift.setPower(liftPower);
            rightLift.setPower(liftPower);
            sleep(500);

            //drive forward
            leftDrive.setPower(-drivePower);
            rightDrive.setPower(-drivePower);
            sleep(1500);

            //stop
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(2000);

            //drop Marker
            hookServo.setPosition(1.0);
            sleep(1000);
            hookServo.setPosition(0);
            sleep(1000);

            /*
            Park in crater
             */
            leftDrive.setPower(-turnPower);
            sleep(1000);
            leftDrive.setPower(0);

            leftDrive.setPower(turnPower);
            rightDrive.setPower(turnPower);
            sleep(1000);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(500);

            leftDrive.setPower(-turnPower);
            rightDrive.setPower(turnPower);
            sleep(1000);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(500);

            //drive to crater
            leftDrive.setPower(drivePower);
            rightDrive.setPower(drivePower);
            sleep(3000);

            //stop
            leftDrive.setPower(0);
            rightDrive.setPower(0);

        }
    }
}
