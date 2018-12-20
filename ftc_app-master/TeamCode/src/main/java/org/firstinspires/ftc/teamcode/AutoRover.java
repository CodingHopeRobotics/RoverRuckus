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


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/*
Autonomous with Depot and Crater
 */
@Autonomous(name="AutoRover", group="Linear Opmode")
//@Disabled
public class AutoRover extends LinearOpMode {

    private Servo hookServo = null;  //Team Marker Drop
    private DcMotor leftDrive = null;  //2 Wheel Drive
    private DcMotor rightDrive = null;  //2 Wheel Drive
    private DcMotor leftLift = null;  //2 Arm Lift
    private DcMotor rightLift = null;  // 2 Arm Lift

    @Override
    public void runOpMode() throws InterruptedException {

        hookServo = hardwareMap.get(Servo.class, "hookServo");
        //hookServo.setPosition(0);
        //2 Wheel Drive
        leftDrive  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        //invert
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //2 Arm Lift
        leftLift  = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        //invert
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            //Drop from lander
            leftLift.setPower(1);
            rightLift.setPower(1);
            sleep( 2000);
            leftLift.setPower(0);
            rightLift.setPower(0);
            /*
            Move from latch
             */
            //turn out from latch
            leftDrive.setPower(-0.5);
            rightDrive.setPower(0.5);
            sleep(500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(250);

            //Lower Arm
            leftLift.setPower(1);
            rightLift.setPower(1);
            sleep( 750);

            leftLift.setPower(0);
            rightLift.setPower(0);

            //straighten robot
            leftDrive.setPower(-0.5);
            rightDrive.setPower(-0.5);
            sleep(500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setPower(0.5);
            rightDrive.setPower(-0.5);
            sleep(500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            /*
            Claim Depot
             */
            leftDrive.setPower(-0.5);
            rightDrive.setPower(-0.5);

            sleep(2000);

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            hookServo.setPosition(1.0);
            sleep(1000);
            hookServo.setPosition(0);
            sleep(1000);

            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);

            sleep(500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            //Retract Lift Arm
            leftLift.setPower(1);
            rightLift.setPower(1);
            sleep( 2000);

            leftLift.setPower(0);
            rightLift.setPower(0);
            /*
            Park In Crater
             */





            break;
            //

            //sleep(2000);

            //hookServo.setPosit
            //
            // ion(0);
          }
    }
}
