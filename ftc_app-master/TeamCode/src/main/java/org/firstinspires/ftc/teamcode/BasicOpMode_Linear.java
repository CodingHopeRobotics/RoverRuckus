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

@TeleOp(name="RoverRuckus2", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    /*
    * 2 Wheel Drive
    */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    /*
    * 2 Arms to lift the robot
     */
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    /*
    * Intake Motor at the front of the arm
    */
    private DcMotor intakeMotor = null;

    /*
    * Servo to place the marker
    */
    private Servo hookServo = null;
    private double dumpPosition = 0;

    @Override
    public void runOpMode() {

        /*
        * Marker Servo
        */
        hookServo = hardwareMap.get(Servo.class, "hookServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
        * 2 Wheel Drive
        */
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        //invert
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
        * 2 Arm Lift
        */
        leftLift  = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        //invert
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        //leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        * Intake Motor
        */
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        /*
        * Wait for the game to start (driver presses PLAY)
         */
        waitForStart();
        runtime.reset();

        /*
        * run until the end of the match (driver presses STOP)
        */
        while (opModeIsActive()) {

            /*
            * Wheel Mechanics
            */
            double leftPower;
            double rightPower;

            leftPower  = gamepad1.left_stick_y * 3 / 4;
            rightPower = gamepad1.right_stick_y * 3 /4;
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            /*
            * Intake Motor Mechanics
            */
            if(gamepad1.dpad_up){//out
                intakeMotor.setPower(0.3);
            }else{
                if(gamepad1.dpad_down){//in
                    intakeMotor.setPower(-0.3);
                }else{
                    intakeMotor.setPower(0);
                }
            }

            /*
            * 2 Arm Lift Mechanics
            */
            if(gamepad1.left_trigger == 1){//up
                leftLift.setPower(0.5);
                rightLift.setPower(0.5);
            }else{
                if(gamepad1.right_trigger==1){
                    leftLift.setPower(-0.5);
                    rightLift.setPower(-0.5);
                }else{
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                }
            }

            /*
             Show the elapsed game time and wheel power.
              */
            telemetry.addData("Lift Position", "Lift is at" + leftLift.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
