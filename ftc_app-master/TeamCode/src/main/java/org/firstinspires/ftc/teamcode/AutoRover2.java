package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*
Autonomous with Crater Only
 */
@Autonomous(name="AutoRoverLatch2", group="Linear Opmode")
//@Disabled
public class AutoRover2 extends LinearOpMode {

    private Servo hookServo = null;  //Team Marker Drop
    private DcMotor leftDrive = null;  //2 Wheel Drive
    private DcMotor rightDrive = null;  //2 Wheel Drive
    private DcMotor leftLift = null;  //2 Arm Lift
    private DcMotor rightLift = null;  // 2 Arm Lift

    @Override
    public void runOpMode() {
        hookServo = hardwareMap.get(Servo.class, "hookServo");
        //hookServo.setPosition(0);
        //2 Wheel Drive
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
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

        double liftPower = 0.5;

        while (opModeIsActive()) {
            //Drop from lander
            leftLift.setPower(liftPower);
            rightLift.setPower(liftPower);
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
            leftLift.setPower(liftPower);
            rightLift.setPower(liftPower);
            sleep( 750);

            leftLift.setPower(0);
            rightLift.setPower(0);

            //straighten robot
            leftDrive.setPower(liftPower);
            rightDrive.setPower(-liftPower);
            sleep(500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            /*
            Park in Crater
             */
            leftDrive.setPower(-0.5);
            rightDrive.setPower(-0.5);
            sleep(2000);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }



}
