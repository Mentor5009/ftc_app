package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


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

@TeleOp(name = "k9unite", group = "Rocky")

public class k9unite extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor flagSpinner;


    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        flagSpinner = hardwareMap.get(DcMotor.class, "flagSpinner");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();



        waitForStart();
        while(opModeIsActive()) {

             double leftDrivePower;
             double rightDrivePower;
             double flagSpinnerPower;

            leftDrivePower = gamepad1.left_stick_y;
            rightDrivePower = -gamepad1.right_stick_y;

            if (gamepad1.a){flagSpinnerPower = 0.5;}

            else if (gamepad1.b){flagSpinnerPower = -0.5;}

            else flagSpinnerPower = 0;

            flagSpinner.setPower(flagSpinnerPower);
            leftDrive.setPower(leftDrivePower);
            rightDrive.setPower(rightDrivePower);
            telemetry.update();

            sleep(50);
        }
    }

}