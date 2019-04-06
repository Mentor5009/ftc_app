package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;




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

@TeleOp(name = "k9Teleop", group = "Rocky")

public class k9Teleop extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor flagSpinner;
    //private Rev2mDistanceSensor sensorRange;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        flagSpinner = hardwareMap.get(DcMotor.class, "flagSpinner");
        //sensorRange = hardwareMap.get(Rev2mDistanceSensor.class,"distanceSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        //telemetry.addData(">>", "Press start to continue");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;



        telemetry.update();



        waitForStart();
        while(opModeIsActive()) {

             double leftDrivePower;
             double rightDrivePower;
             double flagSpinnerPower;

            leftDrivePower = gamepad1.left_stick_y;
            rightDrivePower = -gamepad1.right_stick_y;

            if (gamepad2.a){flagSpinner.setPower(0.5);}



            else if (gamepad2.b){flagSpinner.setPower(-0.5);}

            else flagSpinner.setPower(0);

            if (gamepad2.b){flagSpinner.setPower(-.5);}

            else flagSpinner.setPower(0);



            leftDrive.setPower(leftDrivePower);
            rightDrive.setPower(rightDrivePower);

            //telemetry.addData("deviceName",sensorRange.getDeviceName() );
            //telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.INCH));
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.update();

            sleep(50);
        }
    }

}