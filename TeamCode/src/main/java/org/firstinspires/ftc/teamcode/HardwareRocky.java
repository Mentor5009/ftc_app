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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "leftDrive"
 * Motor channel:  Right drive motor:        "rightDrive"
 * <p>
 * Note: the configuration of the servos is such that:
 * As the arm servo approaches 0, the arm position moves up (away from the floor).
 * As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareRocky {

    final Length wheelDiamater = new Length(4, Length.Unit.INCH);
    /* Public OpMode members. */
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;
    public DcMotorEx lift = null;
    public DcMotorEx arm = null;
    public Servo marker = null;
    public DcMotorEx chickenFingers;
    public double tpr;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareRocky() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Servos
        marker = hwMap.get(Servo.class, "marker");

        // Define and Initialize Motors
        leftDrive = (DcMotorEx) hwMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = (DcMotorEx) hwMap.get(DcMotorEx.class, "rightDrive");
        lift = (DcMotorEx) hwMap.get(DcMotorEx.class, "lift");
        arm = (DcMotorEx) hwMap.get(DcMotorEx.class, "arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        chickenFingers = (DcMotorEx) hwMap.get(DcMotorEx.class, "chickenFingers");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        arm.setPower(0);
        chickenFingers.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chickenFingers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //set position of servos
        marker.setPosition(0.8);
        while (marker.getPosition() < 0.8) ;

        tpr = 1066;
    }

    void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //
    public void move(Length d, double power, OpMode om) {
        //tpr = leftDrive.getMotorType().getTicksPerRev();
        double ticks = inchesToTicks(d); //d.in(Length.Unit.INCH)*tpr / ((wheelDiamater.in(Length.Unit.INCH))* Math.PI);
        resetEncoders();

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        while (Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {

        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void moveChih(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void moveChina(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }

    //Robot pivots towards the crater from the depot
    public void pivot(double angle, double power, OpMode om) {
        double rads = angle * Math.PI / 180;
        double robotwidth = 17;
        double ticks = inchesToTicks(new Length(.5 * rads * robotwidth, Length.Unit.INCH));
        resetEncoders();
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setPower(-power);
        leftDrive.setPower(power);
        while (Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void liftmove(double inches, double power) {
        double ticks = liftInchesToTicks(inches);
        lift.setPower(power);
        resetEncoders();
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(lift.getCurrentPosition()) < Math.abs(ticks)) {

        }
        lift.setPower(0);
    }

    public void armMove(double angle, double power) {
        double ticks = armDegreesToTicks(angle);
        resetEncoders();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(arm.getCurrentPosition()) < Math.abs(ticks)) {

        }
        arm.setPower(0);
    }


    public double liftInchesToTicks(double liftInches){
        return (2132*liftInches)/2.25;
    }

    public double inchesToTicks(Length d) {
        return d.in(Length.Unit.INCH)*tpr / ((wheelDiamater.in(Length.Unit.INCH))* Math.PI);
    }
    public double armDegreesToTicks (double armDegrees){ return (tpr*armDegrees)/120;
    }

    public void chickenspin(double power) {
        chickenFingers.setPower(power);
    }


}

//delete everything under this if shit goes wrong.

public String getGoldPos (OpMode op) {

    if (tfod != null) {
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            op telemetry.addData("# Object Detected", updatedRecognitions.size());
            int i = 1;
            String leftMineral = "";
            String centreMineral = "";

            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData("object " + String.valueOf(i), recognition.getLabel() + "," + recognition.getTop() + "," + recognition.getBottom());
                i++;

                if (recognition.getTop() < 600 && leftMineral != "silver") {
                    leftMineral = recognition.getLabel();
                } else if (recognition.getTop() >= 600 && recognition.getTop() <= 1000 && centreMineral != "silver") {
                    centreMineral = recognition.getLabel();

                }


            }
            telemetry.addData("left min.", leftMineral);
            telemetry.addData("centre min.", centreMineral);
        }
        op 
    }
}
}

