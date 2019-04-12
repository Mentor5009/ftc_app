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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.Telemetry;


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
    private final double WHEEL_DIAMETER = 4;

    /* Public OpMode members. */
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;
    public DcMotorEx lift = null;
    public DcMotorEx arm2 = null;
    public DcMotorEx arm = null;
    public Servo marker = null;
    public Servo tilter = null;
    //public Servo bigboi = null;
    public AnalogInput potentiometer;
    //public DistanceSensor distance;
    public DcMotorEx chickenFingers;
    public DcMotorEx upper = null;
    public boolean transportMode = false;
    private double tpr;
    private static final double DEGREES_PER_VOLT = -125;
    private static final double TILTER_DEEGRRES_PER_ARM_DEGREE = -0.00678;
    private static final double MAX_ARM_ANGLE = 225;
    private static final double MAX_SERVO_POSITION = 1;
    private static final double POSITION_UNIT_PER_DEGREE = 0.00444444;   //relates servo position to degrees

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private LinearOpMode om;

    /* Constructor */
    public HardwareRocky(LinearOpMode opMode) {
        om = opMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Servos
        marker = hwMap.get(Servo.class, "marker");
        chickenFingers = hwMap.get(DcMotorEx.class, " chickenFingers");
        //bigboi = hwMap.get(Servo.class, "bigboi");
        // Define and Initialize Motors
        leftDrive = (DcMotorEx) hwMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = (DcMotorEx) hwMap.get(DcMotorEx.class, "rightDrive");
        lift = (DcMotorEx) hwMap.get(DcMotorEx.class, "lift");
        arm2 = (DcMotorEx) hwMap.get(DcMotorEx.class, "arm2");
        arm = (DcMotorEx) hwMap.get(DcMotorEx.class, "arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        upper = (DcMotorEx) hwMap.get(DcMotorEx.class, "upper");
        tilter = hwMap.get(Servo.class, "tilter");

        potentiometer = hwMap.analogInput.get("potentiometer");
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        arm2.setPower(0);
        arm.setPower(0);
        chickenFingers.setPower(0);
        upper.setPower(0);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        resetEncoders();

        //set position of servos
        /*marker.setPosition(0.8);
        while (om.opModeIsActive() && marker.getPosition() < 0.8) om.sleep(50);*/

        tpr = 1066;
    }


    private void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void dropFromLander() {
        upper.setPower(0.9);
        om.telemetry.update();
        while (om.opModeIsActive() && upper.getCurrentPosition() < 17400) {
            om.telemetry.addData("going up", upper.getCurrentPosition());
            om.telemetry.addData("op mode", om.opModeIsActive());
            om.telemetry.update();
            om.idle();
        }

        upper.setPower(0);
        move(12, -.6);

       // move(9, -0.6); //reverse to  closer to sample for a better look
    }

    public void move(double inches, double power) {
        //tpr = leftDrive.getMotorType().getTicksPerRev();
        double ticks = inchesToTicks(inches);
        resetEncoders();

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {
            leftDrive.setPower(power);
            rightDrive.setPower(power);
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

//    public void stop() {
//        leftDrive.setPower(0);
//        rightDrive.setPower(0);
//    }

    //Robot pivots towards the crater from the depot
    public void pivot(double angle, double power) {
        double rads = angle * Math.PI / 180;
        double robotwidth = 17;
        double ticks = inchesToTicks(.5 * rads * robotwidth);
        resetEncoders();
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {
            rightDrive.setPower(-power);
            leftDrive.setPower(power);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void rightpivot(double angle, double power) {
        double rads = angle * Math.PI / 180;
        double robotwidth = 34;
        double ticks = inchesToTicks(.5 * rads * robotwidth);
        resetEncoders();
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() &&  Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {
            rightDrive.setPower(power);
            leftDrive.setPower(0);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void leftpivot(double angle, double power) {
        double rads = angle * Math.PI / 180;
        double robotwidth = 34;
        double ticks = inchesToTicks(.5 * rads * robotwidth);
        resetEncoders();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks)) {
        rightDrive.setPower(0);
            leftDrive.setPower(power);
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

        while (om.opModeIsActive() && Math.abs(lift.getCurrentPosition()) < Math.abs(ticks)) {
            om.sleep(50);
        }
        lift.setPower(0);
    }

    public void armMove(double angle, double power) {
        resetEncoders();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (om.opModeIsActive() && getArmAngle() > angle) {
            arm.setPower(power);
            arm2.setPower(power);
        }
    }

    public void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        arm.setPower(0);
        chickenFingers.setPower(0);
        upper.setPower(0);

    }

    public double liftInchesToTicks(double liftInches) {
        return (2132 * liftInches) / 2.25;
    }

    public double inchesToTicks(double inches) {
        return (inches * tpr) / (WHEEL_DIAMETER * Math.PI);
    }

    public double armDegreesToTicks(double armDegrees) {
        return (tpr * armDegrees) / 120;
    }

    public void chickenspin(double power) {
        chickenFingers.setPower(power);
    }

    public double getArmAngle() {
        double armAngle = potentiometer.getVoltage() * DEGREES_PER_VOLT + 135;
        om.telemetry.addData("arm angle", armAngle);
        return armAngle;
    }

    public double getTilterPosition() {
        double tilterPosition = TILTER_DEEGRRES_PER_ARM_DEGREE * getTilterAngle() + 0.75;
    return tilterPosition;}

    public double getTilterAngle() {
        double tilterAngle = 135-getArmAngle();
    return tilterAngle;
    }

    }

    /*public double calculateNewBigBoiPosition() {
        double cradleAngle = MAX_ARM_ANGLE - getArmAngle();
        return MAX_SERVO_POSITION - (cradleAngle * POSITION_UNIT_PER_DEGREE);
    }*/

    /*public void setCradleAngle(){
        bigboi.setPosition(calculateNewBigBoiPosition());
        om.telemetry.addData("bigboi pos ", calculateNewBigBoiPosition() );*/



