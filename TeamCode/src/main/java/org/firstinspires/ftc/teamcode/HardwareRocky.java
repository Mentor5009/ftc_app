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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;
import org.firstinspires.ftc.teamcode.vision.RockyImu;
import org.firstinspires.ftc.teamcode.autonomous.IMUtest;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    //motors
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;
    public DcMotorEx lift = null;
    public DcMotorEx arm2 = null;
    public DcMotorEx arm = null;
    public DcMotorEx chickenFingers;
    public DcMotorEx upper = null;

    //servos
    public Servo marker = null;
    public Servo tilter = null;
    public Servo canadarmLeft = null;
    public Servo canadarmCentre = null;
    public Servo canadarmRight = null;
    public AnalogInput potentiometer;

    //Sensors
    public Rev2mDistanceSensor DS1;
    public Rev2mDistanceSensor DS2;
    public Rev2mDistanceSensor DS3;
    BNO055IMU imu;

    public boolean transportMode = false;

    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    //Constants
    private static final double DEGREES_PER_VOLT = -78.95;
    private static final double TILTER_DEEGRRES_PER_ARM_DEGREE = -0.00678;
    private static final double MAX_ARM_ANGLE = 225;
    private static final double MAX_SERVO_POSITION = 1;
    private static final double POSITION_UNIT_PER_DEGREE = 0.00444444;   //relates servo position to degrees
    private double tpr;

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

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

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

        canadarmLeft = hwMap.get(Servo.class, "canadarmLeft");
        canadarmCentre = hwMap.get(Servo.class, "canadarmCentre");
        canadarmRight = hwMap.get(Servo.class, "canadarmRight");

        potentiometer = hwMap.analogInput.get("potentiometer");

        imu = hwMap.get(BNO055IMU.class, "IMU1");

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        arm2.setPower(0);
        arm.setPower(0);
        chickenFingers.setPower(0);
        upper.setPower(0);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoders();



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


    //This is to drop from the lander with the upper during autonomous

    public void dropFromLander() {
        upper.setPower(0.9);
        om.telemetry.update();
        while (om.opModeIsActive() && upper.getCurrentPosition() < 17400) {
            om.telemetry.addData("Descending", upper.getCurrentPosition());
            om.telemetry.addData("op mode", om.opModeIsActive());
            om.telemetry.update();
            om.idle();
        }
        upper.setPower(0);
    }
    //This function is for moving the robot (tank control)

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


    // moveChih and moveChina is for slow mode

    public void moveChih(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void moveChina(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }


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

    public void pivotRight(double angle, double power) {
        double rads = angle * Math.PI / 180;
        double robotwidth = 17;
        double ticks = inchesToTicks(.5 * rads * robotwidth);
        resetEncoders();
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < 2*Math.abs(ticks)) {
            leftDrive.setPower(power);
            rightDrive.setPower(power*-.1);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void leftPivot(double angle, double power) {
        double rads = angle * Math.PI / 180;
        double robotwidth = 17;
        double ticks = inchesToTicks(.5 * rads * robotwidth);
        resetEncoders();
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() && Math.abs(rightDrive.getCurrentPosition()) < 2*Math.abs(ticks)) {
            rightDrive.setPower(power);
            leftDrive.setPower(power*-.1);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    //This function makes the upper move in autonomous

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

    //This functions makes the arm move in autonomous

    public void armMove(double angle, double power) {
        resetEncoders();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (om.opModeIsActive() && getArmAngle() > angle) {
            arm.setPower(power);
            arm2.setPower(power);
        }
    }

    //This stops every single function when the robot is stopped

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

    //These functions make the ATTS work
    //With the potentiometer we can get the arm angle
    public double getArmAngle() {
        double armAngle = (potentiometer.getVoltage()-1.09) * DEGREES_PER_VOLT + 135;
        om.telemetry.addData("arm angle", armAngle);
        return armAngle;
    }

    //With the arm angle we can control the tilter

    public double getTilterPosition() {
        double tilterPosition = TILTER_DEEGRRES_PER_ARM_DEGREE * getTilterAngle() + 0.75;
    return tilterPosition;}

    //This makes the finalizes the ATTS by moving at certain angles when the arm is at a certain angle

    public double getTilterAngle() {
        double tilterAngle = 145-getArmAngle();
    return tilterAngle;
    }
    public void gyroMove(double inches, double power) {

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        om.telemetry.addData("Mode", "calibrating...");
        om.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!om.isStopRequested() && !imu.isGyroCalibrated()) {
            om.sleep(50);
            om.idle();
        }

        om.telemetry.addData("Mode", "waiting for start");
        om.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        om.telemetry.update();

        // wait for start button.

        om.waitForStart();

        om. telemetry.addData("Mode", "running");
        om.telemetry.update();

        om.sleep(1000);

        // drive until end of period.

        // Use gyro to drive in a straight line.
        correction = checkDirection();

        om.telemetry.addData("1 imu heading", lastAngles.firstAngle);
        om.telemetry.addData("2 global heading", globalAngle);
        om.telemetry.addData("3 correction", correction);

        om.telemetry.update();
        double ticks = inchesToTicks(inches);
        resetEncoders();
        om.telemetry.addData("gyro ticks",ticks);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {
            om.telemetry.addData("left drive position",leftDrive.getCurrentPosition());
            om.telemetry.addData("right drive position",rightDrive.getCurrentPosition());
            om.telemetry.update();
            leftDrive.setPower(power + correction);
            rightDrive.setPower(power - correction);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getMoveAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .00001;

        angle = getMoveAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) {


        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double rads = degrees * Math.PI / 180;
        double robotwidth = 17;
        double ticks = inchesToTicks(.5 * rads * robotwidth);

        if (degrees < 0) {   // turn right.

            while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {
                leftDrive.setPower(-power);
                rightDrive.setPower(power);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        } else if (degrees > 0) {   // turn left.
            while (om.opModeIsActive() && Math.abs(leftDrive.getCurrentPosition()) < Math.abs(ticks) || Math.abs(rightDrive.getCurrentPosition()) < Math.abs(ticks)) {
                leftDrive.setPower(power);
                rightDrive.setPower(-power);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // set power to rotate.
            // rotate until turn is completed.
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (om.opModeIsActive() && getMoveAngle() == 0) {
                }

                while (om.opModeIsActive() && getMoveAngle() > degrees) {
                }
            } else    // left turn.
                while (om.opModeIsActive() && getMoveAngle() < degrees) {
                }

            // turn the motors off.
            rightDrive.setPower(0);
            leftDrive.setPower(0);

            // wait for rotation to stop.
            om.sleep(1000);

            // reset angle tracking on new heading.
            resetAngle();
        }

    }


}






