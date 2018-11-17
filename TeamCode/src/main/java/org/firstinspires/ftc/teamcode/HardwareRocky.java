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

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
    public void move(Length d, double power) {
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
    public void pivot(double angle, double power) {
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
    // if something goes wrong delete everything below this
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
    @TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
//@Disabled
    public class ConceptTensorFlowObjectDetection extends LinearOpMode {
        private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
        private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        private static final String VUFORIA_KEY = "AfZdcpz/////AAAAGeFAEIQ7eEL9ilMArx0PrTpfGi14uY5DxJNi9A/pNhrpWpMLBsZIt21zn61HlpOEsX4SW/GyN//S+CJpHALNQkDftlrlJ3+cGtzrVC0ZZcEpltXAdp/5CkO+M7Q3rDOtKBeFhCBnjDUVswvmD0sU9mRgjVhn5TvOSXcSuJIJymIy5x15BUxbqsZe+5Rkzt4a/4ltQvr3jN13s4RECp03x+zfPWKR7S79x1+VSITaBB5lrv43p9ZEBJeIaWlAXQTST8O0uf2YhNXCuzrBxuAgL5onSpOWmBUzyFxE8cooXOgyktMm/mtYHG+vujg4gG9FpxFFLutypcN3hLaBOqfS1DyNrQD1i05cpRLwJ4M0Gszc";

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        private VuforiaLocalizer vuforia;

        /**
         * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
         * Detection engine.
         */
        private TFObjectDetector tfod;

        @Override
        public void runOpMode() {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }

                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }

        /**
         * Initialize the Vuforia localization engine.
         */
        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }

        /**
         * Initialize the Tensor Flow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }
    }

}


