package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.Length;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;


@Autonomous(name = "Depot")
public class FacingDepot extends LinearOpMode {
    private HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    public TFObjectDetector tfod;
    List<Recognition> updatedRecognitions;
    private VuforiaLocalizer vuforia;
    private String goldPos = "right";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRocky(this);
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        ElapsedTime t = new ElapsedTime();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        initVuforia();

        // initialize tensor flow
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        tfod.activate();
        goldPos = getGoldPos();

        /** Wait for the game to begin **/
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        // descend from lander
        robot.dropFromLander();

        // retract upper (descent arm) while scanning for the gold mineral position
        robot.upper.setPower(-0.9);
        telemetry.addData("Before", robot.upper.getCurrentPosition());
        telemetry.update();
        while (robot.upper.getCurrentPosition() > -16000 && opModeIsActive()) {
            goldPos = getGoldPos();
            telemetry.addData("goldpos", goldPos);
            telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
            telemetry.update();
        }
        robot.upper.setPower(0);

        //move based on gold position;
        if (opModeIsActive()) {
            if (goldPos == "left") {
                //this at the end
                robot.pivot(57, 0.6); // turn toward gold
                robot.move(new Length(38, Length.Unit.INCH), -0.6); //reverse to gold and push through
                robot.pivot(100, -0.6); // turn toward depot
                robot.move(new Length(43, Length.Unit.INCH), -0.6); // reverse into depot
                robot.marker.setPosition(0.2); // drop marker
                robot.pivot(100, 0.6); // turn toward crater
                robot.move(new Length(71, Length.Unit.INCH), 0.6); // forward to crater
                //robot.armMove(45, 0.6); // rotate arm over crater
            }
            if (goldPos == "right") {
                robot.pivot(53, -0.6); // turn toward gold
                robot.move(new Length(38, Length.Unit.INCH), -0.6); //reverse to gold and push through
                robot.pivot(107, .6);  // turn toward depot
                robot.move(new Length(44, Length.Unit.INCH), -0.6); // reverse to depot
                //robot.pivot(10, -.6, this); // turn toward crater
                robot.marker.setPosition(0.2); // drop marker
                robot.move(new Length(74, Length.Unit.INCH), 0.6); // forward to crater
                //robot.armMove(45, 0.6); // rotate arm over crater
            }
            if (goldPos == "centre") {
                robot.move(new Length(49, Length.Unit.INCH), -0.6); //reverse to gold and push through to depot
                robot.pivot(62, 0.6); // turn toward crater
                robot.marker.setPosition(0.2); // drop marker
                robot.move(new Length(70, Length.Unit.INCH), 0.6); // forward to crater
                //robot.armMove(45, 0.6); // rotate arm over crater

            }
        }
        // shut down object detector
        if (opModeIsActive() && tfod != null) {
            tfod.shutdown();
        }

    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(robot.TFOD_MODEL_ASSET, robot.LABEL_GOLD_MINERAL, robot.LABEL_SILVER_MINERAL);
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public String getGoldPos() {
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.milliseconds() < 4000) {
            if (tfod != null) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    int i = 1;

                    String rightMineral = "";
                    String centreMineral = "";
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("object " + String.valueOf(i), recognition.getLabel() + "," + recognition.getTop() + "," + recognition.getBottom());
                        i++;

                        if (recognition.getTop() < 600 && !rightMineral.equals("Silver Mineral")) {
                            rightMineral = recognition.getLabel();
                        } else if (recognition.getTop() >= 600 && recognition.getTop() <= 1000 && !centreMineral.equals("Silver Mineral")) {
                            centreMineral = recognition.getLabel();
                        }
                    }
                    telemetry.addData("rightMineral", rightMineral);
                    telemetry.addData("centreMineral", centreMineral);

                    if (!rightMineral.equals("Silver Mineral")) {
                        return "right";
                    } else if (!centreMineral.equals("Silver Mineral")) {
                        return "centre";
                    } else {
                        return "left";
                    }


                }

            }
        }
        return "right";
    }
}
    


