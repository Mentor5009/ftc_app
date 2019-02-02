package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Facing Crater With marker TFod End")

public class FacingCraterTfodEnd extends LinearOpMode {
    HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    public TFObjectDetector tfod;
    List<Recognition> updatedRecognitions;
    private VuforiaLocalizer vuforia;
    private String goldPos = "right";
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRocky();
        robot.init(hardwareMap, this);
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
        robot.upper.setPower(0.9);
        while (robot.upper.getCurrentPosition() < 17200 && opModeIsActive()) {
            telemetry.addData("going up", robot.upper.getCurrentPosition());
            telemetry.update();
        }
        robot.upper.setPower(0);

        robot.move(new Length(9, Length.Unit.INCH), -0.6, this); //reverse to  closer to sample for a better look

        // retract upper (descent arm) while scanning for the gold mineral position
        robot.upper.setPower(-0.9);
        telemetry.addData("Before", robot.upper.getCurrentPosition());
        telemetry.update();
        while (robot.upper.getCurrentPosition() > -12000 && opModeIsActive()) {
            goldPos = getGoldPos();
            telemetry.addData("goldpos", goldPos);
            telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
            telemetry.update();
        }
        robot.upper.setPower(0);

        if (goldPos == "left") {
            robot.pivot(55, 0.6, this); // turn toward gold
            robot.move(new Length(28, Length.Unit.INCH), -0.6, this); //reverse to gold and push through
            //after hitting sample
            robot.move(new Length(15, Length.Unit.INCH), 0.6, this);
            robot.pivot(35,.6,this);
            robot.move(new Length(44, Length.Unit.INCH), 0.9, this);
            //at wall
            robot.pivot(37, -0.6, this);
            //moves towards depot
            robot.move(new Length(50,Length.Unit.INCH), 0.9, this);
            robot.pivot(220, -0.7,this);
            robot.marker.setPosition(0.2);//Leave depot to go to crater
            robot.move(new Length(70, Length.Unit.INCH), 0.9, this);

            //robot.armMove(45,0.6);
        }
        if (goldPos == "right") {
            robot.pivot(54, -0.6, this); // turn toward gold
            robot.move(new Length(31, Length.Unit.INCH), -0.6, this);
            robot.pivot(120, 0.6, this);
            robot.move(new Length(70, Length.Unit.INCH), 0.6, this);
            //in depot
            robot.pivot(120, -0.6, this);
            robot.marker.setPosition(0.2);
            //Leave depot to go to crater
            robot.move(new Length(25, Length.Unit.INCH), 0.9, this);
            robot.move(new Length(25, Length.Unit.INCH), -0.9, this);
            robot.pivot(60,-.6,this);
            robot.move(new Length(62, Length.Unit.INCH), 0.9, this);



            //robot.armMove(45,0.6);
        }
        if (goldPos == "centre") {
            robot.move(new Length(21, Length.Unit.INCH), -0.6, this); //reverse to gold and push through to depot
            robot.move(new Length(15, Length.Unit.INCH), 0.6, this);
            //already hit sample
            robot.pivot(80, 0.6, this);
            robot.move(new Length(52, Length.Unit.INCH), 0.6, this);
            //moves towards depot
            robot.pivot(25, -0.6, this);
            robot.move(new Length(45, Length.Unit.INCH), 0.6, this);
            //in depot
            robot.pivot(40,-.8,this);
            robot.marker.setPosition(0.2);
            //Leave depot to go to crater
            robot.move(new Length(24, Length.Unit.INCH), 0.9, this);
            robot.move(new Length(24, Length.Unit.INCH), -0.9, this);
            robot.pivot(147.4,-8,this);
            robot.move(new Length(78, Length.Unit.INCH), 0.9, this);


            //robot.armMove(45,0.6);

            // shut down object detector
            if (tfod != null) {
                tfod.shutdown();
            }

        }
    }
    public void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(robot.TFOD_MODEL_ASSET, robot.LABEL_GOLD_MINERAL, robot.LABEL_SILVER_MINERAL);
    }


    public void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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


