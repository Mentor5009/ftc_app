package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomous Facing Depot")
public class RockyAutonomous extends LinearOpMode {
    HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRocky();
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        //robot.liftmove(4.90,0.6);
        //testing how to add tensor flow
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
                            robot.pivot(1, 0.6);
                            robot.pivot(3, 0.6);
                            robot.move(new Length(3, Length.Unit.INCH), -.6);
                            robot.pivot(4, -0.6);
                            robot.move(new Length(2, Length.Unit.INCH), -.6);
                            robot.pivot(3, 0.6);
                            robot.move(new Length(2, Length.Unit.INCH), -.6);
                            robot.armMove(45, 0.6);

                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            robot.pivot(3, -0.6);
                            robot.pivot(3, -0.6);
                            robot.move(new Length(3, Length.Unit.INCH), -.6);
                            robot.pivot(4, 0.6);
                            robot.move(new Length(2, Length.Unit.INCH), -.6);
                            robot.pivot(3, -0.6);
                            robot.move(new Length(2, Length.Unit.INCH), -.6);
                            robot.armMove(45, 0.6);
                        } else {
                            robot.pivot(1, 0.6);
                            robot.move(new Length(48, Length.Unit.INCH), -.6);
                            robot.pivot(45, 0.6);
                            robot.marker.setPosition(0.2);
                            TimeUnit.SECONDS.sleep(1);
                            robot.move(new Length(70, Length.Unit.INCH), .6);
                            robot.armMove(45, 0.6);

                        }

                    }

                    robot.pivot(1, 0.6);
       /*robot.move(new Length( 48,Length.Unit.INCH),-.6);
        robot.pivot(45, 0.6);
        robot.marker.setPosition(0.2);
        TimeUnit.SECONDS.sleep(1);
       // robot.pivot(135, .6);
        robot.move(new Length( 70,Length.Unit.INCH), .6);
        robot.armMove(45,0.6);*/


                }
            }
        }
    }
}
