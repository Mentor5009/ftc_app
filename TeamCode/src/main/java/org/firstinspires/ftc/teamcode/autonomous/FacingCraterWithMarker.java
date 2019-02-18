package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.Length;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;

import java.util.List;

@Autonomous(name = "Crater")

public class FacingCraterWithMarker extends LinearOpMode {
    HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldDetector goldDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        MineralPosition goldPos = MineralPosition.RIGHT;
        robot = new HardwareRocky(this);
        robot.init(hardwareMap);

        goldDetector = new GoldDetector(this);

        /** Wait for the game to begin **/
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // descend from lander
        robot.dropFromLander();

        // retract upper (descent arm) while scanning for the gold mineral position
        robot.upper.setPower(-0.9);
        telemetry.addData("Before", robot.upper.getCurrentPosition());
        telemetry.update();
        while (robot.upper.getCurrentPosition() > -12000 && opModeIsActive()) {
            goldPos = goldDetector.getGoldPos(4000);
            telemetry.addData("goldpos", goldPos);
            telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
            telemetry.update();
        }
        robot.upper.setPower(0);

        switch(goldPos) {
            case LEFT:
                robot.pivot(55, 0.6); // turn toward gold
                robot.move(25, -0.6); //reverse to gold and push through
                //after hitting sample
                robot.move(15, 0.6);
                robot.pivot(18,.6);
                robot.move(45, 0.9);
                //at wall
                robot.pivot(25, -0.6);
                //moves towards depot
                robot.move(50, 0.9);
                robot.pivot(220, -0.7);
                robot.marker.setPosition(0.2);//Leave depot to go to crater
                robot.move(70, 0.9);
                //robot.armMove(45,0.6);

                //robot.armMove(45,0.6);
                break;
            case RIGHT:
                robot.pivot(54, -0.6); // turn toward gold
                robot.move(31, -0.6);
                robot.pivot(120, 0.6);
                robot.move(70, 0.6);
                //in depot
                robot.pivot(180, -0.6);
                robot.marker.setPosition(0.2);
                //Leave depot to go to crater
                robot.move(62, 0.9);


                //robot.armMove(45,0.6);
                break;
            case CENTRE:
                robot.move(21, -0.6); //reverse to gold and push through to depot
                robot.move(15, 0.6);
                //already hit sample
                robot.pivot(80, 0.6);
                robot.move(52, 0.6);
                //moves towards depot
                robot.pivot(25, -0.6);
                robot.move(45, 0.6);
                //in depot
                robot.pivot(220,-.8);
                robot.marker.setPosition(0.2);
                //Leave depot to go to crater
                robot.move(70, 0.9);




                /*0.15 volts at lowest = 19.29 degrees
                1.2 volts at furthest back point = 154.32
                 */


                //robot.armMove(45,0.6);
                break;
        }

        goldDetector.shutdown();
    }
}


