package org.firstinspires.ftc.teamcode.autonomous.RedSide;

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

@Autonomous(name = "RedCraterMarker")
public class RedCraterMarker extends LinearOpMode {
    HardwareRocky robot;
    //private ElapsedTime runtime = new ElapsedTime();
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
        //runtime.reset();

        // descend from lander
        robot.dropFromLander();
        //Goes backward
        robot.move(9, -0.6);


        // retract upper (descent arm) while scanning for the gold mineral position
        if (opModeIsActive()) {
            telemetry.addData("Before", robot.upper.getCurrentPosition());
            telemetry.update();
            while (opModeIsActive() && robot.upper.getCurrentPosition() > -3800) {
                robot.upper.setPower(-0.9);
                goldPos = goldDetector.getGoldPos(4000);
                telemetry.addData("goldpos", goldPos);
                telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
                telemetry.update();
            }
            robot.upper.setPower(0);
            robot.move(8, -.6);

            switch (goldPos) {
                case LEFT: //hits both golds

                    robot.canadarmLeft.setPosition(0.01);
                    sleep(1500);
                    robot.canadarmLeft.setPosition(0.99);
                    sleep(1000);
                    moveFromCrater(true);
                    break;

                case RIGHT:

                    robot.canadarmRight.setPosition(.99);
                    sleep(2500);
                    robot.canadarmRight.setPosition(.01);
                    sleep(1000);
                    moveFromCrater(false);

                    break;
                case CENTRE:

                    robot.canadarmCentre.setPosition(.01);
                    sleep(2500);
                    robot.canadarmCentre.setPosition(.99);
                    sleep(1500);
                    moveFromCrater(false);

                    break;
            }
            goldDetector.shutdown();

        }

    }

    public void moveFromCrater(boolean isLeft) throws InterruptedException {
        robot.pivotRight(85,.8);
        //moves to wall
        robot.move(30,.8);
        robot.pivot(10,-.8);
        if (isLeft) {
            robot.canadarmLeft.setPosition(0.2);
        }
        robot.move(57,0.8);
        if (isLeft) {
            robot.canadarmLeft.setPosition(0.99);
        }
        robot.pivot(240,-0.8);
        robot.marker.setPosition(0.99);
        robot.move(70, .8);
    }
}
