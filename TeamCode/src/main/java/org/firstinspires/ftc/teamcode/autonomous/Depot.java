package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;


@Autonomous(name = "depot")
public class Depot extends LinearOpMode {
    private HardwareRocky robot;
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
        //runtime.reset();
        // descend from lander
        robot.dropFromLander();
        robot.move(9, -0.6);
        telemetry.update();
        // retract upper (descent arm) while scanning for the gold mineral position
        telemetry.addData("Before", robot.upper.getCurrentPosition());
        telemetry.update();

        if (opModeIsActive()) {
            telemetry.addData("Before", robot.upper.getCurrentPosition());
            telemetry.update();
            while (opModeIsActive() && robot.upper.getCurrentPosition() > -2724) {
                robot.upper.setPower(-0.9);
                goldPos = goldDetector.getGoldPos(4000);
                telemetry.addData("goldpos", goldPos);
                telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
                telemetry.update();
            }
            runtime.reset();
            robot.upper.setPower(0);
            while (opModeIsActive() && runtime.milliseconds() < 3000){
                goldPos = goldDetector.getGoldPos(4000);
                telemetry.addData("goldpos", goldPos);
                telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
                telemetry.update();
            }
            robot.move(8, -.6);

            switch (goldPos) {
                case LEFT:
                    robot.canadarmLeft.setPosition(0.01);
                    sleep(2500);
                    robot.canadarmLeft.setPosition(0.99);

                    moveFromDepot();
                    break;
                case RIGHT:

                    robot.canadarmRight.setPosition(.99);
                    sleep(2500);
                    robot.canadarmRight.setPosition(.01);

                    moveFromDepot();
                    /*robot.pivotRight(165,.8);
                    //moves to wall
                    robot.move(25, .8);
                    robot.pivot(40, .8);
                    robot.move(10, .8);
                    robot.pivot(180,0.8);
                    robot.move(14, -.8);
                    robot.marker.setPosition(0.99);
                    sleep(800);
                    robot.pivot(45,0.8);
                    robot.move(74, .8)*/


                    break;
                case CENTRE:
                    robot.canadarmCentre.setPosition(.01);
                    sleep(2500);
                    robot.canadarmCentre.setPosition(.99);

                    moveFromDepot();

                    break;
            }

            goldDetector.shutdown();
        }
    }

    public void moveFromDepot () {
        robot.leftPivot(85,.8);
        //moves to wall
        robot.move(55, -.8);
        robot.pivot(60, -.8);
        robot.move(62, 0.8);
        robot.pivot(255,0.8);
        sleep(800);
        robot.marker.setPosition(0.99);
        robot.move(60, .8);

    }
}

