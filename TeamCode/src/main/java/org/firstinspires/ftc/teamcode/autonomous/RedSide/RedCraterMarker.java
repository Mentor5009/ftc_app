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

@Autonomous(name = "Red Crater")
public class CraterMarker extends LinearOpMode {
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
        robot.move(12, -0.6);


        // retract upper (descent arm) while scanning for the gold mineral position
        if (opModeIsActive()) {
        telemetry.addData("Before", robot.upper.getCurrentPosition());
        telemetry.update();
        while (opModeIsActive() && robot.upper.getCurrentPosition() > -12000) {
            robot.upper.setPower(-0.9);
            goldPos = goldDetector.getGoldPos(4000);
            telemetry.addData("goldpos", goldPos);
            telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
            telemetry.update();
        }
        robot.upper.setPower(0);

    switch (goldPos) {
        case LEFT: //hits both golds
            robot.canadarmLeft.setPosition(0.01);
            robot.canadarmLeft.setPosition(0.99);

            /*//after hitting sample
            robot.move(20, 0.6);
            robot.pivot(40, .6);
            robot.move(45, 0.9);
            // half way there
            robot.pivot(72, -0.6);
            //moves towards depot
            robot.move(56, 0.9);
            robot.pivot(270, -0.7);
            robot.marker.setPosition(0.2);//Leave depot to go to crater
            robot.move(67, 0.9);
            //robot.liftmove(7, 0.97);
            //robot.armMove(10, 0.5);
            //robot.armMove(45,0.6);
            //robot.armMove(45,0.6);*/
            break;
        case RIGHT:
            robot.canadarmRight.setPosition(.99);
            robot.canadarmRight.setPosition(.01);

            /*//after hitting sample
            robot.pivot(150, 0.6);
            robot.move(76, 0.6);
            //in depot
            robot.pivot(275, -0.6);
            robot.marker.setPosition(0.2);
            //Leave depot to go to crater
            robot.move(65, 0.9);
            //
            //robot.armMove(10, 0.5);
            //robot.armMove(45,0.6);*/
            break;
        case CENTRE:
            robot.canadarmCentre.setPosition(.01);
            robot.canadarmCentre.setPosition(.99);

            /*//already hit sample
            robot.pivot(95, 0.6);
            robot.move(54, 0.6);
            //moves towards depot
            robot.pivot(30, -0.6);
            robot.move(45, 0.6);
            //in depot
            robot.pivot(245, -.8);

            robot.marker.setPosition(0.2);
            //Leave depot to go to crater
            sleep(100);
            robot.move(71, 0.9);
            //Negative power goes towards 135, positive goes 0)
           // robot.liftmove(7, 0.97);
           // robot.armMove(10, 0.5);

            //robot.armMove(45,0.6);*/
            break;
    }

    goldDetector.shutdown();
}

    }
}

