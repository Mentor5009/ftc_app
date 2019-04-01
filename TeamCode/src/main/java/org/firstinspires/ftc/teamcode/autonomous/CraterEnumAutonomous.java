package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;
import org.firstinspires.ftc.teamcode.autonomous.CraterEnum;


@Autonomous(name = "CraterEnumAutonomous")
public class CraterEnumAutonomous extends LinearOpMode {
    HardwareRocky robot;
    //private ElapsedTime runtime = new ElapsedTime();
    private GoldDetector goldDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        MineralPosition goldPos = MineralPosition.RIGHT;
        CraterEnum currentMode = CraterEnum.WaitForStart;
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

            robot.pivot(70, 0.6); // turn toward gold
            robot.move(18, -0.6); //reverse to gold and push through


            //after hitting sample
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
            //robot.armMove(45,0.6);
            break;
        case RIGHT:
            robot.pivot(68, -0.6); // turn toward gold
            robot.move(26, -0.6);
            //after hitting sample
            robot.pivot(150, 0.6);
            robot.move(76, 0.6);
            //in depot
            robot.pivot(275, -0.6);
            robot.marker.setPosition(0.2);
            //Leave depot to go to crater
            robot.move(65, 0.9);
            //
            //robot.armMove(10, 0.5);
            //robot.armMove(45,0.6);
            break;
        case CENTRE:
            robot.move(17, -0.6); //reverse to gold and push through to depot
            robot.move(14, 0.6);
            //already hit sample
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

            //robot.armMove(45,0.6);
            break;
    }

    goldDetector.shutdown();
}

    }
}

