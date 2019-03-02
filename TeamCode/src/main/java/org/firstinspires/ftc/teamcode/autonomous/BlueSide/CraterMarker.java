package org.firstinspires.ftc.teamcode.autonomous.BlueSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;

@Autonomous(name = "Blue Crater")

public class CraterMarker extends LinearOpMode {
    HardwareRocky robot;
    //private ElapsedTime runtime = new ElapsedTime();
    private GoldDetector goldDetector;
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()){
        runOpModeWhileActive();
        }
        robot.StopAll();
    }
        public void runOpModeWhileActive() throws InterruptedException {
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

        // retract upper (descent arm) while scanning for the gold mineral position

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

if (opModeIsActive()){
        switch(goldPos) {
            case LEFT:
                robot.pivot(55, 0.6); // turn toward gold
                robot.move(25, -0.6); //reverse to gold and push through
                //after hitting sample
                robot.move(16, 0.6);
                robot.pivot(30,.6); //20 red, 30 blue
                robot.move(45, 0.9);
                //at wall
                robot.pivot(25, -0.6);
                //moves towards depot
                robot.move(50, 0.9);
                robot.pivot(220, -0.7);
                robot.marker.setPosition(0.2);//Leave depot to go to crater
                robot.move(62, 0.9);
                //robot.liftmove(6, 0.7);
               // robot.armMove(10, 0.5);
                //robot.armMove(45,0.6);

                //robot.armMove(45,0.6);
                break;
            case RIGHT:
                robot.pivot(45, -0.6); // turn toward gold
                robot.move(26, -0.6);
                //after hitting sample
                robot.pivot(125, 0.6);
                robot.move(76, 0.6);
                //in depot
                robot.pivot(210, -0.6);
                robot.marker.setPosition(0.2);
                //Leave depot to go to crater
                robot.move(66, 0.9);
                //robot.liftmove(6, 0.7);
                //robot.armMove(10, 0.5);


                //robot.armMove(45,0.6);
                break;
            case CENTRE:
                robot.move(31, -0.6); //reverse to gold and push through to depot
                robot.move(15, 0.6);
                //already hit sample
                robot.pivot(79, 0.6);
                robot.move(52, 0.6);
                //moves towards depot
                robot.pivot(18, -0.6);
                robot.move(45, 0.6);
                //in depot
                robot.pivot(220,-.8);
                robot.marker.setPosition(0.2);
                //Leave depot to go to crater
                robot.move(62, 0.9);
               // robot.liftmove(6, 0.7);
                //Negative power goes towards 135, positive goes 0)
                //robot.armMove(10, 0.5);




                //robot.armMove(45,0.6);
                break;
        }}

        goldDetector.shutdown();

    }
}

