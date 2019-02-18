package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.Length;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;

import java.util.List;

@Autonomous(name = "Crater no marker")
public class FacingCraterNoMarker extends LinearOpMode {
    HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldDetector goldDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        MineralPosition goldPos = MineralPosition.RIGHT;

        robot = new HardwareRocky(this);
        robot.init(hardwareMap);

        goldDetector = new GoldDetector(this);

        /* Wait for the game to begin */
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
        while (robot.upper.getCurrentPosition() > -16000 && opModeIsActive()) {
            goldPos = goldDetector.getGoldPos(4000);
            telemetry.addData("goldpos", goldPos);
            telemetry.addData("Not there yet", robot.upper.getCurrentPosition());
            telemetry.update();
        }
        robot.upper.setPower(0);

        switch(goldPos) {
            case LEFT:
                robot.pivot(55, 0.6); // turn toward gold
                robot.move(34, -0.6); //reverse to gold and push through
                robot.armMove(10, 0.5);
                //robot.armMove(45,0.6);
                break;
            case RIGHT:
                robot.pivot(54, -0.6); // turn toward gold
                robot.move(31, -0.6); //reverse to gold and push through
                robot.armMove(10, 0.5);

                //robot.armMove(45,0.6);
                break;
            case CENTRE:
                robot.move(28, -0.6);
                robot.armMove(10, 0.5);//reverse to gold and push through to depot
                //robot.armMove(45,0.6);
                break;
        }

        goldDetector.shutdown();
    }
}




