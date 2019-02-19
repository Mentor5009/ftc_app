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
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;


@Autonomous(name = "Depot")
public class FacingDepot extends LinearOpMode {
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

        switch (goldPos) {
            case LEFT:
                //this at the end
                robot.pivot(57, 0.6); // turn toward gold
                robot.move(38, -0.6); //reverse to gold and push through
                robot.pivot(100, -0.6); // turn toward depot
                robot.move(43, -0.6); // reverse into depot
                robot.marker.setPosition(0.2); // drop marker
                robot.pivot(100, 0.6); // turn toward crater
                robot.move(62, 0.6); // forward to crater
                robot.armMove(10, 0.5);//robot.armMove(45, 0.6); // rotate arm over crater
                break;
            case RIGHT:
                robot.pivot(53, -0.6); // turn toward gold
                robot.move(38, -0.6); //reverse to gold and push through
                robot.pivot(107, .6);  // turn toward depot
                robot.move(44, -0.6); // reverse to depot
                //robot.pivot(10, -.6, this); // turn toward crater
                robot.marker.setPosition(0.2); // drop marker
                robot.move(62, 0.6); // forward to crater
                robot.armMove(10, 0.5); //robot.armMove(45, 0.6); // rotate arm over crater
                break;
            case CENTRE:
                robot.move(49, -0.6); //reverse to gold and push through to depot
                robot.pivot(62, 0.6); // turn toward crater
                robot.marker.setPosition(0.2); // drop marker
                robot.move(62, 0.6); // forward to crater
                robot.armMove(10, 0.5);//robot.armMove(45, 0.6); // rotate arm over crater
                break;
        }

        goldDetector.shutdown();
    }
}
    


