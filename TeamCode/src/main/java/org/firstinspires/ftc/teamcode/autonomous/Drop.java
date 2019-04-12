package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;

@Autonomous(name = "drop")
public class Drop extends LinearOpMode {
    private HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldDetector goldDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        MineralPosition goldPos = MineralPosition.RIGHT;

        robot = new HardwareRocky(this);
        robot.init(hardwareMap);

        goldDetector = new GoldDetector (this);

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

        robot.canadarmLeft.setPosition(0.01);
        robot.canadarmCentre.setPosition(.01);
        robot.canadarmRight.setPosition(.99);

        sleep(50);

        robot.canadarmLeft.setPosition(0.99);
        robot.canadarmCentre.setPosition(.99);
        robot.canadarmRight.setPosition(.01);


        telemetry.addData("canArmLeft", robot.canadarmLeft.getPosition());
        telemetry.addData("canArmCentre", robot.canadarmCentre.getPosition());
        telemetry.addData("canArmRight", robot.canadarmRight.getPosition());

       // robot.pivot(90, .8);
       // telemetry.addData("Move Angle",robot.getMoveAngle());

        telemetry.update();
        while(opModeIsActive() && runtime.milliseconds() < 4000){}
    }
}



