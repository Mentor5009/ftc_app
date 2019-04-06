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

@Autonomous(name = "Gyro")
public class RockyimuGyro extends LinearOpMode {
    HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HardwareRocky(this);
        robot.init(hardwareMap);

        /** Wait for the game to begin **/
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robot.rotate(90,0.5);

        /*robot.gyroMove(30, .5);

        robot.gyroMove(30, .5);
        robot.rotate(90,0.5);
        robot.gyroMove(30, .5);
        robot.rotate(90,0.5);
        robot.gyroMove(30, .5);
        robot.rotate(90,0.5);*/
    }
}

