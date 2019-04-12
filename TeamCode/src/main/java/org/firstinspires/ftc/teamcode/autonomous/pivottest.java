package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;
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

    @Autonomous(name = "pivot test")
    public class pivottest extends LinearOpMode {
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
            robot.leftpivot(90, 0.5);
            robot.rightpivot(90, 0.5);
        }
    }
