package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareRocky;
import org.firstinspires.ftc.teamcode.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.vision.MineralPosition;
import org.firstinspires.ftc.teamcode.vision.RockyImu;


@Autonomous(name= "IMUtest")
public class IMUtest extends LinearOpMode {
    private HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldDetector goldDetector;
    private rotate 

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






}}
