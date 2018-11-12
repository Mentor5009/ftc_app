package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Facing Depot")
public class RockyAutonomous extends LinearOpMode {
    HardwareRocky robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRocky();

        robot.init(hardwareMap);
        waitForStart();
        //runtime.reset();

        //robot.pivot(5,0.6);
        //robot.move(new Length( -2,Length.Unit.INCH),0.6);
        robot.pivot(90, 0.6);
        //robot.move(new Length(48,Length.Unit.INCH),.6);
        //robot.marker.setPosition(0.2);
        //robot.marker.setPosition(0.6);
        //robot.pivot(135, .6);
        //robot.move(new Length( 74,Length.Unit.INCH), .6);
    }


}
