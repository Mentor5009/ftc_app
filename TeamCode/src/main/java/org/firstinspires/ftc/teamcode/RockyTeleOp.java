/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RockyTeleOp: Telop Tank", group="Rocky")
//@Disabled
public class RockyTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRocky robot = new HardwareRocky();     // Use a Rocky's hardware

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;


            if (gamepad1.dpad_down) {
                robot.moveChih(- 0.25);
            }
            else if (gamepad1.dpad_up) {
                robot.moveChih(0.25);
            }
            else if (gamepad1.dpad_left){
                robot.moveChina(-.25);
            }
            else if (gamepad1.dpad_right) {
                    robot.moveChina(.25);
            }
            else {
                robot.leftDrive.setPower(left);
                robot.rightDrive.setPower(right);
            }

            robot.arm.setPower(gamepad2.left_trigger-gamepad2.right_trigger);

            if(gamepad2.dpad_up){
                robot.Tilter.setPosition(0.6);
            }
            else {
                robot.Tilter.setPosition(0.4);
            }

            if(gamepad1.x){
                robot.bigboi.setPosition(0.6);
            }
            else if (gamepad1.b){
                robot.bigboi.setPosition(0.4);
            }


            if(gamepad2.x){
                robot.chickenFingers.setPower(0.8); // set power for chicken fingers and position
            }
            else if(gamepad2.a){
                robot.chickenFingers.setPower(-0.8); // set power for chicken fingers reverse direction
            }
            else if(gamepad2.right_bumper){
                robot.lift.setPower(1.0);
            }
            else if(gamepad2.left_bumper){
                robot.lift.setPower(-1.0);

            }
            else {
                robot.chickenFingers.setPower(0);
                robot.lift.setPower(0);
            }
            if(gamepad2.y){
                robot.upper.setPower(1);
            }
            else if (gamepad2.b) {
                robot.upper.setPower(-1);
            }

            else {
                robot.upper.setPower(0);
            }


            /*while (gamepad2.dpad_right @@ robot.potentiometer.getVoltage()<0.853)*/



            //TELEMETRY ZONE
            telemetry.addData( "right bumper", gamepad2.right_bumper);
            telemetry.addData( "left bumper", gamepad2.left_bumper);
            telemetry.addData( "right trigger", gamepad2.right_trigger);
            telemetry.addData( "left trigger", gamepad2.left_trigger);

            telemetry.addData("upper encoder", robot.upper.getCurrentPosition());
            telemetry.addData("Tilter encoder", robot.Tilter.getPosition());
            telemetry.addData("potentiometer", robot.potentiometer.getVoltage());
            telemetry.addData("bigboi", robot.bigboi.getPosition());

            //telemetry.addData("left",  "%.2f", left);

            //telemetry.addData("right", "%.2f", right);
            //telemetry.addData("marker",robot.marker.getPosition());
            telemetry.addData("lift", robot.lift.getPower());
            telemetry.addData("lift", robot.lift.getCurrentPosition());
            telemetry.addData("arm", robot.arm.getPower());
            telemetry.addData("upper", robot.upper.getPower());
            telemetry.addData("arm encoder", robot.arm.getCurrentPosition());
            telemetry.addData("upper encoder", robot.upper.getCurrentPosition());
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);


        }
    }
}
