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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

@TeleOp(name="RockyTeleOpEmergency", group="Rocky")
//@Disabled
public class RockyTeleOpEmergency extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRocky robot = new HardwareRocky(this);     // Use a Rocky's hardware

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello 5009 team member");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;


            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            /*if ((left) > 0 && (right) > 0) {
                robot.DpadMove(1);

            } else if ((left) > 0 && (right) < 0){
                robot.DpadPivot(1);

            } else if ((left) < 0 && (right) > 0){
                robot.DpadPivot(-0.4);

            } else if ((left) < 0 && (right) < 0){
                robot.DpadMove(-1);

            } else if(gamepad1.dpad_up) {
                robot.DpadMove(0.4);

            } else if (gamepad1.dpad_down) {
                robot.DpadMove(-0.4);

            } else if(gamepad1.dpad_left) {
                robot.DpadPivot(-0.4);

            } else if (gamepad1.dpad_right) {
                robot.DpadPivot(0.4);

            }    else{
                robot.DpadMove(0);
            }*/

            //TODO: "GUYS, DON'T FORGET YOUR CURLY BRACKETS!"  - Chih-Hung
            /*Imagine yourself as a super powerful GOD/GODDESS
             and you're creating the functions of a brand new metallic creature
             and you need to create EVERY PART of it
             Example:
             if (Something happens){
                Something responds to such thing that happened
             }else  <-- else means if the "if" didn't happen then everything else
             will just fall to this statement{
                Something.Something;
             }

             E.g.:

              if (Chih-Hung eats food){
                Chih-Hung.stomach.StartsDigestion;
             }else{
                Chih-Hung.stomach.Starves;
                Chih-Hung.brain.sendNeurotransmitters.Hungry;
                Chih-Hung.brain.setMood.Grumpy;
                Chih-Hung.cells.startDying;
             }

             OR if you prefer Physics
             ("For every action, there is an equal and opposite reaction"-Newton)
             if(Newton.hitsCar){
                Car.action.react.hitNewtonBack;
                Customer.stare.atCar;
                Customer.stare.atNewton;
                Newton.react.surprisedPikachuMeme;
             }else{
                Car.action.nothing;
             }*/

            if(gamepad1.b) {
                robot.marker.setPosition(0.2);
            }else{
                robot.marker.setPosition(0.6);
            }

            if(gamepad1.right_bumper){
                robot.lift.setPower(-1.0);
            }
            else if(gamepad1.left_bumper){
                robot.lift.setPower(1.0);
            }
            else{
                robot.lift.setPower(0);
            }

            robot.arm.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            {
                //This is to make sure that the arm does not slam onto the ground or the lander.
                /*if ((robot.arm.getCurrentPosition() <= 99) || (robot.arm.getCurrentPosition() >= 99999)){
                     //The arm will slow down to a power of 50% when the arm reaches lower or higher then the encoder value.
                    robot.arm.setPower(0.5);
                }
            }*/


            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("marker",robot.marker.getPosition());
            telemetry.addData("lift", robot.lift.getPower());
            telemetry.addData("arm", robot.arm.getPower());
            telemetry.addData("arm",robot.arm.getCurrentPosition());
            //telemetry.addData("potentiometer",robot.potentiometer.getVoltage());
            telemetry.update();

            //TODO: CURLY BRACKETS!!!!!!
            // set power for chicken fingers
            //The chicken fingers will pick up with the "a" button, push out "x" and will stop with "b".
            if(gamepad1.a){
                //robot.chickenFingers.setPower(0.7);

            }
            if (gamepad1.x){
                //robot.chickenFingers.setPower(-0.7);
                }
            if (gamepad1.b){
               // robot.chickenFingers.setPower(0);
                }
                /*if(robot.potentiometer.getVoltage() <= 0.445)
                {robot.arm.setPower(0);
                }

                if(robot.potentiometer.getVoltage() >= 1.304)
                {robot.arm.setPower(-0.2);
                }
                if(gamepad1.x) {robot.arm.setTargetPosition(-152);
                }*/



            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
    }
}


