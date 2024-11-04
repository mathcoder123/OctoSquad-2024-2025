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
//import Attachments;

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that  all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="HangTest", group="Iterative OpMode")
public class HangTest extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Attachments robot = new Attachments();
    public int pastCountL = 0;
    public int newCountL = 0;
    public int pastCountR = 0;
    public int newCountR = 0;
    public boolean hangOn = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.initialize(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
               // Controller 2
        // Hang
//        if (gamepad2.a) {
//            if (robot.leftLiftMotor.getVelocity() > 0) {
//                robot.rightLiftMotor.setPower(-1);
//                robot.leftLiftMotor.setPower(1); }
//            else {
//                robot.rightLiftMotor.setPower(0);
//                robot.leftLiftMotor.setPower(0);
//            }
//        }
//        if (gamepad1.a) {
//            robot.hang(1, Constants.hangLeftHigh, Constants.hangRightHigh);
//        }
//        else if (gamepad1.b) {
//            robot.hang(1, Constants.hangLeftDown, Constants.hangRightDown);
//        }
//        else if (gamepad1.right_trigger>0.02) {
//            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow); }
//
////         Hang Servos
//        if (gamepad2.left_bumper) {
//            robot.setLeftHangServo(Constants.hangLeftOpen);
//            robot.setRightHangServo(Constants.hangRightOpen);
//        }
//        else if (gamepad2.left_trigger>0.02){
//            robot.setLeftHangServo(Constants.hangLeftClosed);
//            robot.setRightHangServo(Constants.hangRightClosed);
//        }
//
//        telemetry.addData("leftHang", robot.getLeftLiftMotorPosition());
//        telemetry.addData("rightHang", robot.getRightLiftMotorPosition());

        newCountL = robot.getLeftLiftMotorPosition();
        newCountR = robot.getRightLiftMotorPosition();






        if (gamepad1.b) {

            hangOn = true;
            //if ((newCountL - pastCountL) > 0) {
                robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftLiftMotor.setPower(.6);
            //else {
              //  robot.setLeftHangServo(Constants.hangLeftOpen);
            //}

            //if ((newCountR - pastCountR) < 0) {
                robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightLiftMotor.setPower(-.6);
            //else {
              //  robot.setRightHangServo(Constants.hangRightOpen);
            //}
        }

        if (hangOn) {
            if(newCountL == pastCountL) {
                robot.leftLiftMotor.setPower(0);
                robot.rightLiftMotor.setPower(0);
            }

        }



        pastCountL = newCountL;
        pastCountR = newCountR;
        telemetry.addData("leftHang", robot.getLeftLiftMotorPosition());
        telemetry.addData("rightHang", robot.getRightLiftMotorPosition());
        telemetry.update();
    }
    @Override
    public void stop() {

    }
}
