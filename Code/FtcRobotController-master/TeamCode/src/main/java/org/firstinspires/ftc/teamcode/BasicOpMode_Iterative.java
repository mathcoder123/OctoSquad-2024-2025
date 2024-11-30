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

@TeleOp(name="TeleOp1", group="Iterative OpMode")
public class BasicOpMode_Iterative extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Attachments robot = new Attachments();
    public int pastCountL = 0;
    public int newCountL = 0;
    public int pastCountR = 0;
    public int newCountR = 0;
    public boolean hangOn = false;
    public boolean clawOn = false;
    public boolean LHDone = false;
    public boolean RHDone = false;
    public boolean LoopDone = false;

    /*
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LoopDone = false;
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
        robot.setLeftHangServo(Constants.hangLeftClosed);
        robot.setRightHangServo(Constants.hangRightClosed);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        if (LoopDone) {
            robot.leftLiftMotor.setPower(.2);
            robot.rightLiftMotor.setPower(-.2);
            robot.LF.setPower(0);
            robot.LB.setPower(0);
            robot.RF.setPower(0);
            robot.RB.setPower(0);
            return;
        }
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x; // Left/right
        double rx = -gamepad1.right_stick_x; // Rotation

        // Calculate the motor power drive
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // DRIVE

        if (gamepad1.dpad_up) {
            robot.LF.setPower(.65);
            robot.LB.setPower(.65);
            robot.RF.setPower(.65);
            robot.RB.setPower(.65);
        }
        else if (gamepad1.dpad_down) {
            robot.LF.setPower(-.65);
            robot.LB.setPower(-.65);
            robot.RF.setPower(-.65);
            robot.RB.setPower(-.65);
        }
        else if (gamepad1.dpad_right) {
            robot.LF.setPower(.65);
            robot.LB.setPower(-.65);
            robot.RF.setPower(-.65);
            robot.RB.setPower(.65);
        }
        else if (gamepad1.dpad_left) {
            robot.LF.setPower(-.65);
            robot.LB.setPower(.65);
            robot.RF.setPower(.65);
            robot.RB.setPower(-.65);
        }
        else{
        robot.LF.setPower(frontLeftPower);
        robot.LB.setPower(backLeftPower);
        robot.RF.setPower(frontRightPower);
        robot.RB.setPower(backRightPower);
        }/*
         */


        // Vertical Slide - 15cm
        if (gamepad1.right_bumper) {
            robot.setVerticalLinear(1, Constants.verticalSlideHigh);
            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow);
        } else if (gamepad1.right_trigger>0.02) {
            robot.setVerticalLinear(1, Constants.verticalSlideLow);
            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow);
        } else if (gamepad1.left_bumper) {
            robot.setVerticalLinear(1, Constants.verticalSlideSubmersible);
            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow);
        } else if (gamepad1.left_trigger>0.02) {
            robot.setVerticalLinear(1, Constants.verticalSlideBasket);
            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow);
            clawOn = false;
        }
        else if (gamepad1.right_stick_button) {
            robot.setVerticalLinear(1, Constants.verticalSlidePickup);
            robot.hang(1, Constants.hangLeftPickup, Constants.hangRightPickup);
        }
        else if (gamepad1.left_stick_button) {
            robot.setVerticalLinear(1, Constants.verticalSlidePickupHigh);
            robot.hang(1, Constants.hangLeftPickup, Constants.hangRightPickup);
        }

        // Basket Servo
        if (gamepad1.y){
            robot.setBasketServo(Constants.basketOpen);
        }
        else {
            robot.setBasketServo(Constants.basketClosed);
        }

        if (gamepad1.x) {
            robot.setBackClawServo(Constants.backClawOpen);
        }
        else if (!clawOn){
            robot.setBackClawServo(Constants.backClawClose);
        }


        // Controller 2
        newCountL = robot.getLeftLiftMotorPosition();
        newCountR = robot.getRightLiftMotorPosition();

        if (gamepad1.a) {
            clawOn = true;
            robot.setBackClawServo(0.47);
            robot.leftLiftMotor.setPower(0);
            robot.rightLiftMotor.setPower(0);
            hangOn = false;
            robot.hang(1, Constants.hangLeftHigh, Constants.hangRightHigh);
            robot.setLeftHangServo(Constants.hangLeftClosed);
            robot.setRightHangServo(Constants.hangRightClosed);
        }
        else if (gamepad1.right_trigger>0.02) {
            hangOn = false;
            robot.setLeftHangServo(Constants.hangLeftClosed);
            robot.setRightHangServo(Constants.hangRightClosed);
            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow);
            robot.leftLiftMotor.setPower(0);
            robot.rightLiftMotor.setPower(0);
        }


//         Hang Servos
//        if (gamepad2.left_bumper) {
//            robot.setLeftHangServo(Constants.hangLeftOpen);
//            robot.setRightHangServo(Constants.hangRightOpen);
//        }
//        else if (gamepad2.left_trigger>0.02){
//            robot.setLeftHangServo(Constants.hangLeftClosed);
//            robot.setRightHangServo(Constants.hangRightClosed);
//        }

        if (gamepad1.b) {
            hangOn = true;
            LHDone = false;
            RHDone = false;
            robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftLiftMotor.setPower(.6);
            robot.rightLiftMotor.setPower(-.6);
            robot.setRightHangServo(Constants.hangRightClosed);
            robot.setLeftHangServo(Constants.hangRightClosed); }

        if (hangOn) {
            if ((newCountL == pastCountL) && (LHDone == false) && (newCountL > -800)) {
                // Open servo
                robot.setLeftHangServo(Constants.hangLeftOpen);

                // Wait 1 sec
//                ElapsedTime time = new ElapsedTime();
//                time.reset();
//                while (time.milliseconds() < 60) {
//                }
                robot.leftLiftMotor.setPower(.2);
                LHDone = true;
            }
            if ((newCountR == pastCountR) && (RHDone == false) && (newCountR < 800)) {
                // set servo
                robot.setRightHangServo(Constants.hangRightOpen);

//                ElapsedTime time2 = new ElapsedTime();
//                time2.reset();
                // Wait 1 sec
//                while (time2.milliseconds() < 60) {
//                }
                robot.rightLiftMotor.setPower(-.2);
                RHDone = true;
            }

            if (LHDone && RHDone) {
                LoopDone = true;
                }
            }


        pastCountL = newCountL;
        pastCountR = newCountR;
        telemetry.addData("leftHang", robot.getLeftLiftMotorPosition());
        telemetry.addData("rightHang", robot.getRightLiftMotorPosition());
        telemetry.update();

        double power = -gamepad2.left_stick_y;
        if (power > 0.05) {
            double move = robot.getClawArmPosition()+0.02;
            if (move > Constants.clawArmUp) {
                move = Constants.clawArmUp;
            }
            robot.setClawArmServo(move);
        }
        else if (power < -0.05) {
            double move = robot.getClawArmPosition()-0.02;
            if (move < Constants.clawArmDown) {
                move = Constants.clawArmDown;
            }
            robot.setClawArmServo(move);
        }
        //Vertical Arm
        else if (gamepad2.x) {
            robot.setClawArmServo(Constants.clawArmMiddle);
        }

        else if (gamepad2.y) {
            robot.setClawArmServo(Constants.clawArmMiddleHigh);
        }
        if (robot.getClawArmPosition() < Constants.clawArmDown) {
            robot.setClawArmServo(Constants.clawArmDown);
        }

        if (gamepad2.right_trigger > 0.02) {
            robot.setClawServo(Constants.clawOpen);
        }
        else {
            robot.setClawServo(Constants.clawClose);
        }

        double power2 = -gamepad2.right_stick_y;
        if (power2 > 0.01 && robot.getHorizontalSlidePosition() > -890) {// && robot.getHorizontalSlidePosition() > -890) {
            robot.setHorizontalLinearPower(-power2); }
        else if ((power2 < -0.01 & robot.getHorizontalSlidePosition() < 0)){// && robot.getHorizontalSlidePosition() < 0) {
            robot.setHorizontalLinearPower(-power2);}
        else {
            robot.setHorizontalLinearPower(0); }
}
    @Override
    public void stop() {
    }
}