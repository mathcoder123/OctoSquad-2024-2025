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
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp1", group="Iterative OpMode")
public class BasicOpMode_Iterative extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Attachments robot = new Attachments();
    public boolean hangLockOpen = false;
    public boolean yPressed = false;

    // Definitions
    // private DcMotor leftDrive = null;
    // private DcMotor rightDrive = null;
    // FIX CONSTANTS ----------------------------------------------------------------------------
    /*private double clawPosition = Constants.clawOpen;
    private double clawArmPosition = Constants.clawArmDown;
    private double planePosition = Constants.planeHold;

    private double liftPower = 0;
    private boolean useLiftPower = true;
    private boolean liftModeUpdate = false;
    private boolean liftUseEnc = true;
    private int targetLiftPosition = Constants.liftLow;
    //private int currentLiftPosition = robot.getLiftMotorPosition();

    private double hangPower = 0;
    private boolean useHangPower = true;
    private boolean hangModeUpdate = false;
    private boolean hangUseEnc = true;
    private int targetHangPosition = Constants.hangLow;
    //private int currentHangPosition = robot.getHangMotorPosition();

    private boolean limits = true;*/



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
        /*
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);*/
        /*
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = Constants.moveSpeed;
        double rotationMultiplier = Constants.rotSpeed;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.3;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 0.3;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;*/
        /*
        // Drive

        // Move seperate motors
        // robot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);

        */
        /* -------------------------------------------- TODO -------------------------------------------- */
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
            robot.LF.setPower(.5);
            robot.LB.setPower(.5);
            robot.RF.setPower(.5);
            robot.RB.setPower(.5);
        }
        else if (gamepad1.dpad_down) {
            robot.LF.setPower(-.5);
            robot.LB.setPower(-.5);
            robot.RF.setPower(-.5);
            robot.RB.setPower(-.5);
        }
        else if (gamepad1.dpad_right) {
            robot.LF.setPower(.5);
            robot.LB.setPower(-.5);
            robot.RF.setPower(-.5);
            robot.RB.setPower(.5);
        }
        else if (gamepad1.dpad_left) {
            robot.LF.setPower(-.4);
            robot.LB.setPower(.4);
            robot.RF.setPower(.4);
            robot.RB.setPower(-.4);
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
        } else if (gamepad1.right_trigger>0.02) {
            robot.setVerticalLinear(1, Constants.verticalSlideLow);
        } else if (gamepad1.left_bumper) {
            robot.setVerticalLinear(1, Constants.verticalSlideSubmersible);
        } else if (gamepad1.left_trigger>0.02) {
            robot.setVerticalLinear(1, Constants.verticalSlideBasket);
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
        else{// if (gamepad1.b) {
            robot.setBackClawServo(Constants.backClawClose);
        }


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
//        if (gamepad2.left_trigger > 0.02) {
//            robot.hang(1, Constants.hangLeftLow, Constants.hangRightLow);
//        }
//        else if (gamepad2.left_bumper) {
//            robot.hang(1, Constants.hangLeftHigh, Constants.hangRightHigh);
//        }

        // Hang Servos
//        if (gamepad2.right_bumper) {
//            if (!yPressed) {
//                yPressed = true;
//                if (!hangLockOpen) {
//                    hangLockOpen = true;
//                    robot.setLeftHangServo(Constants.hangLeftOpen);
//                    robot.setRightHangServo(Constants.hangRightOpen);
//                }
//                else {
//                    hangLockOpen = false;
//                    robot.setLeftHangServo(Constants.hangLeftClosed);
//                    robot.setRightHangServo(Constants.hangRightClosed);
//                }
//            }
//        }
//        else {
//            yPressed = false;
//        }
//
        double power = -gamepad2.left_stick_y;
        if (power > 0.05) {
            double move = robot.getClawArmPosition()+0.01;
            if (move > Constants.clawArmUp) {
                move = Constants.clawArmUp;
            }
            robot.setClawArmServo(move);
        }
        else  if (power < -0.05) {
            double move = robot.getClawArmPosition()-0.01;
            if (move < Constants.clawArmDown) {
                move = Constants.clawArmDown;
            }
            robot.setClawArmServo(move);
        }
        //Vertical Arm
        if (gamepad2.x) {
            robot.setClawArmServo(Constants.clawArmMiddle);
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
