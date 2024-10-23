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

@TeleOp(name="TeleOp", group="Iterative OpMode")
public class BasicOpMode_Iterative extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();

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
        double rx = gamepad1.right_stick_x; // Rotation

        // Calculate the motor power drive
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // DRIVE

        if (gamepad1.dpad_up) {
            robot.LF.setPower(.3);
            robot.LB.setPower(.3);
            robot.RF.setPower(.3);
            robot.RB.setPower(.3);
        }
        else if (gamepad1.dpad_down) {
            robot.LF.setPower(-.3);
            robot.LB.setPower(-.3);
            robot.RF.setPower(-.3);
            robot.RB.setPower(-.3);
        }
        else if (gamepad1.dpad_right) {
            robot.LF.setPower(.3);
            robot.LB.setPower(-.3);
            robot.RF.setPower(-.3);
            robot.RB.setPower(.3);
        }
        else if (gamepad1.dpad_left) {
            robot.LF.setPower(-.3);
            robot.LB.setPower(.3);
            robot.RF.setPower(.3);
            robot.RB.setPower(-.3);
        }
        else{
        robot.LF.setPower(frontLeftPower);
        robot.LB.setPower(backLeftPower);
        robot.RF.setPower(frontRightPower);
        robot.RB.setPower(backRightPower);
        }


        // font
        if (gamepad1.left_bumper) {
            robot.clawServo.setPosition(Constants.clawOpen);
        } else if (gamepad1.right_bumper) {
            robot.clawServo.setPosition(Constants.clawClose);
        }

        if (gamepad1.left_trigger > 0) {
            robot.backClawServo.setPosition(Constants.backClawOpen);
        } else if (gamepad1.right_trigger > 0) {
            robot.clawServo.setPosition(Constants.backClawClose);
        }

        if (gamepad1.x) {
            robot.setHorizontalLinear(1, Constants.horizontalSlideLow);
        } else if (gamepad1.y) {
            robot.setHorizontalLinear(1, Constants.horizontalSlideHigh);
        }

        if (gamepad1.a) {
            robot.setHorizontalLinear(1, Constants.verticalSlideLow);
        } else if (gamepad1.b) {
            robot.setHorizontalLinear(1, Constants.verticalSlideHigh);
        }








/*
        if (gamepad2.b) {
            clawArmPosition = Constants.clawArmUp;
        } else if (gamepad2.a) {
            clawArmPosition = Constants.clawArmDown;
        }

        if (gamepad2.x || gamepad1.x) {
            planePosition = Constants.planeRelease;
        }

        if (gamepad2.dpad_up) {
            targetLiftPosition = Constants.liftHigh;
        } else if (gamepad2.dpad_down) {
            targetLiftPosition = Constants.liftLow;
        }

        if (gamepad2.y) {
            targetHangPosition = Constants.hangHigh;
        } else if (gamepad2.x) {
            targetHangPosition = Constants.hangLow;
        }


        if (gamepad2.dpad_left) {
            targetLiftPosition = Constants.liftHigh;
            clawArmPosition = Constants.clawArmUp;
        } else if (gamepad2.dpad_right) {
            targetLiftPosition = Constants.liftLow;
            clawArmPosition = Constants.clawArmDown;
        }


        double liftJoystick = gamepad2.left_stick_y;
        if (liftJoystick < -0.12) {
            liftUseEnc = true;
            // user trying to lift up
            if (currentLiftPosition < Constants.liftMax || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftJoystick > 0.12) {
            liftUseEnc = true;
            // user trying to lift down
            if (currentLiftPosition > Constants.liftMin || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftDownRatio;
                if (currentLiftPosition > Constants.liftSlow) {
                    liftPower *= Constants.liftSlowRatio;
                }
            } else {
                liftPower = 0;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }


        double hangJoystick = gamepad2.right_stick_y;
        if (hangJoystick < -0.12) {
            hangUseEnc = true;
            // user trying to lift up
            if (currentHangPosition < Constants.hangMax || !limits) {
                useHangPower = true;
                hangPower = hangJoystick * Constants.hangUpRatio;
            } else {
                hangPower = 0;
            }
        } else if (hangJoystick > 0.12) {
            hangUseEnc = true;
            // user trying to lift down
            if (currentHangPosition > Constants.hangMin || !limits) {
                useHangPower = true;
                hangPower = hangJoystick * Constants.hangDownRatio;
                if (currentHangPosition > Constants.hangSlow) {
                    hangPower *= Constants.hangSlowRatio;
                }
            } else {
                hangPower = 0;
            }
        } else if (useHangPower) {
            hangPower = 0;
        }


        double clawArmJoystick = -gamepad2.right_stick_y;
        if (clawArmJoystick > 0.2) {
            // User trying to push the clawArm up by pushing the joystick up
            if ((clawArmPosition + Constants.clawArmSpeed) < Constants.clawArmUp) { // making sure servo value doesnt go higher than max
                clawArmPosition += Constants.clawArmSpeed * clawArmJoystick;
            } else {
                clawArmPosition = Constants.clawArmUp;
            }
        } else if (clawArmJoystick < -0.2) {
            // User trying to pull the clawArm down by pushing the joystick down
            if ((clawArmPosition - Constants.clawArmSpeed) > Constants.clawArmDown) { // making sure servo value doesnt go into negative
                clawArmPosition += Constants.clawArmSpeed * clawArmJoystick; // not doing -= because clawArmJoystick is already negative
            } else {
                clawArmPosition = Constants.clawArmDown;
            }
        }*/



        /* -------------------------------------------- ACTION -------------------------------------------- */
/*
        robot.setClawServo(clawPosition);
        robot.setClawArmServo(clawArmPosition);
        robot.setPlaneServo(planePosition);

        if (liftModeUpdate && liftUseEnc) {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftModeUpdate = false;
        }

        if (useLiftPower) {
            robot.runLiftMotor(liftPower);
        } else {
            setLiftMotor(targetLiftPosition);
        }

        if (hangModeUpdate && hangUseEnc) {
            robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangModeUpdate = false;
        }

        if (useHangPower) {
            robot.runHangMotor(hangPower);
        } else {
            setHangMotor(targetHangPosition);
        }
    }*/
/*
    void setLiftMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentLiftPosition) / Constants.liftMax;
        //Initial Time
        telemetry.addData("1", "error: " + error);
        if (Math.abs(error) > (Constants.liftTolerance / -Constants.liftMax)) {
            //Setting p action
            newPower = error * Constants.liftkPTele;
            powDir = Math.signum(error);
            newPower = Math.min(Math.abs(newPower), 1);

            // Going down
            if (powDir == 1) {
                newPower = Math.max(newPower * Constants.liftDownRatio, Constants.liftMinPow);
                if (currentLiftPosition > Constants.liftSlow) {
                    newPower *= Constants.liftSlowRatio;
                }
            }
            // Going up
            else {
                newPower = Math.min(-newPower, -Constants.liftMinPow - Constants.liftkF * currentLiftPosition / Constants.liftMax);
            }
            telemetry.addData("Lift Motor", newPower);
            robot.runLiftMotor(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            robot.setLiftMotor(0.5, position);
            liftModeUpdate = true;
            liftUseEnc = false;
        } else if (!liftModeUpdate){
            robot.runLiftMotor(0);
        }
    }*/
/*
    void setHangMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentHangPosition) / Constants.hangMax;
        //Initial Time
        telemetry.addData("1", "error: " + error);
        if (Math.abs(error) > (Constants.hangTolerance / -Constants.hangMax)) {
            //Setting p action
            newPower = error * Constants.hangkPTele;
            powDir = Math.signum(error);
            newPower = Math.min(Math.abs(newPower), 1);

            // Going down
            if (powDir == 1) {
                newPower = Math.max(newPower * Constants.hangDownRatio, Constants.hangMinPow);
                if (currentLiftPosition > Constants.hangSlow) {
                    newPower *= Constants.hangSlowRatio;
                }
            }
            // Going up
            else {
                newPower = Math.min(-newPower, -Constants.hangMinPow - Constants.hangkF * currentHangPosition / Constants.hangMax);
            }
            telemetry.addData("Hang Motor", newPower);
            robot.runHangMotor(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            robot.setHangMotor(0.5, position);
            hangModeUpdate = true;
            hangUseEnc = false;
        } else if (!liftModeUpdate){
            robot.runHangMotor(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    }
    @Override
    public void stop() {
    }
}