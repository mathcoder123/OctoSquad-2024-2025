/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="FarSpecimen", group="Robot")

public class FarSpecimen extends LinearOpMode {

    private final Attachments robot = new Attachments();

    /* Declare OpMode members. */
    private DcMotorEx         LF   = null;
    private DcMotorEx         LB  = null;
    private DcMotorEx         RF   = null;
    private DcMotorEx         RB  = null;
    private GoBildaPinpointDriver odo = null; // Declare OpMode member for the Odometry Computer
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    private int     leftFrontTarget   = 0;
    private int     leftBackTarget    = 0;
    private int     rightFrontTarget  = 0;
    private int     rightBackTarget   = 0;

    private double  leftFrontSpeed     = 0;
    private double  rightFrontSpeed    = 0;
    private double  leftBackSpeed     = 0;
    private double  rightBackSpeed    = 0;

//    public static double clawClose = 0.67;
//    public static double clawOpen = 0.5;
//
//    public static double backClawClose = 0.68;
//    public static double backClawOpen = 0.54;
//
//    public static double clawArmDown = 0.03;
//    public static double clawArmUp = 0.55;
//    public static double clawArmMiddle = 0.10;
//    public static double clawArmMiddleHigh = 0.30;
//    //public static double clawArmSpeed = 0.05;
//
//    public static double hangLeftOpen = 0.4;
//    public static double hangLeftClosed = 0.5;
//
//    public static double hangRightOpen = 0.4;
//    public static double hangRightClosed = 0.5;
//
//    public static double basketOpen = 1;
//    public static double basketClosed = 0.54;
//
//    /* -------------------------------------------- MOTOR CONSTANTS -------------------------------------------- */
//
//
//    public static int horizontalSlideLow = 0;
//    public static int horizontalSlideHigh = -1000;
//
//    public static int verticalSlideLow = 0;
//    public static int verticalSlideHigh = -4365;//3200;
//    public static int verticalSlideSubmersible = -2936;//2100;
//    public static int verticalSlideBasket = -2377;//1700;
//
//    public static int hangLeftLow = 0;
//    public static int hangLeftHigh = -3200;
//    public static int hangLeftDown = 100;
//
//    public static int hangRightLow = 0;
//    public static int hangRightHigh = 3200;
//    public static int hangRightDown = -100;




    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_CM   = 10.4 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_CM * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.1;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 0.1 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.


    static final double    P_DRIVE_THRESHOLD       = 60.0;
    static final double    P_DRIVE_MAX_ERROR = 0.02;



    @Override
    public void runOpMode() {

        robot.initialize(hardwareMap);

        // Initialize the drive system variables.
        RB = robot.LF; // hardwareMap.get(DcMotorEx.class, "LF"); //RB
        RF = robot.LB; //hardwareMap.get(DcMotorEx.class, "LB"); //RF
        LB = robot.RF; //hardwareMap.get(DcMotorEx.class, "RF"); //LB
        LF = robot.RB; //hardwareMap.get(DcMotorEx.class, "RB"); //LF

        RF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */


        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            telemetry.addData("X ",pos.getX(DistanceUnit.CM));
            telemetry.addData("Y ",pos.getY(DistanceUnit.CM));
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        //0.2 had 0 error 10 cm thresh
        //0.5 had 7 cm error at 10 cm thresh
        //0.5 had 3 cm error at 20 cm thresh (all overshoot) @ 100 dis
        //0.5 had -0.5 cm error at 30 cm thresh (undershoot) @ 100 dis
        //0.5 had -0.7 cm error at 30 cm thresh (undershoot) @ 200 dis
        //1.0 had 11.2 cm error at 30 cm thresh (undershoot) @ 200 dis
        //1.0 had 6.5 cm error at 40 cm thresh (undershoot) @ 200 dis
        //1.0 had 3.97 cm error at 50 cm thresh (undershoot) @ 200 dis
        //1.0 had 4.8 cm error at 50 cm thresh (undershoot) @ 100 dis
        //1.0 had -0.5 cm error at 60 cm thresh (undershoot) @ 100 dis
        //1.0 had -0.2 cm error at 60 cm thresh (undershoot) @ 200 dis

        //////////TODO TODO TODO TODO TODO TODO TODO TODO TODO //////////////////////////////////
        //////////TODO TODO TODO TODO TODO TODO TODO TODO TODO //////////////////////////////////
        //////////TODO TODO TODO TODO TODO TODO TODO TODO TODO //////////////////////////////////
        //////////TODO TODO TODO TODO TODO TODO TODO TODO TODO //////////////////////////////////

        // Specimen hang

        driveReversePID(1.0, 25, 0, 60, 0.6);
        robot.setBackClawServo(Constants.backClawClose);
        robot.setClawArmServo(Constants.clawArmMiddleHigh);
        robot.setVerticalLinear(1.0,-300);
        driveRightPID(1.0, 28, 0, 60, 0.6);
        robot.setVerticalLinear(1.0,-1960);
//        driveRightPID(1.0, 30, 0, 60, 0.6);
        driveReversePID(1.0,44, 0,60,0.6);
        driveReversePIDLim(0.2,1000, 0,60,0.6, 66); // 1 second = 66.666
        robot.setVerticalLinear(1.0, -1632);
        timer(800);
        robot.setBackClawServo(Constants.backClawOpen);

        // High Basket Drop
        driveForwardPID(1.0,45, 0,60,1);
        robot.setVerticalLinear(1.0, 0);
        driveRightPID(1.0,127, 0,60,1);
        turnToHeading(.8, -180);
        robot.setClawArmServo(Constants.clawArmDown);
        robot.setClawServo(Constants.clawOpen);
        timer(100);
        robot.setHorizontalLinear(-480);
//        driveForwardPID(0.4,4, 0,60,0.2);
        robot.setBasketServo(Constants.basketClosed);
        timer(500);
        robot.setClawServo(Constants.clawClose+0.01);
        timer(500);
        robot.setHorizontalLinear(0);
        robot.setClawArmServo(Constants.clawArmUp);
        timer(1000);
        robot.setClawServo(Constants.clawOpen); //

        robot.setVerticalLinear(1.0, -3085);

        driveLeftPID(1.0,5, 0,60,1);
        driveReversePID(1.0,10, 0,60,1);
        turnToHeading(.8, -225);

        timer(1000);
        robot.setBasketServo(Constants.basketOpen);
        timer(1500);
        robot.setVerticalLinear(1.0, 0);

        timer(1000);

        turnToHeading(.8, 0);

        driveLeftPID(1.0, 240, 0, 60, 1);


////        timer(1000);
//        turnToHeading(.8, -180);
//        driveReversePID(1.0, 8, -180, 60, 0.6);
//        turnToHeading(.8, -235);




        //driveForwardPID(1.0, 200, 0, 60, 0.2);
        //driveReversePID(1,200,0,60.0,0.2);

        //turnToHeading(0.2, -170.0);
        //turnToHeading(0.2, 170.0);

        //driveStraightLeft(0.1, 200.0, 0.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
        //driveStraightRight(1, 200.0, 0.0);
        //driveForwardPID(0.2, 100);*/

        while (opModeIsActive()){
            odo.update();
            Pose2D pos = odo.getPosition();
            telemetry.addData("X ",pos.getX(DistanceUnit.CM));
            telemetry.addData("Y ",pos.getY(DistanceUnit.CM));
            telemetry.addData("Heading ", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        //sleep(1000);  // Pause to display last telemetry message.
    }


    /*
    public void showHeadingCorrection(double heading){
        targetHeading = heading;  // Save for telemetry
        double currentHeading;
        currentHeading = getHeading();
        // Determine the heading current error
        headingError = targetHeading - currentHeading;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        telemetry.addData("Target: Current: Err", "%5.2f : %5.2f : %5.2f", targetHeading, currentHeading, headingError);
        telemetry.addLine();
        telemetry.update();
    }*/

    /*void turnLeft(){
        //RF forward
        //RB forward
        RF.setPower(0.2);
        RB.setPower(0.2);
        //LF reverse
        //LB reverse
        LF.setPower(-0.2);
        LB.setPower(-0.2);
    }*/


    //forward X +
    //left Y +
    public void timer(int count) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.milliseconds() < count && opModeIsActive()) {
            }
    }

    public void driveForwardPID(double maxDriveSpeed,
                                double distance,
                                double heading,
                                double pThreshold,
                                double maxError){
        double newX =0;
        double currentX=0;
        double yCorrection =0;
        double yInit=0;

        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.CM);
            newX = currentX + distance;
            yInit = pos.getY(DistanceUnit.CM);
        }
        //move forward PID
        double errorX;
        errorX = newX - currentX;

        while(opModeIsActive() && (errorX > maxError)){
            odo.update();
            Pose2D pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.CM);
            errorX = newX - currentX;
            yCorrection = pos.getY(DistanceUnit.CM) - yInit;
            if(errorX > pThreshold){
                //move at 1, correct for rotation error
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                moveRobot(maxDriveSpeed, turnSpeed , yCorrection);
            } else {
                double speedFact;
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                speedFact = (errorX / pThreshold);
                moveRobot((maxDriveSpeed * speedFact), turnSpeed, yCorrection);
            }
        }

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveRobot(0, 0, 0);
    }

    public void driveReversePID(double maxDriveSpeed,
                                double distance,
                                double heading,
                                double pThreshold,
                                double maxError){
        double newX =0;
        double currentX=0;
        double yCorrection =0;
        double yInit=0;

        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.CM);
            newX = currentX - distance;
            yInit = pos.getY(DistanceUnit.CM);
        }

        //move forward PID
        double errorX;
        errorX = newX - currentX;


        while(opModeIsActive() && (errorX < (-1.0*maxError))) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.CM);
            errorX = newX - currentX;
            yCorrection = pos.getY(DistanceUnit.CM) - yInit;
            if (errorX < (-1.0*pThreshold) ) {
                //move at 1, correct for rotation error
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                moveRobot((-1.0 * maxDriveSpeed), turnSpeed, yCorrection);
            } else {
                double speedFact;
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                speedFact = (errorX / pThreshold);
                moveRobot((maxDriveSpeed * speedFact), turnSpeed, yCorrection);
            }
        }
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveRobot(0, 0, 0);
    }

    public void driveReversePIDLim(double maxDriveSpeed,
                                double distance,
                                double heading,
                                double pThreshold,
                                double maxError,
                                   double limit){
        double counter = 0;
        double newX =0;
        double currentX=0;
        double yCorrection =0;
        double yInit=0;

        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.CM);
            newX = currentX - distance;
            yInit = pos.getY(DistanceUnit.CM);
        }

        //move forward PID
        double errorX;
        errorX = newX - currentX;


        while(opModeIsActive() && (errorX < (-1.0*maxError))) {
            counter ++;
            if (counter <= limit) {
                odo.update();
                Pose2D pos = odo.getPosition();
                currentX = pos.getX(DistanceUnit.CM);
                errorX = newX - currentX;
                yCorrection = pos.getY(DistanceUnit.CM) - yInit;
                if (errorX < (-1.0*pThreshold) ) {
                    //move at 1, correct for rotation error
                    turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                    moveRobot((-1.0 * maxDriveSpeed), turnSpeed, yCorrection);
                } else {
                    double speedFact;
                    turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                    speedFact = (errorX / pThreshold);
                    moveRobot((maxDriveSpeed * speedFact), turnSpeed, yCorrection);
            }}
            else {
                LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                moveRobot(0, 0, 0);
                return;
            }
        }
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveRobot(0, 0, 0);
    }

    public void driveLeftPID(double maxDriveSpeed,
                             double distance,
                             double heading,
                             double pThreshold,
                             double maxError){
        double newY =0;
        double currentY=0;
        double xCorrection =0;
        double xInit=0;

        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentY = pos.getY(DistanceUnit.CM);
            newY = currentY + distance;
            xInit = pos.getX(DistanceUnit.CM);
        }

        //move forward PID
        double errorY;
        errorY = newY - currentY;

        while(opModeIsActive() && (errorY > maxError)) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentY = pos.getY(DistanceUnit.CM);
            errorY = newY - currentY;
            xCorrection = pos.getX(DistanceUnit.CM) - xInit;
            if (errorY > pThreshold) {
                //move at 1, correct for rotation error
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                moveLeftRobot(maxDriveSpeed, turnSpeed, xCorrection);
            } else {
                double speedFact;
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                speedFact = (errorY / pThreshold);
                moveLeftRobot((maxDriveSpeed * speedFact), turnSpeed, xCorrection);
            }
        }
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveLeftRobot(0, 0,0);
    }


    public void driveRightPID(double maxDriveSpeed,
                              double distance,
                              double heading,
                              double pThreshold,
                              double maxError){
        double newY =0;
        double currentY=0;
        double xCorrection =0;
        double xInit=0;

        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentY = pos.getY(DistanceUnit.CM);
            newY = currentY - distance;
            xInit = pos.getX(DistanceUnit.CM);
        }

        //move forward PID
        double errorY;
        errorY = newY - currentY;

        while(opModeIsActive() && (errorY < (-1.0*maxError))) {
            odo.update();
            Pose2D pos = odo.getPosition();
            currentY = pos.getY(DistanceUnit.CM);
            errorY = newY - currentY;
            xCorrection = pos.getX(DistanceUnit.CM) - xInit;
            if (errorY < (-1.0*pThreshold) ){
                //move at 1, correct for rotation error
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                moveRightRobot(maxDriveSpeed, turnSpeed, xCorrection);
            } else {
                double speedFact;
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                speedFact = Math.abs((errorY / pThreshold));
                moveRightRobot((maxDriveSpeed * speedFact), turnSpeed, xCorrection);
            }
        }
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveRightRobot(0, 0, 0);
    }



    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in cm) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_CM);
            leftTarget = LF.getCurrentPosition() + moveCounts;
            leftTarget = LB.getCurrentPosition() + moveCounts;
            rightTarget = RF.getCurrentPosition() + moveCounts;
            rightTarget = RB.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            LF.setTargetPosition(leftTarget);
            LB.setTargetPosition(leftTarget);
            RF.setTargetPosition(rightTarget);
            RB.setTargetPosition(rightTarget);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (RF.isBusy() && RB.isBusy()) &&
                    (LF.isBusy() && LB.isBusy())
            ) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed,0);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveStraightLeft(double maxDriveSpeed,
                                  double distance,
                                  double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_CM);
            leftFrontTarget = LF.getCurrentPosition() - moveCounts;
            leftBackTarget = LB.getCurrentPosition() + moveCounts;
            rightFrontTarget = RF.getCurrentPosition() + moveCounts;
            rightBackTarget = RB.getCurrentPosition() - moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            LF.setTargetPosition(leftFrontTarget);
            LB.setTargetPosition(leftBackTarget);
            RF.setTargetPosition(rightFrontTarget);
            RB.setTargetPosition(rightBackTarget);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0,0);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (RF.isBusy() && RB.isBusy()) &&
                    (LF.isBusy() && LB.isBusy())
            ) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed,turnSpeed,0);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0,0);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void driveStraightRight(double maxDriveSpeed,
                                  double distance,
                                  double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_CM);
            leftFrontTarget = LF.getCurrentPosition() + moveCounts;
            leftBackTarget = LB.getCurrentPosition() - moveCounts;
            rightFrontTarget = RF.getCurrentPosition() - moveCounts;
            rightBackTarget = RB.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            LF.setTargetPosition(leftFrontTarget);
            LB.setTargetPosition(leftBackTarget);
            RF.setTargetPosition(rightFrontTarget);
            RB.setTargetPosition(rightBackTarget);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0,0);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (RF.isBusy() && RB.isBusy()) &&
                    (LF.isBusy() && LB.isBusy())
            ) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed,0);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0,0);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************



    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        odo.update();
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            odo.update();

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobot( 0, turnSpeed, 0);
            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0,0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        turnToHeading(.8, -180);
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            odo.update();
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed,0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0,0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double currentHeading;
        targetHeading = desiredHeading;  // Save for telemetry
        currentHeading = getHeading();
        // Determine the heading current error
        headingError = targetHeading - currentHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }





    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn, double yError) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double yCorrection = 0.03 * yError;

        leftFrontSpeed  = drive - turn + yCorrection ;
        leftBackSpeed = drive - turn - yCorrection ;
        rightFrontSpeed = drive + turn - yCorrection ;
        rightBackSpeed = drive + turn + yCorrection ;

        // Scale speeds down if either one exceeds +/- 1.0;
        double maxLeft = Math.max(Math.abs(leftFrontSpeed), Math.abs(leftBackSpeed));
        double maxRight = Math.max(Math.abs(rightFrontSpeed), Math.abs(rightBackSpeed));
        double max = Math.max(maxRight,maxLeft);


        if (max > 1.0)
        {
            leftFrontSpeed /= max;
            leftBackSpeed  /= max;
            rightFrontSpeed /= max;
            rightBackSpeed /= max;
        }

        LF.setPower(leftFrontSpeed);
        LB.setPower(leftBackSpeed);

        RF.setPower(rightFrontSpeed);
        RB.setPower(rightBackSpeed);
    }

    public void moveLeftRobot(double drive, double turn, double xError) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double xCorrection = 0.03 * xError;

        leftFrontSpeed = (-1.0*drive)- turn - xCorrection;
        leftBackSpeed = drive - turn - xCorrection;

        //rightSpeed = drive + turn;
        rightFrontSpeed = drive+turn - xCorrection;
        rightBackSpeed = (-1.0*drive)+turn - xCorrection;

        // Scale speeds down if either one exceeds +/- 1.0;
        double maxLeft = Math.max(Math.abs(leftFrontSpeed), Math.abs(leftBackSpeed));
        double maxRight = Math.max(Math.abs(rightFrontSpeed), Math.abs(rightBackSpeed));
        double max =  Math.max(maxRight, maxLeft);

        if (max > 1.0)
        {
            leftFrontSpeed /= max;
            leftBackSpeed /= max;

            rightFrontSpeed /= max;
            rightBackSpeed /= max;
        }

        LF.setPower(leftFrontSpeed);
        LB.setPower(leftBackSpeed);
        RF.setPower(rightFrontSpeed);
        RB.setPower(rightBackSpeed);
    }

    public void moveRightRobot(double drive, double turn, double xError) {
            // save this value as a class member so it can be used by telemetry.
        double xCorrection = 0.03 * xError;

        leftFrontSpeed = drive - turn - xCorrection;
        leftBackSpeed = (-1.0*drive) - turn - xCorrection;

        //rightSpeed = drive + turn;
        rightFrontSpeed = (-1.0*drive) + turn - xCorrection;
        rightBackSpeed = drive + turn - xCorrection;

        // Scale speeds down if either one exceeds +/- 1.0;
        double maxLeft = Math.max(Math.abs(leftFrontSpeed), Math.abs(leftBackSpeed));
        double maxRight = Math.max(Math.abs(rightFrontSpeed), Math.abs(rightBackSpeed));
        double max =  Math.max(maxRight, maxLeft);
        if (max > 1.0)
        {
            leftFrontSpeed /= max;
            leftBackSpeed /= max;

            rightFrontSpeed /= max;
            rightBackSpeed /= max;
        }
        LF.setPower(leftFrontSpeed);
        LB.setPower(leftBackSpeed);
        RF.setPower(rightFrontSpeed);
        RB.setPower(rightBackSpeed);
    }




    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      LF.getCurrentPosition(),
                    RF.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        Pose2D pos = odo.getPosition();
        return pos.getHeading(AngleUnit.DEGREES);
    }
}
