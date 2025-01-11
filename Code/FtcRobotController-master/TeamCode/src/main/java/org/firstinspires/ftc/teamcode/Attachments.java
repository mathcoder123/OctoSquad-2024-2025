package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class Attachments{


    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public Servo clawServo, clawArmServo, basketServo, intakeL, intakeR, backClawServo;
    public DcMotorEx leftLiftMotor, rightLiftMotor, horizontalLinear, verticalLinear, LF, LB, RF, RB;
    // public WebcamName webcam;
    // public VisionPortal visionPortal;

    public void initialize(HardwareMap hardwareMap){//, boolean zeroOut){//, Telemetry telemetry_) {
//        telemetry = telemetry_;
        //FtcDashboard dashboard = FtcDashboard.getInstance();


        // Initialize Roadrunner
        //initializeRoadrunner(hardwareMap);

//        telemetry.addLine("Roadrunner Initialized");
//        telemetry.update();

        // Motors
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor"); //
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor"); //
        horizontalLinear = hardwareMap.get(DcMotorEx.class, "horizontalLinear"); //
        verticalLinear = hardwareMap.get(DcMotorEx.class, "verticalLinear"); //

        LF = hardwareMap.get(DcMotorEx.class, "LF"); //
        LB = hardwareMap.get(DcMotorEx.class, "LB"); //
        RF = hardwareMap.get(DcMotorEx.class, "RF"); //
        RB = hardwareMap.get(DcMotorEx.class, "RB"); //

//        RF.setDirection(DcMotorEx.Direction.REVERSE);  // Motor facing forward

        RF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
      //leftLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);


        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontalLinear.setTargetPosition(-10);
//        horizontalLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        telemetry.addLine("Motors Initialized");
//        telemetry.update();


        // Servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");
        intakeR = hardwareMap.get(Servo.class, "leftHangServo"); //change?
        intakeL = hardwareMap.get(Servo.class, "rightHangServo"); //change?
        backClawServo = hardwareMap.get(Servo.class, "backClawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

/*//        telemetry.addLine("Servos Initialized");
//        telemetry.update();


        // Camera
        //webcam = hardwareMap.get(WebcamName.class, webcam);

//        telemetry.addLine("Camera Initialized");
//        telemetry.update();

        // Change Drive Motor Modes if not autonomous
//        if (!auto) {
////            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        } else {
////            visionProcessor = new VisionProcessor();
////            visionPortal = VisionPortal.easyCreateWithDefaults(webcam, visionProcessor);
//        }
//*/
    }

    public void initialize2(HardwareMap hardwareMap){//, boolean zeroOut){//, Telemetry telemetry_) {
//        telemetry = telemetry_;
        //FtcDashboard dashboard = FtcDashboard.getInstance();


        // Initialize Roadrunner
        //initializeRoadrunner(hardwareMap);

//        telemetry.addLine("Roadrunner Initialized");
//        telemetry.update();

        // Motors
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor"); //
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor"); //
        horizontalLinear = hardwareMap.get(DcMotorEx.class, "horizontalLinear"); //
        verticalLinear = hardwareMap.get(DcMotorEx.class, "verticalLinear"); //

        LF = hardwareMap.get(DcMotorEx.class, "LF"); //
        LB = hardwareMap.get(DcMotorEx.class, "LB"); //
        RF = hardwareMap.get(DcMotorEx.class, "RF"); //
        RB = hardwareMap.get(DcMotorEx.class, "RB"); //

//        RF.setDirection(DcMotorEx.Direction.REVERSE);  // Motor facing forward

        RF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
        //leftLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");
        intakeR = hardwareMap.get(Servo.class, "leftHangServo"); //change?
        intakeL = hardwareMap.get(Servo.class, "rightHangServo"); //change?
        backClawServo = hardwareMap.get(Servo.class, "backClawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

/*//        telemetry.addLine("Servos Initialized");
//        telemetry.update();


        // Camera
        //webcam = hardwareMap.get(WebcamName.class, webcam);

//        telemetry.addLine("Camera Initialized");
//        telemetry.update();

        // Change Drive Motor Modes if not autonomous
//        if (!auto) {
////            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        } else {
////            visionProcessor = new VisionProcessor();
////            visionPortal = VisionPortal.easyCreateWithDefaults(webcam, visionProcessor);
//        }
//*/
    }

    // Run Motors
    public void hang(double power, int positionLeft, int positionRight) {
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
        leftLiftMotor.setTargetPosition(positionLeft);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setTargetPosition(positionRight);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Set Motors
    public void setHorizontalLinearPower(double power){//, int position) {
        horizontalLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalLinear.setPower(power);
//        horizontalLinear.setTargetPosition(position);
//        horizontalLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalLinear(int position) {
        horizontalLinear.setPower(1);
        horizontalLinear.setTargetPosition(position);
        horizontalLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setVerticalLinear(double power, int position) {
        verticalLinear.setPower(power);
        verticalLinear.setTargetPosition(position);
        verticalLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    // Set Servos
    public void setClawServo (double position) {clawServo.setPosition(position);}
    public void setClawArmServo (double position) {clawArmServo.setPosition(position);}
    public void setBasketServo  (double position) {basketServo.setPosition(position);}
//    public void setLeftHangServo  (double position) {leftHangServo.setPosition(position);}
//    public void setRightHangServo  (double position) {rightHangServo.setPosition(position);}
    public void setBackClawServo  (double position) {backClawServo.setPosition(position);}


    // Get Motor Positions
    public int getHorizontalSlidePosition() {
        return horizontalLinear.getCurrentPosition();
    }
    public int getVerticalSlidePosition() {
        return verticalLinear.getCurrentPosition();
    }
    public int getLeftLiftMotorPosition() {
        return leftLiftMotor.getCurrentPosition();
    }
    public int getRightLiftMotorPosition() {
        return rightLiftMotor.getCurrentPosition();
    }

    // Get Servo Positions
    public double getClawPosition() {
        return clawServo.getPosition();
    }
    public double getClawArmPosition() {
        return clawArmServo.getPosition();
    }
//    public double getLeftHangServoPosition() {
//        return leftHangServo.getPosition();
//    }
//    public double getRightHangServoPosition() {
//        return rightHangServo.getPosition();
//    }
    public double getBackClawServoPosition() {
        return backClawServo.getPosition();
    }
    public double getBasketServoPosition() {
        return basketServo.getPosition();
    } }
/*
    // Getting Prop Location
//    public int getPropLocation() {
//        VisionProcessor.Selected selection = visionProcessor.getSelection();
//        if (selection == VisionProcessor.Selected.LEFT) {
//            return 1;
//        } else if (selection == VisionProcessor.Selected.MIDDLE) {
//            return 2;
//        } else if (selection == VisionProcessor.Selected.RIGHT) {
//            return 3;
//        } else {
//            return 0;
//        }
//    }


}*/