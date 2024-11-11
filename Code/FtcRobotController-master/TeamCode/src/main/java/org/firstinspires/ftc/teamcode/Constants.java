package org.firstinspires.ftc.teamcode;


public class Constants {

    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */
    public static double moveSpeed = 1;
    public static double rotSpeed = 1;


    /* -------------------------------------------- SERVO CONSTANTS -------------------------------------------- */
    //TODO TUNE
    public static double clawClose = 0.67;
    public static double clawOpen = 0.5;

    public static double backClawClose = 0.68;
    public static double backClawOpen = 0.54;

    public static double clawArmDown = 0.03;
    public static double clawArmUp = 0.55;
    public static double clawArmMiddle = 0.10;
    public static double clawArmMiddleHigh = 0.44;
    //public static double clawArmSpeed = 0.05;

    public static double hangLeftOpen = 0.6;
    public static double hangLeftClosed = 0.5;

    public static double hangRightOpen = 0.4;
    public static double hangRightClosed = 0.5;

    public static double basketOpen = 1.02; //1
    public static double basketClosed = 0.54;

    /* -------------------------------------------- MOTOR CONSTANTS -------------------------------------------- */


    public static int horizontalSlideLow = 0;
    public static int horizontalSlideHigh = -890;

    public static int verticalSlideLow = 0;
    public static int verticalSlideHigh = -3085;//3200;
    public static int verticalSlideSubmersible = -1960;//1950;
    public static int verticalSlideBasket = -1632;//1700;
    public static int verticalSlidePickup = -735;//1700;
    public static int verticalSlidePickupHigh = -1000;//1700;

    public static int hangLeftLow = 0;
    public static int hangLeftPickup = -700;
    public static int hangLeftHigh = -3100;

    public static int hangRightLow = 0;
    public static int hangRightPickup = 700;
    public static int hangRightHigh = 3100;
}