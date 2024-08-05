package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Config
public class Constants {

    public static RevBlinkinLedDriver.BlinkinPattern closePattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    public static RevBlinkinLedDriver.BlinkinPattern openPattern = RevBlinkinLedDriver.BlinkinPattern.RED;

    /* -------------------------------------------- TELE OP CONSTANTS -------------------------------------------- */

    public static int startPos = 1;
    /*
        1: Blue Left
        2: Blue Right
        3: Red Left
        4: Red Right
     */


    /* -------------------------------------------- AUTO CONSTANTS -------------------------------------------- */

    public static int parkLoc = 1; // 2 - 1 - 1 - 2 -> LEFT TO RIGHT
    public static int sleepTime = 0; // MILLISECONDS
    public static boolean cycle = false;

    public static boolean continueAutoAfterSpikeMark = true;
    public static int testPropLoc = 1;

    public static double monkey = 1;


    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */

    public static double moveSpeed = 1;
    public static double rotSpeed = 1;


    public static double slowerMoveVel = 10;
    public static double slowerSplineVel =20;


    /* -------------------------------------------- SERVO CONSTANTS -------------------------------------------- */

    // TODO: Tune Values

    public static double pixelHold = 0.5;
    public static double pixelDrop = 1;

    public static double clawClose = 0.3;
    public static double clawOpen = 0.97;


    public static int clawArmLiftDelay = 100;
    public static int clawArmDownDelay = 30;


    public static double clawArmAutoDrop = 0.37;
    public static double clawArmTeleDrop = 0.38;
    public static double clawArmDown = 0.86;
    public static double clawArmUp = 0.3;
    public static double clawArmHigh = 0.51;
    public static double clawArmFar = 0.24;
    public static double clawArmLow = 0.72; // Arm position where it is safe to lower claw
    public static double clawArmDrive = 0.77;
    public static double clawArmSpeed = 0.006;
    public static double clawArmSlowRatio = 0.5;

    public static double clawArm0 = 0.28;
    public static double clawArm60 = clawArmHigh;
    public static double clawArmValPerDegree = (clawArm60 - clawArm0)/60;

    public static double turnClawUp = 0.9; //0.7
    public static double turnClawDown = 0.55; //0.25
    public static double turnClawSpeed = 0.02;

    public static double turnClaw180 = 1; //0.75
    public static double turnClaw90 = 0.7; //0.54
    public static double turnClaw0 = 0.63; //0.33
    public static double turnClawValPerDegree = (turnClaw180 - turnClaw0)/180;

    public static double planeHold = 0.4;
    public static double planeRelease = 0.25;


    /* -------------------------------------------- MOTOR CONSTANTS -------------------------------------------- */


    public static int liftMin = 0;
    public static int liftLow = 100;
    public static int liftClawArmFar = 300;
    public static int liftDropAuto = 600;
    public static int liftArm = 100;
    public static int liftHigh = 550;
    public static int liftMax = 2000;

    public static double liftUpRatio = 1;
    public static double liftDownRatio = 0.5;
    public static int liftSlow = 100;
    public static double liftSlowRatio = 1;

    public static int liftTolerance = 15;
    public static int liftkPTele = 10;
    public static double liftkP = 0.005; // 10
    public static double liftkI = 0;
    public static double liftkD = 0.001;
    public static double liftkF = 0.25;
    public static double liftMinPow = 0.1;




    public static int hangMin = 0;
    public static int hangLow = -1500;
    public static int hangHigh = -2600;
    public static int hangMax = -3100;

    public static double hangUpRatio = 1;
    public static double hangDownRatio = 0.8;
    public static int hangSlow = 100;
    public static double hangSlowRatio = 0.4;

    public static int hangTolerance = 15;
    public static int hangkPTele = 10;
    public static double hangkP = 0.005; // 10
    public static double hangkI = 0;
    public static double hangkD = 0.001;
    public static double hangkF = 0.25;
    public static double hangMinPow = 0.1;


    /* -------------------------------------------- VISION RECTANGLE CONSTANTS -------------------------------------------- */

    public static int rLx = 5;
    public static int rLy = 360;
    public static int rLw = 100;
    public static int rLh = 60;

    public static int rRx = 500;
    public static int rRy = 360;
    public static int rRw = 135;
    public static int rRh = 80;

    public static int rMx = 140;
    public static int rMy = 360;
    public static int rMw = 300;
    public static int rMh = 60;

    /* --------------------------------------------  APRILTAG CONSTANTS ------------------------------------------ */
    public static double DESIRED_DISTANCE = 7;
    public static double APRIL_TAG_OFFSET = 1;
    public static double DROP_TOP_FIRST = 1.8;
    public static double CAMERA_TO_CENTER = 8;
    public static double SPEED_GAIN  =  0.02;
    public static double STRAFE_GAIN =  0.015;
    public static double TURN_GAIN   =  0.01;
    public static double MAX_AUTO_SPEED = 0.5;
    public static double MAX_AUTO_STRAFE = 0.1;
    public static double MAX_AUTO_TURN  = 0.05;

}
