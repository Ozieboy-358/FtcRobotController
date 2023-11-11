package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/* Important info to know
This is a config file with all values that are predefined for our bot, mostly to cut down on complications.
This should be tailored to the specifications of each bot and pulled from for opmodes.
This should NOT contain any instructions and should only contain information specifically related to bootstraps.
*/
public class Config {
    // Motor mappings
    static String teamName = "olympians"; // "olympians" "titans"

    static String[] quadBaseMotorNames = {"left motor 1", "left motor 2", "right motor 1", "right motor 2"};
    // static String[] quadBaseMotorNamesTitans = {"leftFront","leftRear","rightFront","rightRear"};
    // static String[] quadBaseMotorNamesTitans = {"frontleft", "backleft", "frontright", "backright"};
    static String[] whinchPulleyMotorNames = {"wench", "pully"};
    static String[] slideMotorNames = {"slide1", "slide2"};
    static String grabServoName = "grab";
    double headingError = 0.01;
    static String camName = "Webcam 1";
    static String fileName = "Insert_Your_File_Name_For_Your_Object_Here";
    static String startState = "close"; //"far"
    static String allianceState = "red"; //"blue"
    static int[] blueAllianceTagArr = {1, 2, 3};
    static int[] redAllianceTagArr = {4, 5, 6};
    static double driveGain = 0.03;
    static double headingThreshold = 0.01;
    static double maxTurnSpeed = 1;
    static int resolution = 720;

    // imu orientation
    static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    // Motor datum for kinematic reasoning - Pro tip, the only number you really need is pulses_per_inch
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_CM = 10.16;     // For figuring circumference
    static final double PULSES_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.14159);
}