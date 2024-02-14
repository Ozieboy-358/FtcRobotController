package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;

/* Important info to know
This is a config file with all values that are predefined for our bot, mostly to cut down on complications.
This should be tailored to the specifications of each bot and pulled from for opmodes.
This should NOT contain any instructions and should only contain information specifically related to bootstraps.
 */
@Disabled

public class DebugConfig {
    // Motor mappings
    // static String[] quadBaseMotorNamesTitans = {"left motor 1", "left motor 2", "right motor 1", "right motor 2"};
    // static String[] quadBaseMotorNamesTitans = {"leftFront","leftRear","rightFront","rightRear"};
    static String[] quadBaseMotorNamesTitans = {"leftfront", "leftback", "rightfront", "rightback"};
    static String[] whinchPulleyMotorNames = {"pully","wench"};
    static String grabServoName = "grab";
    static String camName = "Camera 1";
    static String fileName = "Insert_Your_File_Name_For_Your_Object_Here";
    static String startState = "close"; //"far"
    static String allianceState = "red"; //"blue"
    static int[] blueAllianceTagArr = {1,2,3};
    static int[] redAllianceTagArr = {4,5,6};
    static double driveGain = 0.03;
    static double headingThreshold = 0.01;
    static double maxTurnSpeed = 1;
    static int resolution = 720;

    // imu orientation
    static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    // Motor datum for kinematic reasoning - Pro tip, the only number you really need is pulses_per_inch
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_CM = 10.16;     // For figuring circumference
    static final double PULSES_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.14159);
}

