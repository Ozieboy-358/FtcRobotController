package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 * The code is structured as a LinearOpMode
 * <p>
 * The path to be followed by the robot is built from a series of drive, turn or pause steps.
 * Each step on the path is defined by a single function call, and these can be strung together in any order.
 * <p>
 * The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 * This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 * To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 * <p>
 * This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 * It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 * So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 * See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 * <p>
 * This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 * Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 * <p>
 * Notes:
 * <p>
 * All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 * In this sample, the heading is reset when the Start button is touched on the Driver station.
 * Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clockwise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Control Approach.
 * <p>
 * To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 * <p>
 * Steering power = Heading Error * Proportional Gain.
 * <p>
 * "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 * and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 * <p>
 * "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name = "Auto_Red_Left", group = "Robot")
public class Auto_Red_Left extends LinearOpMode {

    /* Declare OpMode members. */
    TitanRobot robot = new TitanRobot(this);

    @Override
    public void runOpMode() {
   // Desired turning power/speed (-1 to +1)
        // Initialize the drive system variables.
        robot.init();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // define initialization values for IMU, and then initialize it.


        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode


        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    //  telemetryAprilTag();

                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // Save CPU resources; can resume streaming when needed

                    // Share the CPU.
                    sleep(20);
                    robot.driveStraight(-1,90,0);
                }
                robot.driveStraight(1,90,0);
            }
            robot.StrafeDist(1,90,0);
        }


        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        /*
        reset()
        driveStraight
        holdHeading
        StrafeDist
        Negative heading turns right vice versa
        HoldTime is in Seconds
        */
        robot.moveRobot(10,0);
        // Save more CPU resources when camera is no longer needed.


    }   // end method runOpMode()
}




