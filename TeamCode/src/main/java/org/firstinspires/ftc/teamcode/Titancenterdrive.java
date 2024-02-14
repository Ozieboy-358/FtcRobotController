package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Titancenterdrive",group = "Titan")
public class Titancenterdrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left motor 1");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("left motor 2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("right motor 1");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("right motor 2");
        DcMotor pully = hardwareMap.dcMotor.get("pully");
        DcMotor wench = hardwareMap.dcMotor.get("wench");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower*.7);
            motorBackLeft.setPower(backLeftPower*.7);
            motorFrontRight.setPower(frontRightPower*.7);
            motorBackRight.setPower(backRightPower*.7);

            if (gamepad1.right_trigger>0){
                motorBackLeft.setPower(backLeftPower/2);
                motorBackRight.setPower(backRightPower/2);
                motorFrontRight.setPower(frontRightPower/2);
                motorFrontLeft.setPower(frontLeftPower/2);
            }

            if (gamepad2. left_trigger>0){
                pully.setPower(-100);

            }
            else if (gamepad2.right_trigger>0){
                pully.setPower(100);
            }
            else {
                pully.setPower(0);
            }
            if (gamepad2.left_bumper){
                wench.setPower(-100);
            } else if (gamepad2.right_bumper) {
                wench.setPower(100);
            }
            else wench.setPower(0);
        }
    }
}