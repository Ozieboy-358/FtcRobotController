   package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Olymp centerdrive")
public class Olympcenterdrive extends LinearOpMode {

    private DcMotor slide1 = null;
    private DcMotor slide2 = null;
    private Servo grab = null;
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left motor 1");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("left motor 2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("right motor 1");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("right motor 2");
        grab = hardwareMap.servo.get("grab");
        slide1 = hardwareMap.dcMotor.get("slide1");
        slide2 = hardwareMap.dcMotor.get("slide2");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

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
                slide2.setPower(-100);
                slide1.setPower(100);
            }
            else if (gamepad2.right_trigger>0){
                slide1.setPower(-100);
                slide2.setPower(100);
            }
            else {
                slide1.setPower(0);
                slide2.setPower(0);
            }

            if (gamepad2.a){
                grab.setPosition(.72);
            }
            if (gamepad2.x){
                grab.setPosition(.96);
            }




        }
    }
}