package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {
    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    private Servo grab = null;
    private DcMotor Wench = null;
    private  DcMotor Pulley = null;
    public void runOpMode() throws InterruptedException {

        grab = hardwareMap.servo.get("grab");
        Wench = hardwareMap.dcMotor.get("Wench");
        Pulley = hardwareMap.dcMotor.get("Pulley");

    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    imu.initialize(parameters);

    new ServoTest();

   if (isStopRequested()) return;

   while (opModeIsActive()) {
       double y = -gamepad1.left_stick_y;
       double x = gamepad1.left_stick_x * 1.1;
       double rx = gamepad1.right_stick_x;

       double botHeading = -imu.getAngularOrientation().firstAngle;

       double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
       double rotY = y * Math.cos(botHeading) - x * Math.sin(botHeading);

       if (gamepad2. left_trigger>0){
           Pulley.setPower(100);
       }
       else if (gamepad2.right_trigger>0){
           Wench.setPower(100);
       }
       else {
           Pulley.setPower(0);
           Wench.setPower(0);
       }

       if (gamepad2.left_bumper=true) {
            Pulley.setPower(-100);
       }
        if (gamepad2.right_bumper=true) {
            Wench.setPower(-100);
       }
else {
    Pulley.setPower(0);
    Wench.setPower(0);
       }
       if (gamepad2.a){
           grab.setPosition(0.12);
       }
       if (gamepad2.x){
           grab.setPosition(.34);
       }


   }

}
}
