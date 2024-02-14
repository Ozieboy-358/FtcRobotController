package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Playing_Dev extends OpMode {
    /* Declarations */

    /* Static Variable Declarations */
    static final boolean isBlu = true;
    static final boolean isClose = true;

    /* Drive Motor Names */
    static final String[] driveMotorNames = {"left motor 1", "right motor 1", "left motor 2", "right motor 2"};
    /* Module Motor Names */
    static final String winchName = "wench";
    static final String pullyName = "pully";

    /* Imu Name & Imu Config */
    static final String imuName = "imu";
    static final RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    static final RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

    /* Non-Static Variable Declarations */
    Orientation orientation;

    /* Drive Motors */
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor[] driveMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
    static final int motorArrLength = 4;

    /* Module motors */
    DcMotor winch; // This is for the winch subsystem, intended to lift the robot.
    DcMotor pully; // This is for the pully subsystem, intended to move the hook.
    /* IMU Declaration */
    IMU imu;
    /* Camera Declaration */
    private OpenCvCamera controlHubCam;
    static final double objectWidthInRealWorldUnits = 3.75;
    static final double focalLength = 728;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;
    BlueBlobDetectionPipeline pipeline;

    /* Logic Variables */
    boolean runningNow = false;
    static final double P_DRIVE_GAIN = 0.03;
    static final double P_TURN_GAIN = 0.02;
    static final double headingError = 0.1;
    static final double COUNTS_PER_INCH = 59.4195978;

    /* Seperatory Line */


    /* Seperatory Line */

    @Override
    public void init() {
        /* Drive Motor Mappings */
        for (int i = 0; i <= motorArrLength; i++) {
            driveMotors[i] = hardwareMap.dcMotor.get(driveMotorNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        /* Module Motor Mappings */
        winch = hardwareMap.dcMotor.get(winchName);
        pully = hardwareMap.dcMotor.get(pullyName);
        /* Imu Init Phase */
        imu = hardwareMap.get(IMU.class, imuName);
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                logoDirection,
                                usbDirection
                        )
                )
        );
        /* Variable Declaration */
        orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        /* OpenCV Detection Pipeline instantiation */
        pipeline = new BlueBlobDetectionPipeline();
        /* Autonomous functions instantiation */
        AutoFuncs autoFuncs = new AutoFuncs(isBlu, isClose);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        /* if you need constant telemetry updates, or a camera loop */
    }

    /* All of our additonal functions go under this scary line!!! */

    /* Function which initializes openCV */
    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        telemetry.addLine("OpenCv ready");

    }


    /* ID : OpenCvDetector, This is our class for the detection pipeline */
    class BlueBlobDetectionPipeline extends OpenCvPipeline {
        double width;
        double cX;
        double cY;

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect blue regions
            Mat blueMask = preprocessFrame(input);

            // Find contours of the detected blue regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest blue contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a blue outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 0, 255), 2);

                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);

                // Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);

                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 0, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(255, 0, 0), -1);
            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerBlue = new Scalar(90, 100, 100);
            Scalar upperBlue = new Scalar(130, 255, 255);

            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }
    }

    /* For openCV */
    private static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    /* Automated functions class */
    class AutoFuncs {
        int angularInverter = 1;
        int skipToCenterStage = 1;

        /**
         * @param blue  : Team color as boolean, red is 0, blue is 1.
         * @param close : Close or far from the backboard, if your close 1, else 0.
         */
        AutoFuncs(boolean blue, boolean close) {
            if (close) {
                skipToCenterStage *= 0;
            }
            if (blue) {
                angularInverter *= -1;
            }
        }
    }
}
         /*   private void randTask (double cX){
                double localTheta = 38.3 * angularInverter;
                double rollOut = 225.235;
                if (cX <= 200) {
                    localTheta *= -1;
                } else if (cX <= 800) {
                    localTheta *= 0;
                    rollOut = 262.20;
                } else if (cX <= 1000) {
                    localTheta *= 1;
                    rollOut = 225.235;
                } else {
                    localTheta = 0;
                    rollOut = 0;
                }
                toCore(); // Move to core
                /* Randomization task */
                /*turnToHeading(1, localTheta); // Turn to required heading
                driveStraight(1, rollOut, localTheta); // Move to required dropoff
                driveStraight(1, rollOut, -localTheta); // Return to core
                turnToHeading(1, 0); // Return to original heading
            }
            private void toCore () {
                driveStraight(1, 693.738, 0);
            }
            private void toExchange () {
                driveStraight(1, -338.269, 0);
            }
            private void toCenterStage () {
                turnToHeading(1, 90 * angularInverter);
                driveStraight(1, 1193.80 * skipToCenterStage, 90 * angularInverter);
            }
            private void toBackStage () {
                turnToHeading(1, 94.1 * angularInverter);
                driveStraight(1, 912.011, 94.1 * angularInverter);
                sleep(100);
                turnToHeading(1, -90 * angularInverter);
            }

            public void sequencedAuto (double cX){
                toCore(); // To core moves to the core position for the randomization task segment.
                sleep(100);
                randTask(cX); // Randomization task execution, automatically returns to core position at the end of the execution.
                sleep(100);
                toExchange(); // Moves to exchange position, purpose shifts between modifiers, but generally the exchange between front stage and center stage.
                sleep(100);
                toCenterStage(); // Moves to center stage given we arnt already there.
                sleep(100);
                toBackStage(); // Parks in the backstage, given we have nothing else to do, also prealigns the
            }

        }

        /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
       /* public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = leftMotor1.getCurrentPosition() + moveCounts;
            rightTarget = rightMotor1.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftMotor1.setTargetPosition(leftTarget);
            rightMotor1.setTargetPosition(rightTarget);
            leftMotor2.setTargetPosition(leftTarget);
            rightMotor2.setTargetPosition(rightTarget);

            leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftMotor1.isBusy() && rightMotor1.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
        public void turnToHeading(double maxTurnSpeed,
                                  double heading){

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    }

        */
