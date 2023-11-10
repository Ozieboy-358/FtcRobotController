package org.firstinspires.ftc.teamcode;
import android.util.Size;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// imports for @tags, use these to declare a class as manual, auto, or disabled "wont appear in library"
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
// Importing our config

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name="DEBUG", group="Linear OpMode")
public class KF_DEV_DEBUG extends LinearOpMode {
    // Members, localized private instances of classes to be bent to our will!
    // ElapsedTime
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor's
    private DcMotor f_L_Motor = null;
    private DcMotor f_R_Motor = null;
    private DcMotor b_L_Motor = null;
    private DcMotor b_R_Motor = null;
    DcMotor[] motorArr = {f_L_Motor,f_R_Motor,b_L_Motor,b_R_Motor};
    // IMU
    private IMU imu = null;
    /* Key Config Datum */
    /*
     *  headingError : The amount of allowed error to exit a odeometric movement method.
     *  targetHeading : Value used by odeometric movement methods to define the requested heading (Think heading as in boat/plane heading).
     */
    private double headingError = 0;
    private double targetHeading = 0;

    /* Throw quadWheelBase Object together */
    private quadWheelBase robotBase = new quadWheelBase(
            imu,
            runtime,
            gamepad1,
            gamepad2,
            motorArr
    );
    @Override
    public void runOpMode() {
        // Init methods packaged inside of my class quadWheelBase, done so I can make methods refer to all the members in bulk, flavored modular.
        robotBase.motorInit(
                DebugConfig.quadBaseMotorNamesTitans
        );
        robotBase.imuInit(
                DebugConfig.logoDirection,
                DebugConfig.usbDirection
        );
        // Driver press play, Continue, before that some telemetry updates.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotBase.imu.resetYaw();
        // waitForStart method waits for start, pausing script progression.
        waitForStart();
        // Singleton Header.  Fires once and only once.
        runtime.reset();
        while (opModeIsActive()) {
            // Control loop of robot.
            // This block gets the robots current orientation in the form of 3 angular rotations about the X Y and Z axes.
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,AngleUnit.RADIANS);
            double yRot = orientation.secondAngle;

            robotBase.buttonCheck();

            // POV Mode : uses left joystick to go forward & strafe, and right joystick to rotate.
            double x = -gamepad1.left_stick_y;  // Correct for negative stick value, hardware issue hack job.
            double y = gamepad1.left_stick_x;
            // adjust this to an inverse tangent function to take the hypotenuse using x and y and pull angle B from the trigangle :<) VV
            double rotAngle = gamepad1.right_stick_x;

            // botHeading, updated per run of the loop, returns the angle in radians,

            // Adjusts X and Y for the current bot heading to the proper forward,
            // check cos and sin documenation, but you need to pass radians for your degree measure
            // imagine if you will a complex number, ec
            double rotX = x * Math.cos(yRot) - y * Math.sin(yRot);
            double rotY = x * Math.sin(yRot) + y * Math.cos(yRot);

            robotBase.powSet(rotX,rotY,rotAngle);
            robotBase.updateTelemetries(runtime);
        }
    }
    /* Storing methods here for the time being */
    class quadWheelBase extends DebugConfig {
        /* Members */
        DcMotor frontLeftMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backRightMotor;
        IMU imu;
        ElapsedTime runtime;
        Gamepad gamepad1;
        Gamepad gamepad2;
        DcMotor[] quadBase;
        int motorNum = 4;
        /* Modifier Vars */
        boolean noStrafe;
        boolean onlyTurn;
        double directionalSnapModifier;
        double directionalTargetModifier;
        double speedMod;
        boolean lockRotation;

        /* Kinematic Vars */
        private double targetHeading = 0;
        private double driveSpeed = 0;
        private double turnSpeed = 0;
        private double leftSpeed = 0;
        private double rightSpeed = 0;
        private int leftTarget = 0;
        private int rightTarget = 0;


        /**
         * quadWheelBase object constructor, intended to hold all relevant robot variables in an orderly fashion.
         *
         * @param imuIn      Pass the relevant member.
         * @param runtimeIn  Pass the relevant member.
         * @param gamepad1In Pass the relevant member.
         * @param gamepad2In Pass the relevant member.
         * @param motorArrIn Pass an array containing all our motor members, dont worry both loose and array copies are in the body.
         */
        public quadWheelBase(
                IMU imuIn,
                ElapsedTime runtimeIn,
                Gamepad gamepad1In,
                Gamepad gamepad2In,
                DcMotor[] motorArrIn
        ) {
            imu = imuIn;
            runtime = runtimeIn;
            gamepad1 = gamepad1In;
            gamepad2 = gamepad2In;
            quadBase = motorArrIn;
            frontLeftMotor = quadBase[0];
            backLeftMotor = quadBase[1];
            frontRightMotor = quadBase[2];
            backRightMotor = quadBase[3];
        }

        /**
         * Motor intialization function, intended as a wrapper for all motor init stuff.
         * Current Functionality :
         * Maps motors based on the passed name parameters,
         * Sets the zero power behavior to brake,
         * Sets the proper forwards of the individual motors for a 4 wheel rectangular robot base.
         *
         * @param motorNames String[], feed the string array name of your choice, configuration should be front left, back left, front right, back right.
         */
        private void motorInit(String[] motorNames) {
            for (int i = 1; i < motorNum; i++) {
                quadBase[i] = hardwareMap.dcMotor.get(motorNames[i]);
                quadBase[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (i < 2) {
                    quadBase[i].setDirection(DcMotor.Direction.REVERSE);
                } else {
                    quadBase[i].setDirection(DcMotor.Direction.FORWARD);
                }
            }
        }

        /* Seperator */

        /**
         * imuInit method, pass the IMU, and relevant config states for an easy init.
         * For those wondering why were using this and not pulling Orientation from the config directly,
         * I direct you to the inclusion of the hardware map line, im mostly doing it to consolidate data.
         *
         * @param dirLogo Enum state, See RevHubOrientationOnRobot documentation for options.
         * @param dirUsb  Enum state, See RevHubOrientationOnRobot documentation for options.
         */
        public void imuInit(
                RevHubOrientationOnRobot.LogoFacingDirection dirLogo,
                RevHubOrientationOnRobot.UsbFacingDirection dirUsb
        ) {
            // For context were just defining the state of an enum, the long scary sentence is referencing an enum.
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(dirLogo, dirUsb);
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }

        /* Seperator */

        /**
         * Wrapper for all of our updating telemetries, so they are updated all at once, and possibly conditionally
         *
         * @param runtime ElapsedTime, pass the relevant runtime member.
         */
        public void updateTelemetries(
                ElapsedTime runtime
        ) {
            telemetry.addData("Runtime: ", (runtime.seconds()));

            telemetry.addData("Front Left Motor Power: ", frontLeftMotor.getPower());
            telemetry.addData("Back Left Motor Power: ", backLeftMotor.getPower());
            telemetry.addData("Front Right Motor Power: ", frontRightMotor.getPower());
            telemetry.addData("Back Right Motor Power: ", backRightMotor.getPower());

            telemetry.update();
        }

        /* Seperator */

        /**
         * Come up with a way to explain how sending the power to the motors in particular ratios results in motion.
         * In programmatical terms, were passing axial, lateral, and yaw as doubles between -1 and 1.
         * Of course we have our motor mappings, to ensure we are referring to the correct motors.
         * Includes a built in limiter, ensuring that even if you pass a variable that is greater than the thresholds,
         * it wont error out, and will instead merely reduce the ratio by the highest divisible. ec 1.5 0.5 0.5 1.5 adjusts to 1, 0.33, 0.33, 1
         *
         * @param axial   - Axial = Forward/Backward ratio afflict.
         * @param lateral - Lateral = Strafe Left/Right ratio afflict.
         * @param yaw     - Yaw = Angular rotation around COM given the contact point of the 4 wheels, ec rotation around center.
         */
        public void powSet(
                Double axial,
                Double lateral,
                Double yaw
        ) {
            double powArr[] = {};
            // Ratios
            powArr[0] = axial + lateral + yaw;
            powArr[1] = axial - lateral - yaw;
            powArr[2] = axial - lateral + yaw;
            powArr[3] = axial + lateral - yaw;

            // Motor power regulator preventing any number from exiting the maximum tolerances

            // This caps the max based on the highest number, maintaining the ratio whilst decreasing the overall speed, this can get innefficent when tuned wrong.
            double max = powArr[0];
            for (int i = 1; i < (powArr.length); i++) {
                max = Math.max(max, Math.abs(powArr[i]));
            }
            // divides by highest to cap minimum / maximum to within 1 to -1
            if (max > 1.0) {
                for (int i = 0; i < (powArr.length); i++) {
                    powArr[i] /= max;
                }
            }
            // Send power to motors
            for (int i = 0; i < quadBase.length; i++) {
                quadBase[i].setPower(powArr[i]);
            }
        }

        /**
         * This is a high level method intended to check all linearOpMode gamepads, you must implement your
         * own functionality into the massive internal IF block.
         */
        void buttonCheck() {
            /* Joystick Buttons, Id expect these to be our lock controls */
            if (gamepad1.left_stick_button) {
                noStrafe = true;
            }
            if (gamepad1.right_stick_button) {
                onlyTurn = true;
            }
            /* Gets xy ba, breaks if block when a variable is found */
            if (gamepad1.x) {
                directionalSnapModifier = -90;
            } else if (gamepad1.y) {
                directionalSnapModifier = -90;
            } else if (gamepad1.b) {
                directionalSnapModifier = -90;
            } else if (gamepad1.a) {
                directionalSnapModifier = -90;
            }
            /* Gets dpad, breaks if block when a variable is found */
            if (gamepad1.dpad_up) {
                directionalTargetModifier = 0;
            } else if (gamepad1.dpad_down) {
                directionalTargetModifier = 0;
            } else if (gamepad1.dpad_left) {
                directionalTargetModifier = 0;
            } else if (gamepad1.dpad_right) {
                directionalTargetModifier = 0;
            }
            /* Gets Bumpers, Remove the else tag to allow both to run if you change functionality */
            if (gamepad1.right_bumper) {
                speedMod = 2;
            } else if (gamepad1.left_bumper) {
                speedMod = 0.5;
            }
            /* Gets Center Buttons, These all hold key values and are misc, thus no skips by default */
            if (gamepad1.start) ;
            if (gamepad1.back) ;
            if (gamepad1.guide) ;
            // Gamepad 2
            /* Joystick Buttons, Id expect these to be our lock controls */
            if (gamepad2.left_stick_button) {
                noStrafe = true;
            }
            if (gamepad2.right_stick_button) {
                onlyTurn = true;
            }
            /* Gets xy ba, breaks if block when a variable is found */
            if (gamepad2.x) {
                directionalSnapModifier = -90;
            } else if (gamepad2.y) {
                directionalSnapModifier = -90;
            } else if (gamepad2.b) {
                directionalSnapModifier = -90;
            } else if (gamepad2.a) {
                directionalSnapModifier = -90;
            }
            /* Gets dpad, breaks if block when a variable is found */
            if (gamepad2.dpad_up) {
                directionalTargetModifier = 0;
            } else if (gamepad2.dpad_down) {
                directionalTargetModifier = 0;
            } else if (gamepad2.dpad_left) {
                directionalTargetModifier = 0;
            } else if (gamepad2.dpad_right) {
                directionalTargetModifier = 0;
            }
            /* Gets Bumpers, Remove the else tag to allow both to run if you change functionality */
            if (gamepad2.right_bumper) {
                speedMod = 2;
            } else if (gamepad2.left_bumper) {
                speedMod = 0.5;
            }
            /* Gets Center Buttons, These all hold key values and are misc, thus no skips by default */
            if (gamepad2.start) ;
            if (gamepad2.back) ;
            if (gamepad2.guide) ;
        }

        /**
         * Issues i see with the current drive distance program, potentially incorrect encoder target posistion given turn is implemented,
         * find a way to implement an offset
         *
         * @param cmDistance
         * @param maxSpeed
         * @param heldHeading
         */
        void driveDistance(
                double cmDistance,
                double maxSpeed,
                double heldHeading
        ) {
            int pulseCount = (int) (cmDistance * DebugConfig.PULSES_PER_CM);
            int[] orderedTargets = {};
            // For every index in quadBase, which is 4, run the following : set the target pos to the current pos + the pulse count "see above", set the run mode to run to pos.
            for (int i = 0; i < quadBase.length; i++) {
                quadBase[i].setTargetPosition(quadBase[i].getCurrentPosition() + pulseCount);
                quadBase[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            powSet(Math.abs(maxSpeed), 0.0, 0.0);
            while (
                    opModeIsActive()
                            && quadBase[0].isBusy()
                            && quadBase[1].isBusy()
                            && quadBase[2].isBusy()
                            && quadBase[3].isBusy()
            ) {
                // Adjust the heading to ensure its not slowly drifted overtime.
                double turnSpeed = getHeadingCorrection(heldHeading, DebugConfig.driveGain);
                if (cmDistance < 0) {
                    turnSpeed *= -1.0;
                }
                powSet(cmDistance, 0.0, turnSpeed);
            }


        }

        public void turnToHeading(double maxSpeed, double heldHeading) {

            // Run getSteeringCorrection() once to pre-calculate the current error
            getHeadingCorrection(heldHeading, DebugConfig.driveGain);

            // keep looping while we are still active, and not at heading.
            while (opModeIsActive() && (Math.abs(headingError) > DebugConfig.headingThreshold)) {

                // Determine required steering to keep on heading
                turnSpeed = getHeadingCorrection(heldHeading, DebugConfig.driveGain);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -(DebugConfig.maxTurnSpeed), (DebugConfig.maxTurnSpeed));

                // Pivot in place by applying the turning correction
                powSet(0.0, 0.0, turnSpeed);
            }

            // Stop all motion;
            powSet(0.0, 0.0, 0.0);
        }

        /**
         * Returns an orientation object pulled from the imu, prestacked with all the stats we like.
         *
         * @return Returns an orientation object, pull from that with .secondAngle to get the Y rot, ec the heading.
         */
        Orientation getImuOrientation() {
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            return orientation;
        }

        ;

        double getHeadingCorrection(double targetHeading, double gainFactor) {
            headingError = targetHeading - getImuOrientation().secondAngle;
            if (headingError > 180) headingError -= 360;
            if (headingError <= 180) headingError += 360;
            return Range.clip(headingError * gainFactor, -1, 1);
        }

        ;

        class debugMovementFunctions {
            int resolutionX;
            int resolutionY;
            String fileName;
            ElapsedTime internalTime = runtime;
            RoboCam roboCam = new RoboCam(fileName, resolutionX, resolutionY);
            String fieldState = "none";
            String startState = DebugConfig.startState;
            String allianceState = DebugConfig.allianceState;
            int[] redAllianceTagArr = DebugConfig.redAllianceTagArr;
            int[] blueAllianceTagArr = DebugConfig.blueAllianceTagArr;
            int allianceAngleInverser;

            /**
             * Generates a debugMovementFunctions Instance,
             * said instance contains all our movement methods for autonomous,
             * it also contains any advanced methods intended for usage in manual control period.
             *
             * @param fileName
             * @param resolutionX
             * @param resolutionY
             */
            public debugMovementFunctions(
                    String fileName,
                    int resolutionX,
                    int resolutionY,
                    int[] blueAllianceTags,
                    int[] redAllianceTags
            ) {
                this.resolutionX = resolutionX;
                this.resolutionY = resolutionY;
                this.blueAllianceTagArr = blueAllianceTags;
                this.redAllianceTagArr = redAllianceTags;
                this.fileName = fileName;
                switch (allianceState) {
                    case "red":
                        allianceAngleInverser = 1;
                    case "blue":
                        allianceAngleInverser = -1;
                }
            }
            /*
            81 centimeters for leg 1, 47 or 221 centimeters depending on position (whether the robot
            starts in the back or the front) for leg 2, and 66 centimeters for leg three.
             */
            void leg1_fieldScan(int timeout, String objectName) { // Creates a new ElapsedTime instance, see there documentation.
                timeout += internalTime.time();
                String strFieldState = "none";

                while (internalTime.time() <= timeout) {
                    strFieldState = roboCam.detectFieldMode(objectName);
                    if (strFieldState != "none") {
                        break;
                    }
                }
                fieldState = strFieldState;
            } // pick one at random right now.
            void leg1_movement() {
                driveDistance(81,1,0);
            }
            
            void leg1_deployPixel() {
                switch (fieldState) {
                    case "left": {

                    }
                    ;
                    case "center": {

                    }
                    ;
                    case "right": {

                    }
                    ;
                    case "none": {

                    }
                    ;  // saved this incase you want to add a telemetry call like "I didnt get a fieldstate :>);
                }
            }

            void leg2_movement() {
                int travelDistance = (startState == "close") ? 47 : 221;

            }

            void leg2_backboardScan() {
                // scan the backboard once we have landed there, align properly with that section of the backboard.
                AprilTagDetection currentDetection = roboCam.detectAprilTagById(1);
            }

            void leg2_deployPixel() {
                // deploy the pixel on the backboard.  pixels if you grabbed a white one.
            }

            void leg3_movement() {
                // park and prealign for a strong start in the manual control period.
            }
        }

        class RoboCam extends DebugConfig {
            VisionPortal visPortal;
            TfodProcessor tFodProcessor;
            AprilTagProcessor aprilTagProcessor;
            int resWidth;
            int resHeight;
            String[] searchObjectNames;

            RoboCam(
                    String fileNameStr,
                    int resWidthIn,
                    int resHeightIn
            ) {
                resWidth = resWidthIn;
                resHeight = resHeightIn;
                tFodProcessor = new TfodProcessor.Builder().setModelFileName(fileNameStr).build();
                aprilTagProcessor = new AprilTagProcessor.Builder().build();
                visPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, camName))
                        .setCameraResolution(new Size(resWidth, resHeight))
                        .addProcessors(aprilTagProcessor, tFodProcessor)
                        .build();
            }

            /**
             * Checks current detections for an april tag detection with an ID equal to the input query.
             *
             * @param id int, check the handbook or your custom tags for more.
             * @return AprilTagDetection, the AprilTagDetection object you care about.
             */
            public AprilTagDetection detectAprilTagById(int id) {
                for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                    if (detection.id == id) {
                        return detection;
                    }
                }
                return null;
            }

            ;

            /**
             * Pass an object, if said object exists in a division of the screen, returns left third, middle third, or right third.
             *
             * @param objectName - Name as string of the object to look for.
             * @return
             */
            public String detectFieldMode(String objectName) {
                int thirdWidth = resWidth / 3;
                if (getSpecificRecognitonInArea(0, 0, thirdWidth * 2, 0, objectName)) {
                    return "left";
                } else if (getSpecificRecognitonInArea(thirdWidth, 0, thirdWidth, 0, objectName)) {
                    return "middle";
                } else if (getSpecificRecognitonInArea(thirdWidth * 2, 0, 0, 0, objectName)) {
                    return "right";
                }
                return "none";
            }

            /**
             * Searches for any recognitions inside a defined area, for context this blacks out whatever you define.
             * Personal recommendation, take your total resolution and if you for instance want 1/3 of the screen on the left.
             * Take the width, divide it into thirds, and put two thirds of blackout on the right.
             *
             * @param left
             * @param top
             * @param right
             * @param bottom
             * @return
             */
            private boolean getRecognitonsInArea(
                    int left,
                    int top,
                    int right,
                    int bottom
            ) {
                tFodProcessor.setClippingMargins(left, top, right, bottom);
                for (Recognition recognition : tFodProcessor.getRecognitions()) {
                    for (String name : searchObjectNames) {
                        if (recognition.getLabel() == name) {
                            tFodProcessor.setClippingMargins(0, 0, 0, 0);
                            return true;
                        }
                    }
                }
                tFodProcessor.setClippingMargins(0, 0, 0, 0);
                return false;
            }

            /**
             * Searches for a specific objectLabel inside of a particular area, same functionality as getRecognitionsInArea otherwise.
             *
             * @param left
             * @param top
             * @param right
             * @param bottom
             * @param objectLabel
             * @return
             */
            private boolean getSpecificRecognitonInArea(
                    int left,
                    int top,
                    int right,
                    int bottom,
                    String objectLabel
            ) {
                tFodProcessor.setClippingMargins(left, top, right, bottom);
                for (Recognition recognition : tFodProcessor.getRecognitions()) {
                    if (recognition.getLabel() == objectLabel) {
                        tFodProcessor.setClippingMargins(0, 0, 0, 0);
                        return true;
                    }
                }
                tFodProcessor.setClippingMargins(0, 0, 0, 0);
                return false;
            }
        }
    }
}
