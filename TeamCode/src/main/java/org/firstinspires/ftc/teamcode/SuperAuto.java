package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name = "EmergencyAuto", group = "Linear OpMode")
public class SuperAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        quadWheelBase robotBase = new quadWheelBase(
                new Config(),
                gamepad1,
                gamepad2
        );
        // Driver press play, Continue, before that some telemetry updates.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotBase.imu.resetYaw();
        // waitForStart method waits for start, pausing script progression.
        waitForStart(); // After this the script runs its gametime, use a while loop to create a while running loop.

        robotBase.runtime.reset(); // Resets runtime whenever we start the game.

        /* Autonomous Environment */
        robotBase.movFuncs.autoProtocol("insertObjHere");
        /* Manual Environment */
       /* while(opModeIsActive()){
            /* Specifically pull the base drive systems junk */
        /* Set power for stuff */
           /* switch(Config.teamName){
                case "olympians" : robotBase.buttonCheckOlympians();
                case "titans" : robotBase.buttonCheckTitans();
            }
            robotBase.universalPowPosSetter();*/
    }

    /* Storing methods here for the time being */
    class quadWheelBase extends Config {
        /* Members */
        IMU imu;
        ElapsedTime runtime;
        Gamepad gamepad1;
        Gamepad gamepad2;
        DcMotor[] slides;
        DcMotor[] quadBase = new DcMotor[4];
        Servo grab;
        DcMotor wench;
        DcMotor pully;
        Config config;
        debugMovementFunctions movFuncs;
        /* Modifier Vars */
        boolean noStrafe;
        boolean onlyTurn;
        double directionalSnapModifier;
        double directionalTargetModifier;
        double speedMod;
        boolean lockRotation;

        /* Pow Vars */
        double wenchPow;
        double pulleyPow;
        double servoPos;
        double slidePow;

        /* Kinematic Vars */
        private double turnSpeed = 0;
        Orientation orientation;
        double yRot; // Gets our starting angle to play off of for angular displacement.


        /**
         * quadWheelBase Constructor, any and all init related stuff sequences in here, period.
         *
         * @param configIn - The config this file is pulling from, see DebugConfig for a reference.
         * @param gamepad1 - Using the ease of access pre mapped controllers.
         * @param gamepad2 - Ditto.
         */
        public quadWheelBase(
                Config configIn,
                Gamepad gamepad1,
                Gamepad gamepad2
        ) {
            config = configIn; // Defines the config used in the remainder of the file

            runtime = new ElapsedTime(); // Generate runtime starting from the run of this new command.

            imuInit(config.logoDirection, config.usbDirection); // Generate a mapped IMU.
            orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            yRot = -orientation.firstAngle;

            gamepadInit(gamepad1, gamepad2); // Places our controllers into the instance for usage.

            driveMotorInit(config.quadBaseMotorNames); // Places our drive motors into the instance for usage.

            if (config.teamName == "titans") {
                slideMotorInit(config.slideMotorNames);

            } else if (config.teamName == "olympians") {
                wenchNpullyInit(config.whinchPulleyMotorNames);
            } // defines if we are using the olympians or the titans truss gripper system, then inits accordingly.

            if (config.teamName == "olympians") {
                servoInit(config.grabServoName);
            }

            moveFuncsInit(config);
        }

        public void gamepadInit(
                Gamepad gamepad1In,
                Gamepad gamepad2In
        ) {
            gamepad1 = gamepad1In;
            gamepad2 = gamepad2In;
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
        private void driveMotorInit(String[] motorNames) {
            // for context were feeding a string array containing all our motor names, this is just a
            for (int i = 0; i < motorNames.length; i++) {
                quadBase[i] = hardwareMap.dcMotor.get(motorNames[i]);
                quadBase[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (i < 2) {
                    quadBase[i].setDirection(DcMotor.Direction.REVERSE);
                } else {
                    quadBase[i].setDirection(DcMotor.Direction.FORWARD);
                }
            }
        }

        private void wenchNpullyInit(
                String[] wenchNpullyNames
        ) {
            DcMotor[] wenchNPully = new DcMotor[2];
            for (int i = 0; i < wenchNpullyNames.length; i++) {
                wenchNPully[i] = hardwareMap.dcMotor.get(wenchNpullyNames[i]);
                wenchNPully[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wenchNPully[i].setDirection(DcMotor.Direction.FORWARD);
            }
            wench = wenchNPully[0];
            pully = wenchNPully[1];
        }

        private void slideMotorInit(
                String[] motorNames
        ) {
            for (int i = 0; i <= motorNames.length; i++) {
                slides[i] = hardwareMap.dcMotor.get(motorNames[i]);
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

        public void servoInit(
                String grabIn
        ) {
            grab = hardwareMap.servo.get(grabIn);
        }

        void moveFuncsInit(
                Config config
        ) {
            movFuncs = new debugMovementFunctions(
                    config.blueAllianceTagArr,
                    config.redAllianceTagArr
            );
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

            telemetry.addData("Front Left Motor Power: ", quadBase[0].getPower());
            telemetry.addData("Back Left Motor Power: ", quadBase[1].getPower());
            telemetry.addData("Front Right Motor Power: ", quadBase[2].getPower());
            telemetry.addData("Back Right Motor Power: ", quadBase[3].getPower());

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
            double powArr[] = new double[4];
            // Ratios
            // we can do this because we know 0 will literally allways be the highest number.
            double denom = Math.max(Math.abs(lateral) + Math.abs(axial) + Math.abs(yaw), 1);
            powArr[0] = (axial + lateral + yaw) / denom;
            powArr[1] = (axial - lateral + yaw) / denom;
            powArr[2] = (axial - lateral - yaw) / denom;
            powArr[3] = (axial + lateral - yaw) / denom;
            // Send power to motors
            for (int i = 0; i < quadBase.length; i++) {
                quadBase[i].setPower(powArr[i]);
            }
        }

        private double[] getPowSetValues() {
            double x = -gamepad1.left_stick_y;  // Correct for negative stick value, hardware issue hack job.
            double y = gamepad1.left_stick_x;

            double rotAngle = gamepad1.right_stick_x;

            double rotX = x * Math.cos(yRot) - y * Math.sin(yRot);
            double rotY = x * Math.sin(yRot) + y * Math.cos(yRot);
            double[] returnVar = {rotX, rotY, rotAngle};
            return returnVar;
        }

        public void clawPowSet(
                Double wenchPow,
                Double pulleyPow
        ) {
            wench.setPower(wenchPow);
            pully.setPower(pulleyPow);
        }

        public void servoPosSet(
                double posIn
        ) {
            grab.setPosition(posIn);
        }

        public void slidePowSet(
                Double slidePow
        ) {
            for (int i = 1; i < slides.length + 1; i++) {
                if (i % 2 != 0) {
                    slides[i].setPower(slidePow);
                } else {
                    slides[i].setPower(-slidePow);
                }
            }
        }

        ;

        public void universalPowPosSetter() {
            double[] powArr = getPowSetValues();
            powSet(powArr[0], powArr[1], powArr[2]);
            if (config.teamName == "titans") {
                slidePowSet(slidePow);
            } else if (config.teamName == "olympians") {
                servoPosSet(servoPos);
                clawPowSet(wenchPow, pulleyPow);
            }
        }

        /**
         * This is a high level method intended to check all linearOpMode gamepads, you must implement your
         * own functionality into the massive internal IF block.  This particular checker is for the olympians.
         */
        void buttonCheckOlympians() {
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
                directionalSnapModifier = 0;
            } else if (gamepad1.b) {
                directionalSnapModifier = 90;
            } else if (gamepad1.a) {
                directionalSnapModifier = 180;
            }
            /* Gets dpad, breaks if block when a variable is found */
            if (gamepad1.dpad_up) {
                directionalTargetModifier = 1;
            } else if (gamepad1.dpad_down) {
                directionalTargetModifier = 1;
            } else if (gamepad1.dpad_left) {
                directionalTargetModifier = 1;
            } else if (gamepad1.dpad_right) {
                directionalTargetModifier = 1;
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
                servoPos = 1;
            } else if (gamepad2.y) {

            } else if (gamepad2.b) {

            } else if (gamepad2.a) {
                servoPos = 0.4;
            } else {
                servoPos = 0.7;
            }
            /* Gets dpad, breaks if block when a variable is found */
            if (gamepad2.dpad_up) {
            } else if (gamepad2.dpad_down) {
            } else if (gamepad2.dpad_left) {
            } else if (gamepad2.dpad_right) {
            }
            /* Gets Bumpers, Remove the else tag to allow both to run if you change functionality */
            if (gamepad2.right_bumper) {
                wenchPow = 1;
            } else if (gamepad2.left_bumper) {
                wenchPow = -1;
            } else {
                wenchPow = 0;
            }
            if (gamepad2.right_trigger > 0) {
                pulleyPow = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0) {
                pulleyPow = -gamepad2.left_trigger;
            } else {
                pulleyPow = 0;
            }

            /* Gets Center Buttons, These all hold key values and are misc, thus no skips by default */
            if (gamepad2.start) ;
            if (gamepad2.back) ;
            if (gamepad2.guide) ;
        }

        /**
         * This is a high level method intended to check all linearOpMode gamepads, you must implement your
         * own functionality into the massive internal IF block.  This particular checker is for the olympians.
         */
        void buttonCheckTitans() {
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
                directionalSnapModifier = 0;
            } else if (gamepad1.b) {
                directionalSnapModifier = 90;
            } else if (gamepad1.a) {
                directionalSnapModifier = 180;
            }
            /* Gets dpad, breaks if block when a variable is found */
            if (gamepad1.dpad_up) {
                directionalTargetModifier = 1;
            } else if (gamepad1.dpad_down) {
                directionalTargetModifier = 1;
            } else if (gamepad1.dpad_left) {
                directionalTargetModifier = 1;
            } else if (gamepad1.dpad_right) {
                directionalTargetModifier = 1;
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
                servoPos = 1;
            } else if (gamepad2.y) {

            } else if (gamepad2.b) {

            } else if (gamepad2.a) {
                servoPos = 0.4;
            } else {
                servoPos = 0.7;
            }
            /* Gets dpad, breaks if block when a variable is found */
            if (gamepad2.dpad_up) {
            } else if (gamepad2.dpad_down) {
            } else if (gamepad2.dpad_left) {
            } else if (gamepad2.dpad_right) {
            }
            /* Gets Bumpers, Remove the else tag to allow both to run if you change functionality */
            if (gamepad2.right_bumper) {
            } else if (gamepad2.left_bumper) {
            } else {
            }
            if (gamepad2.right_trigger > 0) {
                slidePow = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0) {
                slidePow = -gamepad2.left_trigger;
            } else {
                slidePow = 0;
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
            int pulseCount = (int) (cmDistance * config.PULSES_PER_CM);
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
                double turnSpeed = getHeadingCorrection(heldHeading, config.driveGain);
                if (cmDistance < 0) {
                    turnSpeed *= -1.0;
                }
                powSet(cmDistance, 0.0, turnSpeed);
            }


        }

        public void turnToHeading(double maxSpeed, double heldHeading) {

            // Run getSteeringCorrection() once to pre-calculate the current error
            getHeadingCorrection(heldHeading, config.driveGain);

            // keep looping while we are still active, and not at heading.
            while (opModeIsActive() && (Math.abs(config.headingError) > config.headingThreshold)) {

                // Determine required steering to keep on heading
                turnSpeed = getHeadingCorrection(heldHeading, config.driveGain);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -(config.maxTurnSpeed), (config.maxTurnSpeed));

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

        double getHeadingCorrection(double targetHeading, double gainFactor) {
            headingError = targetHeading - getImuOrientation().firstAngle;
            if (headingError > 180) headingError -= 360;
            if (headingError <= 180) headingError += 360;
            return Range.clip(headingError * gainFactor, -1, 1);
        }

        class debugMovementFunctions {
            int resolutionX;
            int resolutionY;
            String fileName;
            ElapsedTime internalTime = runtime;
            RoboCam roboCam;
            String fieldState = "none";
            String startState = config.startState;
            String allianceState = config.allianceState;
            int[] redAllianceTagArr = config.redAllianceTagArr;
            int[] blueAllianceTagArr = config.blueAllianceTagArr;
            int allianceAngleInverser;

            /**
             * Generates a debugMovementFunctions Instance,
             * said instance contains all our movement methods for autonomous,
             * it also contains any advanced methods intended for usage in manual control period.
             */
            public debugMovementFunctions(
                    int[] blueAllianceTags,
                    int[] redAllianceTags
            ) {
                this.blueAllianceTagArr = blueAllianceTags;
                this.redAllianceTagArr = redAllianceTags;
                switch (allianceState) {
                    case "red":
                        allianceAngleInverser = 1;
                    case "blue":
                        allianceAngleInverser = -1;
                }
                roboCam = new RoboCam(config.fileName, config.resolution, config.resolution);
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
                driveDistance(81, 1, 0);
                turnToHeading(1, 90);
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
                driveDistance(travelDistance, 1, 90);
            }

            void leg2_backboardScan() {
                // scan the backboard once we have landed there, align properly with that section of the backboard.
                AprilTagDetection currentDetection = roboCam.detectAprilTagById(1);
            }

            void leg2_deployPixel() {
                // deploy the pixel on the backboard.  pixels if you grabbed a white one.
            }

            void leg3_movement() {
                turnToHeading(1, 0);
                driveDistance(-66, 1, 0);
            }

            public void autoProtocol(String objectName) {
                // leg1_fieldScan(3,objectName);
                leg1_movement();
                leg1_deployPixel();
                leg2_movement();
                leg2_backboardScan();
                leg2_deployPixel();
                leg3_movement();
            }
        }

        class RoboCam extends Config {
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
                //  tFodProcessor = new TfodProcessor.Builder().setModelFileName(fileNameStr).build();
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