package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Disabled

@TeleOp(name = "OpenCV Testing - RED Blob")
public class OpenCVExampleREDBlob extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;
    public String path;


    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK

    // Constants for camera resolution
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // Constants for distance calculation
    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;

    @Override
    public void runOpMode() {
        initOpenCV();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", getDistance(width));
            telemetry.addData("",path);
            telemetry.update();
        }

        // Release resources
        controlHubCam.closeCameraDevice();
    }

    private void initOpenCV() {
        // gets the camera monitor view id of the camera itself from the application.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // creates an OpenCvCamera instance from the webcams hardware map, and its monitor view id.
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // Declares the cameras pipeline output as being an instance of the blob detection pipeline.
        // for context we can decide to do whatever we want with the output "pipeline"
        // the blue blob detection pipeline is just a destination for processing the image to our needs.
        controlHubCam.setPipeline(new BlueBlobDetectionPipeline());
        // Tells the camera to start streaming at the resoulution we require,
        // also nicely implements a fun little checkbox for if the camera is say.. upside down!!
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class BlueBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect blue regions
            Mat redmask = preprocessFrame(input); // Detects, defines, and creates objects on every section of color that meets internal criteria.

            // Find contours of the detected blue regions
            List<MatOfPoint> contours = new ArrayList<>();  // MatOfPoint is the object type, is var name
            // hierarchy is a optional expansion upon the hierarchy of the resulting contours.
            Mat hierarchy = new Mat();
            // finds the contours of a given binary image, the contours are the individual points that make up a given shape,
            // its notable to state that the points on the outside of a given shape are ALL possible points about there line.
            // chain_approx_Simple only gets critical points, ec the ones at vertex's or along arcs to a point.
            // imgproc is again a tag value used to call a tagged process
            // returns a list of lists of individual contours which init of themselves are made up of vectors.
            Imgproc.findContours(redmask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest blue contour (blob)
            // finds the largest contour from redmask, ec the one with the greatest area.
            // this is likely to be OUR object.
            MatOfPoint largestContour = findLargestContour(contours);
            // when we actually have a largestContour, follow procedure.
            if (largestContour != null) {
                // Draw a blue outline around the largest detected object
                // this should be self explanitory, it draws lines between each individual point in the contour in order, outlining our shape.
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 0, 255), 2);


                // Calculates the width of the box which contains the following text.
                width = calculateWidth(largestContour);

                // Displays the width
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);

                // Displays the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);

                // Calculate the centroid of the largest contour
                // I dont know man.
                // takes a raster image and completes contour moment calculations, contour area arc length, refuses to elaborate, lmao what.
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();
                // works unless it doesnt, this is posistion in space, not color.
                if ((int)cX<CAMERA_WIDTH/3){
                    path = "LEFT";
                }
                else   if ((int)cX>=(CAMERA_WIDTH/3) && (int)cX<=(2*CAMERA_WIDTH/3)) {
                    path = "CENTER";
                }
                else   if ((int)cX>(2*CAMERA_WIDTH/3)){
                    path = "RIGHT";
                }
                else { path = "NONE"; }

                // Draw a dot at the centroid
                // doesnt seem to work??
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 0, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(255, 0, 0), -1);
            }

            return input;
        }

        /**
         *
         * @param frame - The original image from the camera in this instance.
         * @return A mat file containing a mask fit to the previous frame, the mask contains a covering for
         * the passed mat, which can be used in processing this particular section of the image.
         */
        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat(); // Mat for storing converted frame.
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV); // converts frame from
            // Blue Red Green to Hue Saturation Value, effectively turning the frame from a colored frame to
            // a frame with differing levels of brightness.
            Scalar lowerred = new Scalar(90, 100, 100); // lowest allowed values to be detected.
            Scalar upperred = new Scalar(130, 255, 255); // highest allowed values to be detected.

            Mat redmask = new Mat(); // a frame which when combined with our HSV frame applies a RED overlay.
            Core.inRange(hsvFrame, lowerred, upperred, redmask); /*
            Imagine an image file as a matrix of values, containing rows and columns.
            Now imagine each individual bit in this matrix has a value.
            If any of these bits fall within our range, ec > than lower < than higher
            Outputs into redmask, so basically all detections from the hsvFrame that fall within the query are stored in the mask
            */
                // The following are a collection of morphological operations, the Imgproc.whatever is
                // effectively a pointer used by the method to decide if it feels like doing a certain operation.
            // this gets a kernel, which is basically a fake shape generated and anchored at its center on the frame
            // this "Fake Object" is then used to to complete operations from redmask to redmask.
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redmask, redmask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redmask, redmask, Imgproc.MORPH_CLOSE, kernel);

            return redmask;
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

    private static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
}