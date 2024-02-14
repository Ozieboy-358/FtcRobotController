package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;
@Disabled

@TeleOp
public class PipelineRecordingExample extends LinearOpMode {
    OpenCvWebcam Webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Webcam.setPipeline(new Pipeline());
        Webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Frame Count", Webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", Webcam.getFps()));
            telemetry.addData("Total frame time ms", Webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", Webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", Webcam.getOverheadTimeMs());
            telemetry.addData("Theoritical max fps", Webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.a) {
                Webcam.stopStreaming();
            }
            sleep(100);
        }
    }

    class Pipeline extends OpenCvPipeline {
        boolean vieportPaused;

        public Mat processFrame(Mat input) {

            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);
            return input;
        }


        public void onViewportTapped() {
            vieportPaused = !vieportPaused;

            if (vieportPaused) {
                Webcam.pauseViewport();
            } else {
                Webcam.resumeViewport();
            }
        }
    }
}
