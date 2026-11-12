package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.PipelineLogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Single Webcam Blob Test", group = "Test")
public class testingOneWebcams extends LinearOpMode
{
    public static VisionPortal portal;

    public static FtcDashboard dashboard;

    ElapsedTime TimeSinceStart = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        pipeline proc = new pipeline();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(proc)
                .build();
        proc.setRobotPose(0,0,0);


        // Stream the processor's annotated frames, not the raw portal frames
        dashboard.startCameraStream(proc, 30);


        waitForStart();

        TimeSinceStart.startTime();

        while (opModeIsActive())
        {
            Point c = proc.getCenter();

            if (TimeSinceStart.milliseconds()<1000){
                setManualExposure(5,100);
            }

            double[] centerHsv = proc.getCenterHsv();
            double[] targetHsv = proc.getTargetHsv();
            double[] xy = proc.getRobotXY();
            proc.setRobotPose(0,0,0);
            proc.getOptimalPath();

     /*       double cx = 0, cy = 0;

            if (centerHsv != null) {
                cx = centerHsv[1];
                cy = centerHsv[2];
            }*/


            telemetry.addData("Camera state", portal.getCameraState());
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.addData("Center px", c == null ? "none" : c);
            telemetry.addData("Image-center HSV", centerHsv == null ? "none"
                    : String.format("%.0f %.0f %.0f", centerHsv[0], centerHsv[1], centerHsv[2]));
            telemetry.addData("Target HSV", targetHsv == null ? "none"
                    : String.format("%.0f %.0f %.0f", targetHsv[0], targetHsv[1], targetHsv[2]));
            telemetry.addData("Robot-relative (fwd, left) in", xy == null ? "none"
                    : String.format("%.1f, %.1f", xy[0], xy[1]));
            telemetry.addData("ConturesLength: ", ((PipelineLogger)proc).getContours().size());
          /*  if (!((PipelineLogger)proc).getContours().isEmpty()){
                telemetry.addData("Contures: ", ((PipelineLogger)proc).getContours().get(0));
            }*/
          //  telemetry.addData("cx: ",cx);
         //   telemetry.addData("cy: ",cy);
            telemetry.update();
            //sleep(20);
        }

        dashboard.stopCameraStream();
        portal.close();
    }
    private void setManualExposure(int exposureMS, int gain) {

        if (portal.getCameraState() !=
                VisionPortal.CameraState.STREAMING) {
            return;
        }

        ExposureControl exposure =
                portal.getCameraControl(
                        ExposureControl.class
                );

        if (exposure.isModeSupported(ExposureControl.Mode.Manual)) {
            exposure.setMode(ExposureControl.Mode.Manual);
        }

        sleep(50);

        exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl =
                portal.getCameraControl(GainControl.class);

        gainControl.setGain(gain);
    }
}
