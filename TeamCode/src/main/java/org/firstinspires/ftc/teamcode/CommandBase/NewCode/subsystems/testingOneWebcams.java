package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@TeleOp(name = "Single Webcam Blob Test", group = "Test")
public class testingOneWebcams extends LinearOpMode
{
    public static VisionPortal portal;

    public static FtcDashboard dashboard;

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

        // Stream the processor's annotated frames, not the raw portal frames
        dashboard.startCameraStream(proc, 30);

        waitForStart();

        while (opModeIsActive())
        {
            Point c = proc.getCenter();
            double[] centerHsv = proc.getCenterHsv();
            double[] targetHsv = proc.getTargetHsv();
            double[] xy = proc.getRobotXY();

            telemetry.addData("Camera state", portal.getCameraState());
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.addData("Center px", c == null ? "none" : c);
            telemetry.addData("Image-center HSV", centerHsv == null ? "none"
                    : String.format("%.0f %.0f %.0f", centerHsv[0], centerHsv[1], centerHsv[2]));
            telemetry.addData("Target HSV", targetHsv == null ? "none"
                    : String.format("%.0f %.0f %.0f", targetHsv[0], targetHsv[1], targetHsv[2]));
            telemetry.addData("Robot-relative (fwd, left) in", xy == null ? "none"
                    : String.format("%.1f, %.1f", xy[0], xy[1]));
            telemetry.update();
            sleep(20);
        }

        dashboard.stopCameraStream();
        portal.close();
    }
}
