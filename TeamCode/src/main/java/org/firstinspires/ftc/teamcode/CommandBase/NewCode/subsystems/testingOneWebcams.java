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


        portal = null;
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        portal = builder.build();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(portal, 30);
        /*for (int attempt = 1; attempt <= 3 && !isStopRequested(); attempt++) {
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(proc)
                    .build();

            if (portal.getCameraState() != VisionPortal.CameraState.ERROR) break;

            telemetry.addData("Open attempt failed", attempt);
            telemetry.update();
            portal.close();
            sleep(1000);
        }
*/
        waitForStart();

        while (opModeIsActive())
        {
            Point c = proc.getCenter();
            telemetry.addData("Camera state", portal.getCameraState());
            telemetry.addData("Center", c == null ? "none" : c);
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.update();
            sleep(20);
        }


        portal.close();
    }
}