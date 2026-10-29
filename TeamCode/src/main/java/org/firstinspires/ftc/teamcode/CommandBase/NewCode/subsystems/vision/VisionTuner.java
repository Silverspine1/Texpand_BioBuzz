package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * VISION TUNER - tune the camera and the ball colour against live data, no robot hardware needed except the camera.
 *
 *  Open FTC Dashboard (http://192.168.43.1:8080/dash): the camera stream shows every ball circled with the
 *  distance the robot thinks it is away; Configuration > VisionSubsystem holds every number (camera FOV, mount,
 *  HSV, intake width and side tolerance, ...). Edits apply live.
 *
 *  gamepad1:
 *    A  sample the ball colour: put a ball in the CENTRE of the picture, press A - HSV low/high are filled in
 *    B  save every VisionSubsystem value to /sdcard/FIRST/vision_tune.properties (loaded on every start)
 *    X  next view: camera + circles -> yellow mask -> camera through the mask
 *    Y  calibrate the camera tilt from a known distance: put ONE ball TUNER_KNOWN_FORWARD_CM in front of the chassis
 *       centre (measure it!), press Y - CAM_PITCH_DOWN_DEG is adjusted so the reported distance matches.
 *       Repeat at a second distance: if the two still disagree, CAM_HEIGHT_CM is wrong - nudge it.
 *    D-pad up/down  CAM_HFOV_DEG +/- 1  (a wrong FOV shows as sideways error that grows with bearing)
 *    Back  load the saved file again,   Start  delete the saved file
 *
 *  What to check: the list below the picture gives, for each ball, forward and left in cm FROM THE CHASSIS CENTRE.
 *  Measure a few with a tape. Forward wrong -> tilt/height (Y). Left wrong, worse at the edges -> FOV.
 */
@Config
@TeleOp(name = "Vision Tuner", group = "tuning")
public class VisionTuner extends LinearOpMode {

    /** Distance from the chassis centre to the ball centre, cm, for the Y calibration. */
    public static double TUNER_KNOWN_FORWARD_CM = 60;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        VisionTuneFile.load();
        BallVisionProcessor proc = new BallVisionProcessor();
        VisionPortal portal = VisionSubsystem.buildPortal(hardwareMap, proc);
        FtcDashboard.getInstance().startCameraStream(proc, 15);

        VisionParams params = VisionSubsystem.buildParams();
        BallCamera cam = new BallCamera(params);
        double key = Double.NaN;
        boolean controls = false;
        String status = "";
        boolean lastA = false, lastB = false, lastX = false, lastY = false, lastUp = false, lastDown = false, lastBack = false, lastStart = false;

        telemetry.addLine("Vision Tuner ready. Dashboard: camera stream + Configuration > VisionSubsystem");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!controls) controls = VisionSubsystem.applyCameraControls(portal);
            params = VisionSubsystem.buildParams();
            double k = params.workWidth + params.workHeight * 3.1 + params.hfovDeg * 7.3 + params.camForwardCm * 11.7
                    + params.camLeftCm * 13.1 + params.camHeightCm * 17.9 + params.camPitchDownDeg * 19.3 + params.camYawDeg * 23.7
                    + params.ballRadiusCm * 29.1 + params.scoreMin * 37 + params.newMin * 41 + params.floorPenalty * 43
                    + params.sepFrac * 47 + params.holeFrac * 53 + params.maxDetRangeCm * 59;
            if (k != key) {
                key = k;
                cam = new BallCamera(params);
                proc.configure(cam, new ClusterDetector(cam), params);
            } else {
                proc.setParams(params);
            }

            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, back = gamepad1.back, start = gamepad1.start;
            if (a && !lastA) {
                double[] s = proc.sampleCenterHsv();
                if (s == null) status = "A: no ball colour in the centre box - centre a ball and try again";
                else {
                    VisionSubsystem.H_LOW = Math.max(0, s[0] - VisionSubsystem.AUTO_H_MARGIN);
                    VisionSubsystem.H_HIGH = Math.min(179, s[0] + VisionSubsystem.AUTO_H_MARGIN);
                    VisionSubsystem.S_LOW = Math.max(0, s[1] - VisionSubsystem.AUTO_S_MARGIN);
                    VisionSubsystem.S_HIGH = 255;
                    VisionSubsystem.V_LOW = Math.max(0, s[2] - VisionSubsystem.AUTO_V_MARGIN);
                    VisionSubsystem.V_HIGH = 255;
                    status = String.format("A: sampled HSV %.0f/%.0f/%.0f -> H %.0f-%.0f S>%.0f V>%.0f", s[0], s[1], s[2],
                            VisionSubsystem.H_LOW, VisionSubsystem.H_HIGH, VisionSubsystem.S_LOW, VisionSubsystem.V_LOW);
                }
            }
            if (b && !lastB) {
                try {
                    VisionTuneFile.save();
                    status = "B: saved to " + VisionSubsystem.TUNE_FILE;
                } catch (Exception e) {
                    status = "B: SAVE FAILED " + e.getMessage();
                }
            }
            if (x && !lastX) BallVisionProcessor.VIEW = (BallVisionProcessor.VIEW + 1) % 3;
            if (y && !lastY) status = calibratePitch(proc, params, cam);
            if (up && !lastUp) VisionSubsystem.CAM_HFOV_DEG += 1;
            if (down && !lastDown) VisionSubsystem.CAM_HFOV_DEG -= 1;
            if (back && !lastBack) status = "Back: loaded " + VisionTuneFile.load() + " saved values";
            if (start && !lastStart) status = VisionTuneFile.delete() ? "Start: saved file deleted" : "Start: no saved file";
            lastA = a; lastB = b; lastX = x; lastY = y; lastUp = up; lastDown = down; lastBack = back; lastStart = start;

            BallVisionProcessor.Result r = proc.latest();
            telemetry.addData("camera", portal.getCameraState());
            telemetry.addData("fps / detect ms", r == null ? "-" : String.format("%.1f / %.1f", r.fps, r.procMs));
            telemetry.addData("view (X)", BallVisionProcessor.VIEW);
            telemetry.addData("FOV / tilt / height", "%.1f deg / %.1f deg / %.1f cm", params.hfovDeg, params.camPitchDownDeg, params.camHeightCm);
            telemetry.addData("HSV window", "H %.0f-%.0f  S %.0f-%.0f  V %.0f-%.0f", VisionSubsystem.H_LOW, VisionSubsystem.H_HIGH,
                    VisionSubsystem.S_LOW, VisionSubsystem.S_HIGH, VisionSubsystem.V_LOW, VisionSubsystem.V_HIGH);
            double[] s = proc.sampleCenterHsv();
            telemetry.addData("centre colour (HSV)", s == null ? "no ball colour in the centre box" : String.format("%.0f / %.0f / %.0f", s[0], s[1], s[2]));
            telemetry.addData("status", status);
            if (r != null) {
                telemetry.addData("balls seen", r.detections.size());
                int i = 0;
                for (ClusterDetector.Detection d : r.detections) {
                    i++;
                    telemetry.addData("#" + i, "fwd %.1f  left %+.1f  range %.1f cm  bearing %+.1f deg  r=%.0f px  score %.2f",
                            d.xr, d.yr, d.rangeCm(), d.bearingDeg(), d.radiusPx, d.score);
                    if (i >= 8) break;
                }
            }
            telemetry.update();
            sleep(20);
        }
        FtcDashboard.getInstance().stopCameraStream();
        portal.close();
    }

    /** Adjust CAM_PITCH_DOWN_DEG so the nearest ball reads TUNER_KNOWN_FORWARD_CM forward. */
    private String calibratePitch(BallVisionProcessor proc, VisionParams params, BallCamera cam) {
        BallVisionProcessor.Result r = proc.latest();
        if (r == null || r.detections.isEmpty()) return "Y: no ball in view";
        ClusterDetector.Detection d = r.detections.get(0);
        double lo = VisionSubsystem.CAM_PITCH_DOWN_DEG - 12, hi = VisionSubsystem.CAM_PITCH_DOWN_DEG + 12;
        // the pixel row of the ball is what was measured; find the pitch whose ground point for that pixel is the known distance
        for (int it = 0; it < 40; it++) {
            double mid = 0.5 * (lo + hi);
            VisionParams t = VisionSubsystem.buildParams();
            t.camPitchDownDeg = mid;
            double[] g = new BallCamera(t).groundPoint(d.u, d.v, t.ballRadiusCm);
            double fwd = g == null ? 1e9 : g[0];
            // looking further down (bigger pitch) pulls the ground point toward the robot
            if (fwd > TUNER_KNOWN_FORWARD_CM) lo = mid; else hi = mid;
        }
        double before = VisionSubsystem.CAM_PITCH_DOWN_DEG;
        VisionSubsystem.CAM_PITCH_DOWN_DEG = 0.5 * (lo + hi);
        return String.format("Y: tilt %.1f -> %.1f deg (ball at %.0f cm forward)", before, VisionSubsystem.CAM_PITCH_DOWN_DEG, TUNER_KNOWN_FORWARD_CM);
    }
}
