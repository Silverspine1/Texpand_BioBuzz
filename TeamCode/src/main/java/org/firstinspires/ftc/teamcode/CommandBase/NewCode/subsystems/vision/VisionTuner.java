package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

/**
 * VISION TUNER - tune the ball colour and the camera model against live data (only the camera is needed).
 *
 *  Open FTC Dashboard (http://192.168.43.1:8080/dash). The camera stream starts on the plain camera; X switches to the
 *  yellow MASK and back (left stick button cycles all four views): every ball is circled with the distance the robot
 *  thinks it is away, the centre box shows the live HSV, and on the mask panel a CYAN circle (measured ball-blob radius) is compared
 *  with a RED circle (the size the camera model expects there). Configuration > VisionSubsystem holds every number.
 *
 *  gamepad1
 *    A   sample the ball colour: ball in the CENTRE box -> HSV window filled in
 *    B   save every VisionSubsystem value to /sdcard/FIRST/vision_tune.properties (loaded on every start)
 *    X   camera <-> mask      left stick button: next view (camera / mask / masked camera / side by side)
 *    D-pad up / down      CAM_HFOV_DEG  +1 / -1
 *    D-pad right / left   LENS_RECT_MIX +0.05 / -0.05  (0 = equidistant fisheye-like, 1 = pinhole)
 *    --- camera calibration (fits pitch, FOV and lens shape from balls at measured spots) ---
 *    Y   add a mark: ONE ball in view at the spot given by TUNER_MARK_FORWARD_CM / TUNER_MARK_LEFT_CM
 *        (cm from the CHASSIS CENTRE: forward, left +). Move the ball, edit the two numbers, press Y again.
 *    LB  fit the marks -> CAM_PITCH_DOWN_DEG, CAM_HFOV_DEG, LENS_RECT_MIX are set; RB clear the marks
 *    Back load the saved file again, Start delete the saved file
 *
 *  Quick check without marks: put ONE ball in the middle of the picture. If the cyan circle is clearly bigger than the
 *  red one, the model thinks balls are smaller than they are -> reduce CAM_HFOV_DEG (printed as "try HFOV"). Do the
 *  same with the ball near the left / right edge: if only the edges are wrong the lens shape (LENS_RECT_MIX) is off.
 *  Marks: use 5-8, some straight ahead at different distances and some well off to each side.
 */
@Config
@TeleOp(name = "Vision Tuner", group = "tuning")
public class VisionTuner extends LinearOpMode {

    /** Where the ball for the next calibration mark is, cm from the chassis centre. */
    public static double TUNER_MARK_FORWARD_CM = 60, TUNER_MARK_LEFT_CM = 0;
    /** Fit the lens mix as well as pitch and FOV (turn off to keep the lens shape fixed). */
    public static boolean FIT_LENS_MIX = true;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        VisionTuneFile.load();
        VisionSubsystem.SHOW_EXPECTED_SIZE = true;
        VisionSubsystem.VIEW = 0;
        BallVisionProcessor proc = new BallVisionProcessor();
        VisionPortal portal = VisionSubsystem.buildPortal(hardwareMap, proc);
        FtcDashboard.getInstance().startCameraStream(proc, 15);

        VisionParams params = VisionSubsystem.buildParams();
        double key = Double.NaN;
        boolean controls = false;
        String status = "";
        List<CameraFit.Mark> marks = new ArrayList<>();
        String fitText = "no fit yet";
        boolean lastA = false, lastB = false, lastX = false, lastY = false, lastUp = false, lastDown = false, lastLeft = false,
                lastRight = false, lastBack = false, lastStart = false, lastLb = false, lastRb = false, lastLs = false;

        telemetry.addLine("Vision Tuner ready. Dashboard: camera stream + Configuration > VisionSubsystem");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!controls) controls = VisionSubsystem.applyCameraControls(portal);
            params = VisionSubsystem.buildParams();
            double k = VisionSubsystem.geometryKey(params);
            if (k != key) {
                key = k;
                BallCamera cam = new BallCamera(params);
                proc.configure(cam, new ClusterDetector(cam), params);
            } else {
                proc.setParams(params);
            }

            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            boolean back = gamepad1.back, start = gamepad1.start, lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;
            boolean ls = gamepad1.left_stick_button;
            BallVisionProcessor.Result r = proc.latest();

            if (a && !lastA) {
                double[] s = proc.sampleCenterHsv();
                if (s == null) status = "A: the centre box is not on a ball colour - put a ball in the box and try again";
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
            if (x && !lastX) VisionSubsystem.VIEW = VisionSubsystem.VIEW == 1 ? 0 : 1;
            if (ls && !lastLs) VisionSubsystem.VIEW = (VisionSubsystem.VIEW + 1) % 4;
            if (up && !lastUp) VisionSubsystem.CAM_HFOV_DEG += 1;
            if (down && !lastDown) VisionSubsystem.CAM_HFOV_DEG -= 1;
            if (right && !lastRight) VisionSubsystem.LENS_RECT_MIX = Math.min(1, VisionSubsystem.LENS_RECT_MIX + 0.05);
            if (left && !lastLeft) VisionSubsystem.LENS_RECT_MIX = Math.max(0, VisionSubsystem.LENS_RECT_MIX - 0.05);
            if (y && !lastY) {
                if (r == null || r.blob == null) status = "Y: no ball blob in view";
                else if (r.blobCount > 1) status = "Y: " + r.blobCount + " separate blobs in view - leave ONE ball in view";
                else {
                    marks.add(new CameraFit.Mark(r.blob.u, r.blob.v, TUNER_MARK_FORWARD_CM, TUNER_MARK_LEFT_CM));
                    status = String.format("Y: mark %d added: pixel (%.0f, %.0f) = ball at %.0f fwd / %.0f left cm", marks.size(),
                            r.blob.u, r.blob.v, TUNER_MARK_FORWARD_CM, TUNER_MARK_LEFT_CM);
                }
            }
            if (rb && !lastRb) { marks.clear(); status = "RB: marks cleared"; }
            if (lb && !lastLb) {
                CameraFit.Result fr = CameraFit.fit(params, marks, FIT_LENS_MIX);
                if (fr.ok) {
                    VisionSubsystem.CAM_PITCH_DOWN_DEG = fr.pitchDeg;
                    VisionSubsystem.CAM_HFOV_DEG = fr.hfovDeg;
                    VisionSubsystem.LENS_RECT_MIX = fr.mix;
                }
                fitText = String.format("%s: error %.1f -> %.1f cm RMS; tilt %.1f, HFOV %.1f, lens mix %.2f", fr.note, fr.rmsBeforeCm, fr.rmsCm,
                        fr.pitchDeg, fr.hfovDeg, fr.mix);
                status = "LB: " + fitText;
            }
            if (back && !lastBack) status = "Back: loaded " + VisionTuneFile.load() + " saved values";
            if (start && !lastStart) status = VisionTuneFile.delete() ? "Start: saved file deleted" : "Start: no saved file";
            lastA = a; lastB = b; lastX = x; lastY = y; lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
            lastBack = back; lastStart = start; lastLb = lb; lastRb = rb; lastLs = ls;

            // where it thinks each ball is, first so it is never cut off the screen
            if (r == null) {
                telemetry.addLine("BALLS: no frame yet");
            } else {
                telemetry.addLine(String.format("BALLS (robot relative, from the chassis centre): %d seen", r.detections.size()));
                int n = 0;
                for (ClusterDetector.Detection d : r.detections) {
                    if (++n > 6) break;
                    telemetry.addLine(String.format("  #%d  %.1f cm forward  %+.1f cm left  |  range %.1f cm  bearing %+.0f deg  (%.1f cm past the intake)",
                            n, d.xr, d.yr, d.rangeCm(), d.bearingDeg(), d.xr - params.intakeForwardCm));
                }
            }
            String[] viewNames = {"camera", "MASK", "masked camera", "side by side"};
            telemetry.addData("view (X camera/mask, LS cycle)", viewNames[Math.max(0, Math.min(3, VisionSubsystem.VIEW))]);
            telemetry.addData("camera", portal.getCameraState());
            telemetry.addData("fps / detect ms", r == null ? "-" : String.format("%.1f / %.1f", r.fps, r.procMs));
            telemetry.addData("camera model", "HFOV %.1f deg  lens mix %.2f  tilt %.1f deg  height %.1f cm  forward %.1f cm", params.hfovDeg,
                    params.lensRectMix, params.camPitchDownDeg, params.camHeightCm, params.camForwardCm);
            telemetry.addData("HSV window", "H %.0f-%.0f  S %.0f-%.0f  V %.0f-%.0f", VisionSubsystem.H_LOW, VisionSubsystem.H_HIGH,
                    VisionSubsystem.S_LOW, VisionSubsystem.S_HIGH, VisionSubsystem.V_LOW, VisionSubsystem.V_HIGH);
            double[] raw = proc.centerRawHsv();
            if (raw == null) {
                telemetry.addData("centre HSV", "no frame yet");
            } else {
                boolean in = raw[0] >= VisionSubsystem.H_LOW && raw[0] <= VisionSubsystem.H_HIGH
                        && raw[1] >= VisionSubsystem.S_LOW && raw[1] <= VisionSubsystem.S_HIGH
                        && raw[2] >= VisionSubsystem.V_LOW && raw[2] <= VisionSubsystem.V_HIGH;
                telemetry.addData("centre HSV (box median)", "H %.0f  S %.0f  V %.0f   -> %s the window", raw[0], raw[1], raw[2], in ? "INSIDE" : "OUTSIDE");
            }
            double[] s = proc.sampleCenterHsv();
            telemetry.addData("centre ball colour (A uses this)", s == null ? "box is not on a ball (too dark / grey)"
                    : String.format("%.0f / %.0f / %.0f", s[0], s[1], s[2]));
            if (r != null && r.blob != null) {
                double ratio = r.blob.ratio();
                String hint = r.blobCount > 1 ? "several blobs: size check needs ONE ball"
                        : Math.abs(ratio - 1) < 0.08 ? "size OK"
                        : String.format("ball looks %s than the model expects -> try HFOV %.0f (mix 0)", ratio > 1 ? "BIGGER" : "SMALLER",
                                params.hfovDeg / ratio);
                telemetry.addData("ball size check", "measured r %.1f px  model r %.1f px  ratio %.2f  (%s)", r.blob.measuredR, r.blob.expectedR, ratio, hint);
            }
            telemetry.addData("calibration marks", "%d (Y add, LB fit, RB clear) next ball spot: %.0f fwd / %.0f left cm", marks.size(),
                    TUNER_MARK_FORWARD_CM, TUNER_MARK_LEFT_CM);
            telemetry.addData("last fit", fitText);
            telemetry.addData("status", status);
            if (r != null) {
                int i = 0;
                for (ClusterDetector.Detection d : r.detections) {
                    if (++i > 6) break;
                    telemetry.addData("#" + i + " detail", "r=%.0f px  unique %.2f", d.radiusPx, d.unique);
                }
            }
            telemetry.update();
            sleep(20);
        }
        FtcDashboard.getInstance().stopCameraStream();
        portal.close();
    }
}
