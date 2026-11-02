package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ReducedShotController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ReducedShotModel;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShotFrame;

import java.util.Locale;
import java.util.function.BiConsumer;

/**
 * What the shot controller THINKS, in plain numbers, for the Shot Tuner and Test_tele: where the robot is (odometry, the
 * start pose that was assigned, the team-frame pose the targeter actually uses), where the goal is, how far and in which
 * direction, which way the turret should point against where it points, and the flywheel (direction, power, ticks/s,
 * rpm, ceiling, battery). Every line is one caption / value pair handed to {@code out} (driver station + dashboard).
 * Pure reads: it never commands anything.
 */
public final class ShotDebug {

    private static final ShotFrame FRAME = new ShotFrame();

    private ShotDebug() {
    }

    private static String f(String fmt, Object... a) {
        return String.format(Locale.US, fmt, a);
    }

    private static double wrap180(double a) {
        return a - 360.0 * Math.floor((a + 180.0) / 360.0);
    }

    /** The few lines that matter while shooting, the same in Shot Tuner and Test_tele. {@code feed} / {@code msg} are the OpMode's own texts. */
    public static void compact(Odometry odo, ShooterController sc, String feed, String msg, BiConsumer<String, String> out) {
        final ReducedShotController r = sc.reduced;
        StringBuilder warn = new StringBuilder();
        if (!r.hasModel()) warn.append("NO MODEL: ").append(sc.reducedStatus).append("   ");
        if (sc.flywheelBackwards) warn.append("FLYWHEEL BACKWARDS (ShotControlTuning.shooterMotorReversed)   ");
        if (odo.isPosePending()) warn.append("START POSE NOT WRITTEN (pinpoint ").append(odo.pinpointStatus()).append(")   ");
        if (!ShotControlTuning.gateProtect) warn.append("MOVEMENT PROTECTION OFF   ");
        String problem = ShotControlTuning.settingsProblem();
        if (problem != null) warn.append("SETTINGS: ").append(problem);
        if (warn.length() > 0) out.accept("!!! WARNING", warn.toString().trim());
        out.accept("SHOOTING", f("targeting %s | gate %s | P(make) %.2f", sc.targeting ? "ON" : "OFF", gateText(r), r.feasible ? r.pMake : 0.0));
        out.accept("feed", feed);
        out.accept("pose", f("X %.1f  Y %.1f cm   heading %.1f deg", odo.X(), odo.Y(), odo.Heading()));
        if (r.hasModel()) {
            out.accept("range to goal", f("%.0f cm  (%.1f in)", r.result.d * ShotFrame.CM_PER_IN, r.result.d));
            out.accept("turret  cmd / now / err", f("%.1f / %.1f / %.1f deg", r.turretJavaDeg, sc.getTurretAimDeg(), r.lastTurretErr));
            out.accept("wheel  rpm now / cmd", f("%.0f / %.0f  %s", sc.RPM, r.rpmCmd, sc.flywheelReady() ? "READY" : ""));
            out.accept("hood", f("%.1f of 6.3 deg -> servo %.3f%s", sc.hoodTargetDeg, sc.hoodServoPos,
                    ShotControlTuning.hoodDebug ? "   HOOD DEBUG (gamepad2 Y = off)" : ""));
        }
        if (msg != null && !msg.isEmpty()) out.accept("last action", msg);
    }

    private static String gateText(ReducedShotController r) {
        return ReducedShotController.GATE_TEXT[r.gateState]
                + (r.gateState == ReducedShotController.GATE_UNSETTLED ? " (" + ReducedShotController.REFUSE_TEXT[r.gateRefuse] + ")" : "");
    }

    /** The goal block: ALWAYS the last thing on the screen. */
    public static void goalBlock(ShooterController sc, BiConsumer<String, String> out) {
        final ReducedShotController r = sc.reduced;
        out.accept("=== GOAL (global field cm; alliance " + (ShotControlTuning.blueAlliance ? "BLUE" : "RED") + ", close cell) ===", "");
        if (!r.hasModel()) {
            out.accept("GOAL  X / Y", "no model loaded");
        } else {
            out.accept("GOAL  X / Y", f("%.1f / %.1f cm   (cell %.1f / %.1f, trim %+.1f / %+.1f)", r.goalXcm, r.goalYcm,
                    r.goalDefaultXcm, r.goalDefaultYcm, ShotControlTuning.goalTrimXcm, ShotControlTuning.goalTrimYcm));
        }
        out.accept("HOOD ANGLE boost", f("%+.1f deg at the top hood stop (real shot steeper than the model by this; 0 = model as built).  gamepad2 RIGHT STICK UP = +%.1f (close shots too high / short), DOWN = -%.1f  (hold LEFT TRIGGER = %.1f)",
                ShotControlTuning.hoodAngleBoostDeg, ShotControlTuning.hoodAngleStepDeg, ShotControlTuning.hoodAngleStepDeg, ShotControlTuning.hoodAngleStepCoarseDeg));
        out.accept("gamepad2", f("D-pad up/down = X +-%.0f   left/right = Y +-%.0f cm   (hold LEFT TRIGGER = +-%.0f)   BACK = trim 0   RIGHT BUMPER = alliance   START = save",
                ShotControlTuning.goalStepCm, ShotControlTuning.goalStepCm, ShotControlTuning.goalStepCoarseCm));
    }

    /** Odometry, assigned start pose, team-frame pose, goal geometry and turret check. */
    public static void pose(Odometry odo, ShooterController sc, BiConsumer<String, String> out) {
        final boolean blue = false;                    // the goal is a global point: the pose is never rotated for an alliance
        final double x = odo.X(), y = odo.Y(), h = odo.Heading();
        out.accept("--- WHERE IT THINKS IT IS ---", "");
        out.accept("odometry X / Y / heading", f("%.1f cm / %.1f cm / %.1f deg   (%.1f, %.1f in)", x, y, h, x / 2.54, y / 2.54));
        out.accept("pinpoint", odo.pinpointStatus() + (odo.isPosePending()
                ? "   !!! start pose NOT written yet (waiting for READY): the numbers above are the REQUESTED pose" : "   (pose live)"));
        out.accept("assigned start pose", f("%.1f, %.1f cm @ %.1f deg   (ShooterTuning startXcm / startYcm / startHeadingDeg)",
                ShooterTuning.startXcm, ShooterTuning.startYcm, ShooterTuning.startHeadingDeg));
        double dh = wrap180(h - ShooterTuning.startHeadingDeg);
        out.accept("moved since start", f("dx %.1f cm  dy %.1f cm  dheading %.1f deg   (all ~0 before you move)",
                x - ShooterTuning.startXcm, y - ShooterTuning.startYcm, dh));
        out.accept("frame", f("global field frame (the pose is never rotated; the alliance picks the close goal cell)  LATERAL_SIGN=%d", ShotFrame.LATERAL_SIGN));
        FRAME.toTeam(blue, x, y, h, 0, 0, 0, 0, 0);
        out.accept("turret zero", ShooterTuning.assumeTurretForwardAtStart
                ? "ASSUMED straight forward at start (zeroed at INIT and at PLAY; ShooterTuning.assumeTurretForwardAtStart)"
                : f("saved offset %.1f deg (assumeTurretForwardAtStart is OFF)", ShooterTuning.turretZeroOffsetDeg));
        out.accept("team-frame pose (what the targeter uses)", f("x %.1f in  y %.1f in  heading %.1f deg", FRAME.x, FRAME.y, FRAME.headingDeg));

        ReducedShotModel m = sc.reduced.model();
        if (m == null) {
            out.accept("goal", "no model loaded: " + sc.reducedStatus);
            return;
        }
        // turret axis in the team frame (the pivot offset is rotated by the heading; team "right" = heading + 90 deg)
        double ph = Math.toRadians(FRAME.headingDeg);
        double pivotFwd = ShotControlTuning.pivotFromShooterTuning ? ShooterTuning.turretAxisForwardMeters / ShotFrame.M_PER_IN : m.pivotFwdIn;
        double pivotRight = ShotControlTuning.pivotFromShooterTuning ? 0.0 : m.pivotRightIn;
        if (ShotControlTuning.pivotFromShooterTuning) {
            double[] piv = new double[2];
            ShotFrame.pivotFromJava(ShooterTuning.turretAxisForwardMeters, ShooterTuning.turretAxisSidewaysMeters, piv);
            pivotFwd = piv[0];
            pivotRight = piv[1];
        }
        double px = FRAME.x + pivotFwd * Math.cos(ph) - pivotRight * Math.sin(ph);
        double py = FRAME.y + pivotFwd * Math.sin(ph) + pivotRight * Math.cos(ph);
        double dx = m.aimX - px, dy = m.aimY - py;
        double range = Math.hypot(dx, dy);
        double bearing = Math.toDegrees(Math.atan2(dy, dx));
        double turretTeam = wrap180(bearing - FRAME.headingDeg);
        double turretJava = ShotFrame.turretTeamToJava(turretTeam);
        out.accept("--- WHERE THE GOAL IS ---", "");
        out.accept("goal aim point", f("x %.1f cm  y %.1f cm  z %.1f in   (in the red team frame: %.1f / %.1f in)",
                m.aimX * ShotFrame.CM_PER_IN, m.aimY * ShotFrame.CM_PER_IN, m.aimZ, m.aimX, m.aimY));
        out.accept("turret axis (team frame)", f("x %.1f in  y %.1f in   (offset %.1f fwd / %.1f right from the robot centre)", px, py, pivotFwd, pivotRight));
        out.accept("to goal: dx / dy / RANGE", f("%.1f / %.1f in   RANGE %.1f in  (%.2f m)   field bearing %.1f deg", dx, dy, range, range * ShotFrame.M_PER_IN, bearing));
        out.accept("straight-line turret angle", f("should point %.1f deg (robot-relative, Java sign)", turretJava));
        double meas = sc.getTurretAimDeg();
        out.accept("turret measured / solve cmd", f("%.1f / %.1f deg   straight-line minus measured %.1f deg   (solve cmd also includes the lead for motion)",
                meas, sc.reduced.turretJavaDeg, wrap180(turretJava - meas)));
    }

    /** Flywheel: direction, power, raw ticks, rpm, target, ceiling, battery, and the checks that find a wrong direction or scale. */
    public static void flywheel(ShooterController sc, BiConsumer<String, String> out) {
        final ReducedShotController r = sc.reduced;
        out.accept("=== MODEL ===", !r.hasModel()
                ? ">>> NO MODEL: " + sc.reducedStatus + "  -> flywheel OFF, turret not driven <<<"
                : "OK, rpm / hood / turret come from it: " + sc.reducedStatus);
        out.accept("turret gains", sc.turretTuneSource);
        out.accept("--- FLYWHEEL ---", "");
        if (sc.flywheelBackwards) {
            out.accept("!!! BACKWARDS", "power is positive but the wheel reads NEGATIVE rpm: flip ShotControlTuning.shooterMotorReversed "
                    + "(Dashboard), or the encoder sign is inverted");
        }
        out.accept("motor direction", sc.appliedReversed ? "REVERSE" : "FORWARD");
        out.accept("power / ticks per s", f("%.3f / %.0f   (ticks per rev used: %.0f)", sc.flywheelPower, sc.flywheelTicksPerSecond,
                ShooterTuning.flywheelTicksPerRev));
        out.accept("rpm measured / target", f("%.0f / %.0f   ready window +-%.0f%%  %s", sc.RPM, sc.getTargetRPM(),
                ShooterTuning.flywheelReadyPercent, sc.flywheelReady() ? "READY" : ""));
        out.accept("feed-forward power at target", f("%.3f  (kF %.6f x target; steady power should be near this)",
                ShooterTuning.flywheelKF * sc.getTargetRPM(), ShooterTuning.flywheelKF));
        double ceiling = ShotControlTuning.rpmMaxFromBattery && sc.batteryVolts > 6.0
                ? ShotControlTuning.rpmMaxNoLoadFraction * ShotControlTuning.wheelNoLoadRpmAt12V * sc.batteryVolts / 12.0 : 0.0;
        out.accept("battery / wheel ceiling", f("%.2f V / %.0f rpm  (%.2f x %.0f x V/12; the solve never asks for more)", sc.batteryVolts, ceiling,
                ShotControlTuning.rpmMaxNoLoadFraction, ShotControlTuning.wheelNoLoadRpmAt12V));
        if (r.hasModel()) {
            out.accept("rpm cmd (real) / model", f("%.0f = model %.0f x rpmScale %.2f   max %.0f", r.rpmCmd, r.rpmModelCmd,
                    r.rpmScaleUsed, r.rpmMaxEff));
            out.accept("moving compensation", f("rpm/hood solved %.0f ms ahead (mechDelayS)   accel %.0f, %.0f in/s^2 (%s)",
                    1000.0 * ShotControlTuning.mechDelayS, r.accelX, r.accelY, ShotControlTuning.useAcceleration ? "used" : "OFF"));
            out.accept("exit angle wanted (model) / offset", f("%.1f deg / %+.1f deg  (hood %.2f of 0..6.3)", r.result.angle,
                    ShotControlTuning.angleOffsetDeg, r.hoodDeg));
            out.accept("hood cmd", f("%.2f of 6.3 deg travel at range %.0f in -> servo %.4f%s  (rises inside ~54 in, fully up inside ~33 in)",
                    r.hoodDeg, r.result.d, sc.hoodServoPos, ShotControlTuning.hoodDebug ? "  HOOD DEBUG" : ""));
            out.accept("hood ends", ShotControlTuning.hoodLine());
        }
        out.accept("SCALE CHECK", "hold gamepad2 RIGHT TRIGGER (Shot Tuner) / Test_tele R-trigger = full power with the turret idle: no-load rpm "
                + "should read ~5700-6000. About HALF = flywheelTicksPerRev is 2x too big (or 2 motors / a 2:1 ratio); about DOUBLE = too small.");
    }
}
