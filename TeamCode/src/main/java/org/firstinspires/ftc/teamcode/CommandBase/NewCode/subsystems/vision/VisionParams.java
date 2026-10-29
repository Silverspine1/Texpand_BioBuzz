package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

/**
 * Plain-data copy of every number the vision stack uses. The pure-Java core (BallCamera,
 * ClusterDetector, BallMap, CollectPlanner) only ever sees this class, so it can be unit
 * tested on a PC. {@link VisionSubsystem} owns the @Config fields you edit in FTC Dashboard
 * and copies them in here each loop.
 *
 * Units: cm, degrees, seconds. Robot frame: x forward, y LEFT, z up (origin = chassis centre).
 */
public final class VisionParams {

    // ---- camera (see VisionSubsystem for the descriptions) ----
    public int workWidth = 320, workHeight = 240;
    public double hfovDeg = 90;
    public double camForwardCm = 18, camLeftCm = 0, camHeightCm = 25, camPitchDownDeg = 40, camYawDeg = 0;
    public double ballRadiusCm = 3.556;            // POLLEN: 2.8 in diameter

    // ---- frame handedness: +1 = robot-left is (-sin h, cos h) in the odometry frame, -1 = mirrored ----
    public int lateralSign = -1;

    // ---- detector ----
    public double scoreMin = 0.30, newMin = 0.18, floorPenalty = 1.0, sepFrac = 0.80;
    public double radiusScale = 1.0, minRadiusPx = 2.2, maxRangeCm = 200, holeFrac = 0.30;
    public int maxBalls = 24;

    // ---- tracker ----
    public double gateCm = 5, memoryS = 3.0;
    public int confirmHits = 2, missDrop = 4;
    public double confirmCloseCm = 36, viewMarginPx = 10, minViewRangeCm = 15, maxViewRangeCm = 140;
    public double measSigma0Cm = 1.3, measSigmaPerCm = 0.018, poseSigmaCm = 0.6, calibK = 0.0005;
    public double maxDetRangeCm = 130, fieldHalfXCm = 179.5, fieldHalfYCm = 179.5, outsideRejectCm = 15;

    // ---- intake (relative to the chassis centre) ----
    public double intakeForwardCm = 38.1;     // where a ball starts being pulled in
    public double intakeWidthCm = 44.5;       // mouth width (informational)
    public double intakeSideTolCm = 15.0;     // a ball this far off the intake centreline is still taken
    public double intakeOffsetCm = 0;         // intake centreline offset, + = left of the chassis centre
    public double centerCostSPerCm = 0.005;   // prefers centred balls when it costs nothing
    public double[] lateralOptions = {0.0, 0.85};

    // ---- planner ----
    public double cruiseFrac = 0.90, ballValueS = 3.2, overrunCm = 6.0, wallMarginCm = 26.7;
    public double approachCm = 23, tangentScale = 0.38, turnHideFrac = 0.6, headingEff = 0.7;
    public int beam = 24, maxDepth = 4, consider = 8, nCandidates = 8;
    public double wallTouchCm = 15;           // a ball closer to a wall than this is taken head-on
    public double maxSpeedCmS = 180, maxAccelCmS2 = 250, headingRateDps = 360;
    public double replanS = 0.25, switchGainS = 0.6, minPlanS = 0.45;
    public double noTargetTimeoutS = 0.8, blacklistS = 3.5, stopDistCm = 8;
}
