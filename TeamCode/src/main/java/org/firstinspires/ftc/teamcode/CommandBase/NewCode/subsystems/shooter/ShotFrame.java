package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * The ONE place where the Java robot pose is converted into the shot-control TEAM frame
 * (docs/SHOT_CONTROL_PLAN.md "Coordinate system"; Python twin: {@code ftc_optimizer.shotcontrol.field_frame}).
 *
 * <p>TEAM frame (what {@link ReducedTargeter} uses): inches, origin at the alliance's audience-side LEFT corner
 * seen from the audience, +x audience -> back wall, +y to the right seen from the audience, headings from +x toward
 * +y (left-handed). Blue = red rotated 180 deg about the field centre: (x, y, h) -> (W - x, W - y, h + 180).
 *
 * <p>JAVA frame (what {@code Odometry} reports), as read from the code (2026-11-01, see docs/SHOT_ALGO_DESIGN.md
 * section 11 for the line-level evidence):
 * <ul>
 * <li>units: field cm, heading deg ({@code Odometry.X()/Y()/Heading()}), velocities cm/s (the shooter's
 *     {@code PlannerInputFilter} works in m/s, deg/s);</li>
 * <li>handedness: {@code X = startX - pinpointX, Y = startY + pinpointY, Heading() = 360 - raw} is one reflection of
 *     the Pinpoint's right-handed frame, so heading grows toward +Y and the robot's right is the forward vector
 *     rotated +90 deg in coordinates - the same handedness as TEAM. Agrees with the Follower strafe maths and
 *     {@code VisionSubsystem.LATERAL_SIGN = -1}: {@link #LATERAL_SIGN} = -1;</li>
 * <li>origin / axes: ONE fixed field frame for both alliances, equal to the RED team frame (the blue HIVE is
 *     at Y = 2.15 m > W/2 with its audience cell at X = 1.22 m facing 180 deg in {@code ShooterTuning} /
 *     {@code Shotplanner.getBlueHiveTarget}; in the red team frame the blue cells are at y = 2.12 m, x = 1.26 /
 *     2.21 m): {@link #JAVA_FRAME_IS_FIXED_RED} = true. Blue therefore goes through the 180 deg rotation here.</li>
 * </ul>
 * Every sign choice is one of the two named switches below; nothing else in the shot code flips a sign.
 * Pure Java, no allocation.
 */
public final class ShotFrame {

    /** Field inner width [in] the model was built for (= Python {@code api.FIELD_WIDTH_IN}, CAD inner faces of the walls). */
    public static final double W_MODEL_IN = 141.348;
    /** Real field inner width [in] (ShotControlTuning.fieldWidthCm). The hive sits at the field centre, so the goal moves with it. */
    public static double W_IN = W_MODEL_IN;
    public static final double CM_PER_IN = 2.54;
    public static final double M_PER_IN = 0.0254;

    /**
     * Handedness of the Java odometry frame, meaning of {@code VisionSubsystem.LATERAL_SIGN}:
     * -1 = robot-left is (sin h, -cos h) = same handedness as TEAM (what the code says);
     * +1 = conventional right-handed y-left frame, mapped with y -> W - y, h -> -h (Python
     * {@code odometry_to_team(lateral_sign=+1)}; same origin corner and x axis ASSUMED).
     * UNVERIFIED on the robot: drive the robot to its left and check that Y grows (team +y = right means a
     * robot facing +X that strafes LEFT must see Y DECREASE).
     */
    public static int LATERAL_SIGN = -1;

    /**
     * true: the Java pose is in one fixed field frame = the RED team frame for both alliances (what the code says),
     * so a BLUE robot is rotated 180 deg into its team frame here. false: the Java pose is already alliance-relative
     * (start poses entered in the alliance's own frame), no rotation.
     */
    public static boolean JAVA_FRAME_IS_FIXED_RED = true;

    /** Scratch output of {@link #toTeam} (overwritten, no allocation). */
    public double x, y, headingDeg, vx, vy, omegaDps, ax, ay;

    private static double wrap180(double a) {
        double r = (a + 180.0) % 360.0;
        if (r < 0) r += 360.0;
        return r - 180.0;
    }

    /** True when the Java frame must be rotated 180 deg to reach the alliance's team frame. */
    public static boolean rotate(boolean blue) {
        return blue && JAVA_FRAME_IS_FIXED_RED;
    }

    /** Sign that maps a Java-frame relative angle (turret, heading rate) to TEAM: +1 or -1. */
    public static double handSign() {
        return LATERAL_SIGN < 0 ? 1.0 : -1.0;
    }

    /**
     * Java pose / motion -> TEAM frame for the given alliance. Inputs in the units the shooter code has at hand:
     * position cm, heading deg (Odometry.Heading()), velocity and acceleration m/s and m/s^2 (PlannerInputFilter),
     * heading rate deg/s (same sense as Heading()). Result in {@link #x} ... {@link #ay} (in, deg, in/s, in/s^2).
     */
    public ShotFrame toTeam(boolean blue, double xCm, double yCm, double headingDeg,
                            double vxMps, double vyMps, double omegaDps, double axMps2, double ayMps2) {
        double px = xCm / CM_PER_IN, py = yCm / CM_PER_IN;
        double pvx = vxMps / M_PER_IN, pvy = vyMps / M_PER_IN;
        double pax = axMps2 / M_PER_IN, pay = ayMps2 / M_PER_IN;
        double h = headingDeg, om = omegaDps;
        if (LATERAL_SIGN >= 0) {          // reflection y -> W - y, h -> -h
            py = W_IN - py;
            pvy = -pvy;
            pay = -pay;
            h = -h;
            om = -om;
        }
        if (rotate(blue)) {               // fixed red frame -> blue team frame: 180 deg about the centre
            px = W_IN - px;
            py = W_IN - py;
            pvx = -pvx;
            pvy = -pvy;
            pax = -pax;
            pay = -pay;
            h = h + 180.0;
        }
        x = px;
        y = py;
        this.headingDeg = wrap180(h);
        vx = pvx;
        vy = pvy;
        ax = pax;
        ay = pay;
        this.omegaDps = om;
        return this;
    }

    /**
     * TEAM turret angle (relative to the robot heading, + toward the side the TEAM heading grows to) -> the
     * Java planner's turret convention (relative to Heading(), + toward the side Heading() grows to; this is what
     * {@code ShooterController.getGoalBearingDeg()} and the old Shotplanner hand to {@code SetTurretAngle}, before
     * {@code turretZeroOffsetDeg}). A rotation does not change relative angles; a reflection negates them.
     */
    public static double turretTeamToJava(double turretTeamDeg) {
        return handSign() * turretTeamDeg;
    }

    /** Java turret angle -> TEAM turret angle (inverse of {@link #turretTeamToJava}). */
    public static double turretJavaToTeam(double turretJavaDeg) {
        return handSign() * turretJavaDeg;
    }

    /**
     * Turret-axis offset from the odometry point in the robot frame, from {@code ShooterTuning}
     * ({@code turretAxisForwardMeters}; {@code turretAxisSidewaysMeters} "positive toward the side heading increases
     * toward") -> TEAM robot-frame (forward, right) [in]. Writes {@code out[0] = forward, out[1] = right}.
     */
    public static void pivotFromJava(double forwardM, double sidewaysM, double[] out) {
        out[0] = forwardM / M_PER_IN;
        out[1] = handSign() * sidewaysM / M_PER_IN;
    }

    /** TEAM point (in) -> Java field point (m) for the alliance (inverse of the position part of toTeam). */
    public static void teamPointToJavaM(boolean blue, double xIn, double yIn, double[] outM) {
        double px = xIn, py = yIn;
        if (rotate(blue)) {
            px = W_IN - px;
            py = W_IN - py;
        }
        if (LATERAL_SIGN >= 0) py = W_IN - py;
        outM[0] = px * M_PER_IN;
        outM[1] = py * M_PER_IN;
    }
}
