package frc.robot.subsystems.vision;

import frc.rebuilt.Field;
import frc.spectrumLib.localization.TagEkf;
import org.wpilib.math.geometry.Pose2d;

/**
 * What the field itself says about the robot's position, fed to {@link TagEkf} as one-axis
 * measurements.
 *
 * <ul>
 *   <li><b>Bumps:</b> a steady tilt means the robot is on a bump. If the estimate is near exactly
 *       one bump, the robot's x is that bump's centre x, give or take its depth plus half the
 *       wheelbase. Once per tilt episode.
 *   <li><b>Walls and hubs:</b> the robot's centre can't be within {@code halfSize} of a wall or
 *       inside a hub. When the estimate is, it's pushed back to the boundary along the shallowest
 *       axis. One-sided: nothing happens while the estimate is legal.
 * </ul>
 *
 * Blue geometry from {@link Field}; red mirrored in x.
 */
public final class FieldPriors {

    /** Tilt above this, held for {@link #BUMP_HOLD_SECONDS}, means on a bump. */
    static final double BUMP_TILT_DEG = 4.0;

    static final double BUMP_HOLD_SECONDS = 0.25;

    /** How far outside a bump's outline the estimate may be and still be matched to it. */
    static final double BUMP_MATCH_M = 1.0;

    /** Constraints run at most this often. */
    static final double CONSTRAIN_PERIOD_SECONDS = 0.05;

    /** x min, x max, y min, y max of each bump. */
    static final double[][] BUMPS;

    /** x min, x max, y min, y max of each hub. */
    static final double[][] HUBS;

    static {
        // Field first: touching Field.BlueHub before Field is initialized makes Field's own
        // initializer read BlueHub half-built (NullPointerException in Field.BlueToRed).
        double l = Field.fieldLength;
        double bx0 = Field.BlueHub.centerX - Field.BlueBumps.depth / 2;
        double bx1 = Field.BlueHub.centerX + Field.BlueBumps.depth / 2;
        double left0 = Field.LeftBlueBump.nearRightCorner.getY();
        double left1 = Field.LeftBlueBump.nearLeftCorner.getY();
        double right0 = Field.RightBlueBump.nearRightCorner.getY();
        double right1 = Field.RightBlueBump.nearLeftCorner.getY();
        BUMPS =
                new double[][] {
                    {bx0, bx1, left0, left1},
                    {bx0, bx1, right0, right1},
                    {l - bx1, l - bx0, left0, left1},
                    {l - bx1, l - bx0, right0, right1}
                };
        double hx0 = Field.tag26X;
        double hx1 = Field.tag26X + Field.BlueHub.width;
        double hy0 = Field.BlueHub.centerY - Field.BlueHub.width / 2;
        double hy1 = Field.BlueHub.centerY + Field.BlueHub.width / 2;
        HUBS = new double[][] {{hx0, hx1, hy0, hy1}, {l - hx1, l - hx0, hy0, hy1}};
    }

    private final double bumpStd;
    private final double constrainStd;
    private final double halfSize;

    private double tiltedSince = Double.NaN;
    private boolean usedThisEpisode = false;
    private double lastConstrain = Double.NEGATIVE_INFINITY;
    private long bumpFixes = 0;
    private long constraints = 0;

    /**
     * @param bumpStd std-dev of a bump crossing's x, metres; 0 turns bumps off
     * @param constrainStd std-dev of a wall or hub push, metres; 0 turns constraints off
     * @param halfSize how close the robot's centre can get to a wall or hub, metres
     */
    public FieldPriors(double bumpStd, double constrainStd, double halfSize) {
        this.bumpStd = bumpStd;
        this.constrainStd = constrainStd;
        this.halfSize = halfSize;
    }

    /** One loop: the tilt now, then the estimate. */
    public void update(TagEkf ekf, double t, double tiltDeg) {
        if (!ekf.isSeeded()) {
            return;
        }
        if (bumpStd > 0) {
            bump(ekf, t, tiltDeg);
        }
        if (constrainStd > 0 && t - lastConstrain >= CONSTRAIN_PERIOD_SECONDS) {
            lastConstrain = t;
            constrain(ekf, t);
        }
    }

    private void bump(TagEkf ekf, double t, double tiltDeg) {
        if (tiltDeg < BUMP_TILT_DEG) {
            tiltedSince = Double.NaN;
            usedThisEpisode = false;
            return;
        }
        if (Double.isNaN(tiltedSince)) {
            tiltedSince = t;
        }
        if (usedThisEpisode || t - tiltedSince < BUMP_HOLD_SECONDS) {
            return;
        }
        usedThisEpisode = true;
        Pose2d est = ekf.getPose();
        double[] match = null;
        for (double[] b : BUMPS) {
            if (est.getX() > b[0] - BUMP_MATCH_M
                    && est.getX() < b[1] + BUMP_MATCH_M
                    && est.getY() > b[2] - BUMP_MATCH_M
                    && est.getY() < b[3] + BUMP_MATCH_M) {
                if (match != null) {
                    return; // ponytail: two bumps in reach, so it says nothing
                }
                match = b;
            }
        }
        if (match != null && ekf.addAxis(tiltedSince, 1, 0, (match[0] + match[1]) / 2, bumpStd)) {
            bumpFixes++;
        }
    }

    private void constrain(TagEkf ekf, double t) {
        Pose2d est = ekf.getPose();
        double x = est.getX();
        double y = est.getY();
        // Walls.
        push(ekf, t, 1, 0, x, halfSize, true);
        push(ekf, t, 1, 0, x, Field.fieldLength - halfSize, false);
        push(ekf, t, 0, 1, y, halfSize, true);
        push(ekf, t, 0, 1, y, Field.fieldWidth - halfSize, false);
        // Hubs: out along the shallowest side.
        for (double[] h : HUBS) {
            double x0 = h[0] - halfSize;
            double x1 = h[1] + halfSize;
            double y0 = h[2] - halfSize;
            double y1 = h[3] + halfSize;
            if (x <= x0 || x >= x1 || y <= y0 || y >= y1) {
                continue;
            }
            double[] depth = {x - x0, x1 - x, y - y0, y1 - y};
            int side = 0;
            for (int i = 1; i < 4; i++) {
                if (depth[i] < depth[side]) {
                    side = i;
                }
            }
            switch (side) {
                case 0 -> axis(ekf, t, 1, 0, x0);
                case 1 -> axis(ekf, t, 1, 0, x1);
                case 2 -> axis(ekf, t, 0, 1, y0);
                default -> axis(ekf, t, 0, 1, y1);
            }
        }
    }

    /** Pushes to {@code limit} when {@code value} is below it (atLeast) or above it. */
    private void push(
            TagEkf ekf,
            double t,
            double nx,
            double ny,
            double value,
            double limit,
            boolean atLeast) {
        if (atLeast ? value < limit : value > limit) {
            axis(ekf, t, nx, ny, limit);
        }
    }

    private void axis(TagEkf ekf, double t, double nx, double ny, double value) {
        if (ekf.addAxis(t, nx, ny, value, constrainStd)) {
            constraints++;
        }
    }

    public long bumpFixes() {
        return bumpFixes;
    }

    public long constraints() {
        return constraints;
    }
}
