package frc.spectrumLib.localization;

/**
 * Ring buffer of chassis yaw-rate samples, for the lookback spin gate.
 *
 * <p>From the 2026 offseason {@code Vision.peakYawRateRadPerSec()}: frames that reach the robot now
 * were captured up to a few tenths of a second ago, so the gate has to ask whether the robot was
 * spinning <em>then</em>, not whether it is spinning now. A frame arriving just after a spin stops
 * was captured during it.
 */
public class YawRateHistory {
    private final double[] rates;
    private final double[] times;
    private int index = 0;

    /**
     * @param capacity samples kept; at 50 Hz, 64 covers well over a second
     */
    public YawRateHistory(int capacity) {
        rates = new double[capacity];
        times = new double[capacity];
        java.util.Arrays.fill(times, Double.NEGATIVE_INFINITY);
    }

    /** Records |yaw rate| at a time. */
    public void record(double timeSeconds, double yawRateRadPerSec) {
        rates[index] = Math.abs(yawRateRadPerSec);
        times[index] = timeSeconds;
        index = (index + 1) % rates.length;
    }

    /** Largest |yaw rate| recorded within {@code lookbackSeconds} of {@code nowSeconds}. */
    public double peak(double nowSeconds, double lookbackSeconds) {
        double cutoff = nowSeconds - lookbackSeconds;
        double peak = 0;
        for (int i = 0; i < rates.length; i++) {
            if (times[i] >= cutoff && rates[i] > peak) {
                peak = rates[i];
            }
        }
        return peak;
    }
}
