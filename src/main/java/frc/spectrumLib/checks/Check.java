package frc.spectrumLib.checks;

/**
 * One pre-match self-check, registered with {@link PreMatchChecks}.
 *
 * <p>Both methods run on the main thread after this loop's inputs are read, and must read only
 * logged inputs (or values derived from them), so a replayed match gives the same verdicts. Neither
 * may block, command a device or stop the robot from enabling: a check only reports.
 */
public interface Check {

    /**
     * Called every loop, enabled or not, for checks that must see every loop: a reset flag that is
     * true for one loop, or a condition that has to hold for some time. Keep it cheap.
     *
     * @param nowSeconds this loop's robot time
     */
    default void observe(double nowSeconds) {}

    /**
     * Judges the robot now. Called only while disabled, and not every loop: {@link PreMatchChecks}
     * spreads the checks across loops.
     *
     * @param nowSeconds this loop's robot time
     * @return the verdict
     */
    CheckResult evaluate(double nowSeconds);
}
