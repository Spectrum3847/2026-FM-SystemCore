package frc.spectrumLib.checks;

/**
 * One check's verdict and the reason for it, in words a drive team can act on.
 *
 * @param status the verdict
 * @param reason one line: what was found, with the numbers
 */
public record CheckResult(Status status, String reason) {

    /** Verdicts, least to most severe. Only {@link #FAIL} makes the robot NO-GO. */
    public enum Status {
        /** Nothing to check on this robot (no such device, no data source). */
        SKIP,
        PASS,
        /** Worth a look, but not a reason to hold the match. */
        WARN,
        /** NO-GO. */
        FAIL;

        /** The more severe of two verdicts. */
        public Status worst(Status other) {
            return other.ordinal() > ordinal() ? other : this;
        }
    }

    public static CheckResult skip(String reason) {
        return new CheckResult(Status.SKIP, reason);
    }

    public static CheckResult pass(String reason) {
        return new CheckResult(Status.PASS, reason);
    }

    public static CheckResult warn(String reason) {
        return new CheckResult(Status.WARN, reason);
    }

    public static CheckResult fail(String reason) {
        return new CheckResult(Status.FAIL, reason);
    }

    public boolean isFail() {
        return status == Status.FAIL;
    }

    public boolean isWarn() {
        return status == Status.WARN;
    }
}
