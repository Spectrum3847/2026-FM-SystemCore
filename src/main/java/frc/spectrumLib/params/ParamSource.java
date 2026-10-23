package frc.spectrumLib.params;

/** Where a param's current value came from. Later layers win. */
public enum ParamSource {
    /** The default in the declaration. */
    DEFAULT,
    /** {@code params/<group>.json} in the deploy folder. */
    DEPLOY,
    /** {@code params/robots/<robot>/<group>.json} in the deploy folder. */
    ROBOT,
    /** {@code <group>.json} in the robot-local folder, which a deploy does not wipe. */
    LOCAL,
    /** Changed over the transport (NetworkTables) while disabled, with live tuning on. */
    LIVE
}
