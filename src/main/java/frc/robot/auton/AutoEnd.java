package frc.robot.auton;

import frc.spectrumLib.telemetry.Telemetry;
import java.util.Optional;
import java.util.function.Consumer;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;

/**
 * Logs why the auto command ended and how long it ran. Nine TXMAN1 autos stopped between 2.5 and
 * 17.9 s, and nothing in those logs said whether the auto finished, was interrupted, or the robot
 * was disabled.
 *
 * <p>The scheduler's finish and interrupt callbacks run on the main thread inside the scheduler,
 * and every input the reason reads (the Driver Station's enable, E-stop, FMS, DS attached and match
 * time) is an AdvantageKit input, so a replayed log gives the same reason.
 */
public final class AutoEnd {

    /** Why the auto command ended. */
    public enum Reason {
        /** The auto ran to its end. */
        FINISHED,
        /** It ran past its time limit and was cancelled. */
        TIMED_OUT,
        /** Another command took one of its subsystems; {@code Auton/EndInterruptedBy} names it. */
        INTERRUPTED,
        /** Cancelled by code while enabled, with nothing taking its place. */
        CANCELLED,
        /** Disabled by an E-stop. */
        E_STOP,
        /** Disabled with no time left in the period: the normal end of a match's auto. */
        END_OF_PERIOD,
        /**
         * Disabled by the field with time left in the period. The 2027 Driver Station API has no
         * A-Stop flag, so an A-Stop and a field fault look the same here.
         */
        DISABLED_BY_FMS_EARLY,
        /** Disabled from a Driver Station with no field attached. */
        DISABLED_BY_DS,
        /** Disabled because the Driver Station stopped talking to the robot (a comms drop). */
        DS_DISCONNECTED,
    }

    /**
     * How an auto ended, as logged.
     *
     * @param reason why
     * @param interruptedBy the command that interrupted it, or empty
     * @param seconds how long it ran
     */
    public record Ended(Reason reason, String interruptedBy, double seconds) {}

    /**
     * Match time below which a disable is the period running out. The field counts match time in
     * whole seconds; every TXMAN1 auto that ran its 20 s was disabled reading 0, the early ones at
     * 3 to 18.
     */
    static final double END_OF_PERIOD_MATCH_TIME = 1.0;

    private Command auto;
    private double startSeconds;
    private double timeLimitSeconds = Double.POSITIVE_INFINITY;
    private boolean timedOut;
    private Consumer<Ended> listener;

    @SuppressWarnings("ReferenceEquality") // the scheduled instance, not an equal command
    public AutoEnd() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandExecute(
                command -> {
                    if (command == auto && !timedOut && elapsed() >= timeLimitSeconds) {
                        timedOut = true;
                        command.cancel();
                    }
                });
        scheduler.onCommandFinish(
                command -> {
                    if (command == auto) {
                        ended(false, Optional.empty());
                    }
                });
        scheduler.onCommandInterrupt(
                (command, interruptor) -> {
                    if (command == auto) {
                        ended(true, interruptor);
                    }
                });
    }

    /**
     * Sets the one listener told when an auto ends, on the main thread, after it is logged.
     *
     * @param listener told the reason, the interrupting command and the elapsed time
     */
    public void onEnded(Consumer<Ended> listener) {
        this.listener = listener;
    }

    /**
     * Schedules an auto and watches how it ends.
     *
     * @param command the auto
     * @param timeLimitSeconds cancel it after this long, or {@code Double.POSITIVE_INFINITY}
     */
    public void start(Command command, double timeLimitSeconds) {
        auto = command;
        this.timeLimitSeconds = timeLimitSeconds;
        timedOut = false;
        startSeconds = Timer.getTimestamp();
        CommandScheduler.getInstance().schedule(command);
    }

    private double elapsed() {
        return Timer.getTimestamp() - startSeconds;
    }

    private void ended(boolean interrupted, Optional<Command> interruptor) {
        double seconds = elapsed();
        Reason reason =
                classify(
                        interrupted,
                        interruptor.isPresent(),
                        timedOut,
                        RobotState.isEnabled(),
                        RobotState.isEStopped(),
                        RobotState.isFMSAttached(),
                        RobotState.isDSAttached(),
                        MatchState.getMatchTime());
        String by = interruptor.map(Command::getName).orElse("");
        // Why the auto ended: FINISHED, END_OF_PERIOD, DISABLED_BY_FMS_EARLY, DS_DISCONNECTED,
        // E_STOP, ...
        Telemetry.log("Auton/EndReason", reason.name());
        // The command that interrupted the auto, if one did.
        Telemetry.log("Auton/EndInterruptedBy", by);
        // How long the auto ran.
        Telemetry.log("Auton/EndElapsedSeconds", seconds, "seconds");
        Telemetry.logEvent(
                "Events",
                String.format(
                        "auto %s ended: %s%s after %.2f s",
                        auto.getName(), reason, by.isEmpty() ? "" : " by " + by, seconds));
        Telemetry.print(
                String.format(
                        "*** Auton %s ended: %s%s after %.2f s ***",
                        auto.getName(), reason, by.isEmpty() ? "" : " by " + by, seconds));
        auto = null;
        if (listener != null) {
            listener.accept(new Ended(reason, by, seconds));
        }
    }

    /**
     * Why an auto ended, from the scheduler's callback and this loop's Driver Station state.
     *
     * @param interrupted it ended through the interrupt callback, not by finishing
     * @param hasInterruptor another command interrupted it
     * @param timedOut it was cancelled for running past its time limit
     * @param enabled the robot is enabled this loop
     * @param eStopped the robot is E-stopped
     * @param fmsAttached a field is attached
     * @param dsAttached a Driver Station is talking to the robot
     * @param matchTime seconds left in the period, negative when the DS has no match timer
     * @return the reason
     */
    static Reason classify(
            boolean interrupted,
            boolean hasInterruptor,
            boolean timedOut,
            boolean enabled,
            boolean eStopped,
            boolean fmsAttached,
            boolean dsAttached,
            double matchTime) {
        if (!interrupted) {
            return Reason.FINISHED;
        }
        if (hasInterruptor) {
            return Reason.INTERRUPTED;
        }
        if (eStopped) {
            return Reason.E_STOP;
        }
        if (enabled) {
            return timedOut ? Reason.TIMED_OUT : Reason.CANCELLED;
        }
        // Before the match time and FMS flag: with no Driver Station they say nothing about why.
        if (!dsAttached) {
            return Reason.DS_DISCONNECTED;
        }
        if (matchTime >= 0 && matchTime < END_OF_PERIOD_MATCH_TIME) {
            return Reason.END_OF_PERIOD;
        }
        return fmsAttached ? Reason.DISABLED_BY_FMS_EARLY : Reason.DISABLED_BY_DS;
    }
}
