package frc.spectrumLib.params;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * Publishes params at {@code <prefix>/<group>/<key>} on NetworkTables through AdvantageKit's
 * LoggedNetwork inputs, so the dashboard's requests are in the log too.
 */
public final class NetworkTablesTransport implements ParamTransport {
    private final String prefix;

    /**
     * @param prefix NT path the params go under, e.g. {@code /Tuning}
     */
    public NetworkTablesTransport(String prefix) {
        this.prefix = prefix;
    }

    @Override
    public LiveValue publish(Param<?> param) {
        String key = prefix + "/" + param.path();
        switch (param.type()) {
            case NUMBER:
            case INTEGER:
                {
                    var n = new LoggedNetworkNumber(key, ((Number) param.get()).doubleValue());
                    return LiveValue.of(n::get, v -> n.set(((Number) v).doubleValue()));
                }
            case BOOLEAN:
                {
                    var b = new LoggedNetworkBoolean(key, (Boolean) param.get());
                    return LiveValue.of(b::get, v -> b.set((Boolean) v));
                }
            case STRING:
            default:
                {
                    var s = new LoggedNetworkString(key, param.get().toString());
                    return LiveValue.of(s::get, v -> s.set(v.toString()));
                }
        }
    }

    @Override
    public LiveValue button(String name) {
        var b = new LoggedNetworkBoolean(prefix + "/" + name, false);
        return LiveValue.of(b::get, v -> b.set((Boolean) v));
    }
}
