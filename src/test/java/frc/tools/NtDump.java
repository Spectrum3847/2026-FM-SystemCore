package frc.tools;

import java.util.Arrays;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.Topic;

/**
 * Connects to a robot's NetworkTables server and prints every topic under the given prefixes.
 *
 * <p>{@code ./gradlew ntDump -Phost=172.26.0.1 -Pprefix=/AdvantageKit/RealOutputs/Localization}
 */
public final class NtDump {
    private NtDump() {}

    public static void main(String[] args) throws Exception {
        String host = args.length > 0 ? args[0] : "172.26.0.1";
        String[] prefixes =
                args.length > 1 ? Arrays.copyOfRange(args, 1, args.length) : new String[] {"/"};
        NetworkTableInstance nt = NetworkTableInstance.create();
        nt.startClient("fm-ntdump");
        nt.setServer(host);
        var subs = new java.util.ArrayList<Object>();
        for (String p : prefixes) {
            subs.add(
                    new org.wpilib.networktables.MultiSubscriber(
                            nt, new String[] {p}, org.wpilib.networktables.PubSubOption.SEND_ALL));
        }
        Thread.sleep(4000);
        System.out.println("connected=" + nt.isConnected());
        for (Topic t : nt.getTopics()) {
            String name = t.getName();
            boolean match = Arrays.stream(prefixes).anyMatch(name::startsWith);
            if (!match) {
                continue;
            }
            NetworkTableValue v = t.getGenericEntry().get();
            Object o = v.getValue();
            String s =
                    o instanceof double[] d
                            ? Arrays.toString(d)
                            : o instanceof boolean[] b
                                    ? Arrays.toString(b)
                                    : o instanceof String[] ss
                                            ? Arrays.toString(ss)
                                            : o instanceof long[] l
                                                    ? Arrays.toString(l)
                                                    : o instanceof byte[] bb
                                                            ? "<"
                                                                    + bb.length
                                                                    + " bytes "
                                                                    + t.getTypeString()
                                                                    + ">"
                                                            : String.valueOf(o);
            System.out.println(name + " = " + (s.length() > 160 ? s.substring(0, 160) + "..." : s));
        }
        nt.close();
        System.exit(0);
    }
}
