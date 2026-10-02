package frc.spectrumLib.telemetry;

import java.io.EOFException;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.regex.Pattern;

/**
 * The storage half of {@link DSLogReceiver}: which file names are accepted, the write-then-rename
 * that keeps a half-received file from ever looking complete, and the folder cap. Plain Java, no
 * WPILib, so it runs (and is tested) off the robot.
 *
 * <p>Only files whose names pass {@link #isValidName} (and their {@code .part} temporaries) are
 * ever listed, counted or deleted; anything else in the folder is left alone.
 */
public final class DSLogFiles {
    private DSLogFiles() {}

    /**
     * The Driver Station's own naming, e.g. {@code 2026_10_02 14_03_27 Fri.dslog}. Nothing else is
     * accepted, which also rules out any path separator or {@code ..} in the name.
     */
    private static final Pattern NAME =
            Pattern.compile(
                    "^\\d{4}_\\d{2}_\\d{2} \\d{2}_\\d{2}_\\d{2} [A-Za-z]{3}\\.(dslog|dsevents)$");

    /** Suffix of the file an upload streams into before it is renamed over the real name. */
    public static final String PART_SUFFIX = ".part";

    private static final int BUFFER_BYTES = 64 * 1024;

    /** One stored file, for the listing. */
    public record Entry(String name, long size, long mtimeMillis) {}

    /** Thrown by {@link #receive} when the abort check fires (the robot enabled mid-upload). */
    public static final class AbortedException extends IOException {
        private static final long serialVersionUID = 1L;

        AbortedException() {
            super("robot enabled during upload");
        }
    }

    /**
     * Whether {@code name} is a Driver Station log or events file name.
     *
     * @param name the decoded file name
     * @return true if it may be stored
     */
    public static boolean isValidName(String name) {
        return name != null && NAME.matcher(name).matches();
    }

    /**
     * The log folder: {@code usb} if it is a writable directory, else {@code local}.
     *
     * @param usb the USB stick's log folder
     * @param local the internal log folder
     * @return the folder to use
     */
    public static File chooseLogDir(String usb, String local) {
        File stick = new File(usb);
        return stick.isDirectory() && stick.canWrite() ? stick : new File(local);
    }

    /**
     * Streams exactly {@code length} bytes into {@code dir/name.part}, forces them to disk, then
     * renames the part file over {@code dir/name} (replacing it: a later push of the same name is a
     * larger snapshot). On any failure the part file is deleted and the old file, if any, is left
     * as it was.
     *
     * @param in the request body
     * @param length bytes to read (the request's Content-Length)
     * @param dir the folder, created if needed
     * @param name a name that passed {@link #isValidName}
     * @param abort checked before every chunk; true stops the upload with {@link AbortedException}
     * @return bytes stored
     * @throws IOException if the body ends early, the disk fails, or the upload is aborted
     */
    public static long receive(
            InputStream in, long length, File dir, String name, BooleanSupplier abort)
            throws IOException {
        if (!isValidName(name)) {
            throw new IllegalArgumentException("bad DS log name: " + name);
        }
        Path folder = dir.toPath();
        Files.createDirectories(folder);
        Path target = folder.resolve(name);
        Path part = folder.resolve(name + PART_SUFFIX);
        long written = 0;
        boolean complete = false;
        try {
            try (FileChannel out =
                    FileChannel.open(
                            part,
                            StandardOpenOption.CREATE,
                            StandardOpenOption.TRUNCATE_EXISTING,
                            StandardOpenOption.WRITE)) {
                byte[] buf = new byte[BUFFER_BYTES];
                while (written < length) {
                    if (abort.getAsBoolean()) {
                        throw new AbortedException();
                    }
                    int n = in.read(buf, 0, (int) Math.min(buf.length, length - written));
                    if (n < 0) {
                        throw new EOFException(
                                "body ended after " + written + " of " + length + " bytes");
                    }
                    ByteBuffer chunk = ByteBuffer.wrap(buf, 0, n);
                    while (chunk.hasRemaining()) {
                        out.write(chunk);
                    }
                    written += n;
                }
                // The robot is often switched off minutes after the match: get it onto the disk.
                out.force(true);
            }
            try {
                Files.move(
                        part,
                        target,
                        StandardCopyOption.ATOMIC_MOVE,
                        StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException e) {
                Files.move(part, target, StandardCopyOption.REPLACE_EXISTING);
            }
            complete = true;
            return written;
        } finally {
            if (!complete) {
                try {
                    Files.deleteIfExists(part);
                } catch (IOException | RuntimeException ignored) {
                    // the next start's cleanParts() gets it
                }
            }
        }
    }

    /**
     * Deletes the oldest stored files (and stray part files) until the rest total at most {@code
     * capBytes}. {@code keepName} is never deleted (it is about to be replaced anyway).
     *
     * @param dir the folder
     * @param capBytes most the files may total
     * @param keepName a name to skip, or null
     * @return how many files were deleted
     */
    public static int trim(File dir, long capBytes, String keepName) throws IOException {
        List<Path> files = ownFiles(dir.toPath(), true);
        List<long[]> meta = new ArrayList<>(); // {index, size, mtime}
        long total = 0;
        for (int i = 0; i < files.size(); i++) {
            BasicFileAttributes a = Files.readAttributes(files.get(i), BasicFileAttributes.class);
            meta.add(new long[] {i, a.size(), a.lastModifiedTime().toMillis()});
            total += a.size();
        }
        meta.sort(Comparator.comparingLong((long[] m) -> m[2])); // oldest first
        int deleted = 0;
        for (long[] m : meta) {
            if (total <= capBytes) {
                break;
            }
            Path p = files.get((int) m[0]);
            if (p.getFileName().toString().equals(keepName)) {
                continue;
            }
            if (Files.deleteIfExists(p)) {
                total -= m[1];
                deleted++;
            }
        }
        return deleted;
    }

    /**
     * Deletes part files left by an upload that was cut off (power, crash). Call before the server
     * starts.
     *
     * @param dir the folder
     * @return how many were deleted
     */
    public static int cleanParts(File dir) throws IOException {
        int deleted = 0;
        for (Path p : ownFiles(dir.toPath(), true)) {
            if (p.getFileName().toString().endsWith(PART_SUFFIX) && Files.deleteIfExists(p)) {
                deleted++;
            }
        }
        return deleted;
    }

    /**
     * The stored files, by name.
     *
     * @param dir the folder
     * @return one entry per complete file (no part files)
     */
    public static List<Entry> list(File dir) throws IOException {
        List<Entry> out = new ArrayList<>();
        for (Path p : ownFiles(dir.toPath(), false)) {
            BasicFileAttributes a = Files.readAttributes(p, BasicFileAttributes.class);
            out.add(
                    new Entry(
                            p.getFileName().toString(), a.size(), a.lastModifiedTime().toMillis()));
        }
        out.sort(Comparator.comparing(Entry::name));
        return out;
    }

    /** The listing as a JSON array of {@code {"name","size","mtime"}} (mtime in epoch ms). */
    public static String toJson(List<Entry> entries) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < entries.size(); i++) {
            Entry e = entries.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"name\":")
                    .append(jsonString(e.name()))
                    .append(",\"size\":")
                    .append(e.size())
                    .append(",\"mtime\":")
                    .append(e.mtimeMillis())
                    .append('}');
        }
        return sb.append(']').toString();
    }

    /** A JSON string literal. */
    public static String jsonString(String s) {
        StringBuilder sb = new StringBuilder("\"");
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"' -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                default -> {
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
                }
            }
        }
        return sb.append('"').toString();
    }

    /** Regular files in {@code dir} with a valid name (and, if asked, their part files). */
    private static List<Path> ownFiles(Path dir, boolean includeParts) throws IOException {
        List<Path> out = new ArrayList<>();
        if (!Files.isDirectory(dir)) {
            return out;
        }
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(dir)) {
            for (Path p : stream) {
                String n = p.getFileName().toString();
                boolean own =
                        isValidName(n)
                                || (includeParts
                                        && n.endsWith(PART_SUFFIX)
                                        && isValidName(
                                                n.substring(0, n.length() - PART_SUFFIX.length())));
                if (own && Files.isRegularFile(p)) {
                    out.add(p);
                }
            }
        }
        return out;
    }
}
