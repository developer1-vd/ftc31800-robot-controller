package org.firstinspires.ftc.teamcode.mechanisms;

import android.content.Context;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/**
 * PRACTICE / DEMO ONLY: drive the robot from a phone's touchscreen.
 *
 * Runs a tiny web server on the Robot Controller (Control Hub). A phone connected to the
 * Control Hub's Wi-Fi opens http://192.168.43.1:8090 and gets on-screen joysticks and
 * buttons (assets/phone_gamepad.html). Its input is blended into gamepad1 / gamepad2 every
 * loop, so the real gamepads keep working at the same time.
 *
 * Not for matches: FTC rules allow only the Driver Station to control the robot, and no
 * other device may connect to the Robot Controller's Wi-Fi during a match (R704). That's
 * why this lives in its own OpMode (TeamBotTeleop_Phone), not the competition teleop.
 *
 * Safety: if the phone stops sending for DEADMAN_MS (page closed, Wi-Fi dropped), its input
 * goes back to neutral.
 */
public class PhoneGamepadServer {

    public static final int PORT = 8090;
    public static final String ADDRESS = "http://192.168.43.1:" + PORT;
    private static final long DEADMAN_MS = 300;

    /** Latest input from the phone for one gamepad. */
    private static class PhonePad {
        volatile float lx, ly, rx, ry, lt, rt;
        volatile int buttons;          // bit mask, see BUTTON_* below
        volatile long lastUpdateMs = 0;

        boolean isLive() {
            return System.currentTimeMillis() - lastUpdateMs < DEADMAN_MS;
        }
    }

    // Button bits (must match phone_gamepad.html)
    private static final int A = 1, B = 1 << 1, X = 1 << 2, Y = 1 << 3,
            LB = 1 << 4, RB = 1 << 5, UP = 1 << 6, DOWN = 1 << 7, LEFT = 1 << 8, RIGHT = 1 << 9,
            BACK = 1 << 10, START = 1 << 11;

    private final PhonePad pad1 = new PhonePad();
    private final PhonePad pad2 = new PhonePad();
    private final byte[] page;
    private ServerSocket server;
    private Thread thread;
    private volatile boolean running = false;

    /** Loads the control page from the app's assets. */
    public PhoneGamepadServer(Context context) throws IOException {
        try (InputStream in = context.getAssets().open("phone_gamepad.html")) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buf = new byte[4096];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            page = out.toByteArray();
        }
    }

    public void start() throws IOException {
        server = new ServerSocket();
        server.setReuseAddress(true);   // so restarting the OpMode can reuse the port
        server.bind(new InetSocketAddress(PORT));
        running = true;
        thread = new Thread(this::serve, "PhoneGamepadServer");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        running = false;
        try {
            if (server != null) server.close();
        } catch (IOException ignored) {
        }
    }

    /** True while the phone is connected and sending (for gamepad 1 or 2). */
    public boolean isPhoneConnected() {
        return pad1.isLive() || pad2.isLive();
    }

    /** Blend the phone's input into the real gamepads. Call at the start of every loop. */
    public void applyTo(Gamepad gamepad1, Gamepad gamepad2) {
        apply(pad1, gamepad1);
        apply(pad2, gamepad2);
    }

    /** Sticks/triggers: whichever is pushed further wins. Buttons: either one pressed. */
    private static void apply(PhonePad p, Gamepad g) {
        if (!p.isLive()) return;
        g.left_stick_x  = bigger(g.left_stick_x,  p.lx);
        g.left_stick_y  = bigger(g.left_stick_y,  p.ly);
        g.right_stick_x = bigger(g.right_stick_x, p.rx);
        g.right_stick_y = bigger(g.right_stick_y, p.ry);
        g.left_trigger  = Math.max(g.left_trigger,  p.lt);
        g.right_trigger = Math.max(g.right_trigger, p.rt);
        int b = p.buttons;
        g.a |= (b & A) != 0;
        g.b |= (b & B) != 0;
        g.x |= (b & X) != 0;
        g.y |= (b & Y) != 0;
        g.left_bumper  |= (b & LB) != 0;
        g.right_bumper |= (b & RB) != 0;
        g.dpad_up    |= (b & UP) != 0;
        g.dpad_down  |= (b & DOWN) != 0;
        g.dpad_left  |= (b & LEFT) != 0;
        g.dpad_right |= (b & RIGHT) != 0;
        g.back  |= (b & BACK) != 0;
        g.start |= (b & START) != 0;
    }

    private static float bigger(float a, float b) {
        return Math.abs(b) > Math.abs(a) ? b : a;
    }

    // -------------------------------------------------
    // Minimal HTTP server: GET / -> control page, GET /input?... -> update
    // -------------------------------------------------

    private void serve() {
        while (running) {
            try (Socket socket = server.accept()) {
                socket.setSoTimeout(1000);
                handle(socket);
            } catch (IOException e) {
                // socket closed by stop(), or one bad request: keep serving while running
            }
        }
    }

    private void handle(Socket socket) throws IOException {
        String requestLine = readLine(socket.getInputStream());
        if (requestLine == null) return;
        // Skip the headers
        String header;
        do {
            header = readLine(socket.getInputStream());
        } while (header != null && !header.isEmpty());

        String[] parts = requestLine.split(" ");
        String path = parts.length > 1 ? parts[1] : "/";
        OutputStream out = socket.getOutputStream();
        if (path.startsWith("/input?")) {
            parseInput(path.substring("/input?".length()));
            respond(out, "200 OK", "text/plain", "ok".getBytes(StandardCharsets.UTF_8));
        } else if (path.equals("/") || path.startsWith("/?")) {
            respond(out, "200 OK", "text/html; charset=utf-8", page);
        } else {
            respond(out, "404 Not Found", "text/plain", "not found".getBytes(StandardCharsets.UTF_8));
        }
    }

    /** Query: p=1|2&lx=&ly=&rx=&ry=&lt=&rt=&b=<button bits> */
    private void parseInput(String query) {
        PhonePad pad = pad1;
        float lx = 0, ly = 0, rx = 0, ry = 0, lt = 0, rt = 0;
        int buttons = 0;
        for (String pair : query.split("&")) {
            int eq = pair.indexOf('=');
            if (eq < 0) continue;
            String key = pair.substring(0, eq);
            String value = pair.substring(eq + 1);
            try {
                switch (key) {
                    case "p":  pad = "2".equals(value) ? pad2 : pad1; break;
                    case "lx": lx = clamp(Float.parseFloat(value), -1, 1); break;
                    case "ly": ly = clamp(Float.parseFloat(value), -1, 1); break;
                    case "rx": rx = clamp(Float.parseFloat(value), -1, 1); break;
                    case "ry": ry = clamp(Float.parseFloat(value), -1, 1); break;
                    case "lt": lt = clamp(Float.parseFloat(value), 0, 1); break;
                    case "rt": rt = clamp(Float.parseFloat(value), 0, 1); break;
                    case "b":  buttons = Integer.parseInt(value); break;
                    default: break;
                }
            } catch (NumberFormatException ignored) {
            }
        }
        pad.lx = lx; pad.ly = ly; pad.rx = rx; pad.ry = ry; pad.lt = lt; pad.rt = rt;
        pad.buttons = buttons;
        pad.lastUpdateMs = System.currentTimeMillis();
    }

    private static float clamp(float v, float lo, float hi) {
        return Float.isNaN(v) ? 0 : Math.max(lo, Math.min(hi, v));
    }

    private static void respond(OutputStream out, String status, String type, byte[] body) throws IOException {
        String head = "HTTP/1.1 " + status + "\r\n"
                + "Content-Type: " + type + "\r\n"
                + "Content-Length: " + body.length + "\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n";
        out.write(head.getBytes(StandardCharsets.US_ASCII));
        out.write(body);
        out.flush();
    }

    private static String readLine(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder();
        int c;
        while ((c = in.read()) != -1) {
            if (c == '\n') break;
            if (c != '\r') sb.append((char) c);
            if (sb.length() > 8192) break;   // ignore absurdly long lines
        }
        return (c == -1 && sb.length() == 0) ? null : sb.toString();
    }
}
