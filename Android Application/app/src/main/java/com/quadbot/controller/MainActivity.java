package com.quadbot.controller;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.wifi.WifiNetworkSpecifier;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.View;
import android.view.Window;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.Toast;

/**
 * QuadBot controller app: a thin full-screen shell around the robot's control page.
 *
 * What the app adds on top of opening the page in a browser:
 *   - full screen, landscape, and the screen stays on while you drive
 *   - all of the app's traffic is forced over the robot's WiFi (Android otherwise tends to
 *     send traffic over mobile data when a WiFi network has no internet)
 *   - a "join QuadBot WiFi" button that connects to the robot from inside the app
 *
 * The page itself (assets/index.html) is a copy of the page the robot serves, so it opens
 * instantly. It talks to the robot over a WebSocket at ws://192.168.4.1:81.
 */
public class MainActivity extends Activity {

    // ---- settings (must match AP_SSID / AP_PASS in the robot's firmware) ----
    private static final String ROBOT_SSID = "QuadBot";
    private static final String ROBOT_PASS = "quadbot123";

    // The page to show. The bundled copy opens instantly. To always use the page stored on
    // the robot instead, change this to "http://192.168.4.1/".
    private static final String PAGE_URL = "file:///android_asset/index.html";

    private static final String BG_COLOR = "#0a0e13";

    private final Handler ui = new Handler(Looper.getMainLooper());
    private ConnectivityManager cm;
    private WebView web;
    private Button joinButton;

    private ConnectivityManager.NetworkCallback wifiWatcher;   // keeps the app bound to whatever WiFi is up
    private ConnectivityManager.NetworkCallback joinCallback;  // the "join QuadBot" request
    private Network boundNetwork;

    // ------------------------------------------------------------------ lifecycle

    @SuppressLint("SetJavaScriptEnabled")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);   // never sleep while driving
        cm = (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);

        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.parseColor(BG_COLOR));

        // ---- the control page ----
        web = new WebView(this);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setAllowFileAccess(true);
        s.setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);   // the page is file://, the robot is ws://
        s.setSupportZoom(false);
        s.setBuiltInZoomControls(false);
        web.setBackgroundColor(Color.parseColor(BG_COLOR));
        web.setOverScrollMode(View.OVER_SCROLL_NEVER);
        web.setVerticalScrollBarEnabled(false);
        web.setHorizontalScrollBarEnabled(false);
        web.setLongClickable(false);
        web.setHapticFeedbackEnabled(false);
        web.setOnLongClickListener(v -> true);          // no text-selection / context menu
        web.setWebViewClient(new WebViewClient());      // keep every page inside the app
        root.addView(web, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT));

        // ---- "join WiFi" button, only visible while the page is not connected ----
        float d = getResources().getDisplayMetrics().density;
        joinButton = new Button(this);
        joinButton.setText("Not connected - tap to join QuadBot WiFi");
        joinButton.setAllCaps(false);
        joinButton.setOnClickListener(v -> joinRobotWifi());
        FrameLayout.LayoutParams bp = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.WRAP_CONTENT,
                Gravity.TOP | Gravity.CENTER_HORIZONTAL);
        bp.topMargin = (int) (28 * d);
        root.addView(joinButton, bp);

        setContentView(root);
        web.loadUrl(PAGE_URL);
        startWifiWatcher();
    }

    @Override
    protected void onResume() {
        super.onResume();
        web.onResume();
        ui.post(statusPoll);
    }

    @Override
    protected void onPause() {
        // Pausing the page stops its timers, so the robot stops hearing from us and
        // its deadman timer makes it stand still. That is the safe behavior.
        ui.removeCallbacks(statusPoll);
        web.onPause();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        ui.removeCallbacksAndMessages(null);
        stopCallback(joinCallback);
        stopCallback(wifiWatcher);
        cm.bindProcessToNetwork(null);
        web.destroy();
        super.onDestroy();
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) goImmersive();
    }

    // ------------------------------------------------------------------ status button

    // Every 0.8 s, ask the page whether it is connected (the page marks <body class="off">
    // while it is not) and show or hide the join button to match.
    private final Runnable statusPoll = new Runnable() {
        @Override
        public void run() {
            web.evaluateJavascript(
                    "document.body ? document.body.classList.contains('off') : true",
                    value -> joinButton.setVisibility("false".equals(value) ? View.GONE : View.VISIBLE));
            ui.postDelayed(this, 800);
        }
    };

    // ------------------------------------------------------------------ WiFi

    // Keeps the whole app bound to the WiFi network while one is up, so traffic to the robot
    // is not rerouted over mobile data. This covers the case where you joined QuadBot in the
    // phone's WiFi settings yourself.
    private void startWifiWatcher() {
        NetworkRequest request = new NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .build();
        wifiWatcher = new ConnectivityManager.NetworkCallback() {
            @Override
            public void onAvailable(Network network) {
                boundNetwork = network;
                cm.bindProcessToNetwork(network);
            }

            @Override
            public void onLost(Network network) {
                if (network.equals(boundNetwork)) {
                    boundNetwork = null;
                    cm.bindProcessToNetwork(null);
                }
            }
        };
        cm.registerNetworkCallback(request, wifiWatcher);
    }

    // Joins the robot's WiFi from inside the app. Android shows a one-time system prompt
    // asking you to allow it. This connection belongs to the app: it ends when the app
    // closes, and your phone's normal WiFi and mobile data are left alone.
    private void joinRobotWifi() {
        stopCallback(joinCallback);

        WifiNetworkSpecifier specifier = new WifiNetworkSpecifier.Builder()
                .setSsid(ROBOT_SSID)
                .setWpa2Passphrase(ROBOT_PASS)
                .build();
        NetworkRequest request = new NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)   // the robot has no internet
                .setNetworkSpecifier(specifier)
                .build();

        joinCallback = new ConnectivityManager.NetworkCallback() {
            @Override
            public void onAvailable(Network network) {
                boundNetwork = network;
                cm.bindProcessToNetwork(network);
                ui.post(() -> Toast.makeText(MainActivity.this, "Joined " + ROBOT_SSID, Toast.LENGTH_SHORT).show());
            }

            @Override
            public void onUnavailable() {
                ui.post(() -> Toast.makeText(MainActivity.this,
                        "Could not join " + ROBOT_SSID + ". Is the robot on? You can also join it in WiFi settings.",
                        Toast.LENGTH_LONG).show());
            }

            @Override
            public void onLost(Network network) {
                if (network.equals(boundNetwork)) {
                    boundNetwork = null;
                    cm.bindProcessToNetwork(null);
                }
            }
        };
        cm.requestNetwork(request, joinCallback);
        Toast.makeText(this, "Joining " + ROBOT_SSID + "...", Toast.LENGTH_SHORT).show();
    }

    private void stopCallback(ConnectivityManager.NetworkCallback callback) {
        if (callback == null) return;
        try {
            cm.unregisterNetworkCallback(callback);
        } catch (RuntimeException ignored) {
            // already unregistered
        }
    }

    // ------------------------------------------------------------------ full screen

    private void goImmersive() {
        Window w = getWindow();
        if (Build.VERSION.SDK_INT >= 30) {
            w.setDecorFitsSystemWindows(false);
            WindowInsetsController c = w.getInsetsController();
            if (c != null) {
                c.hide(WindowInsets.Type.systemBars());
                c.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
            }
        } else {
            w.getDecorView().setSystemUiVisibility(
                    View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                            | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                            | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                            | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                            | View.SYSTEM_UI_FLAG_FULLSCREEN
                            | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
        }
    }
}
