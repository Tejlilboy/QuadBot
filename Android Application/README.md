# QuadBot Android app

A thin full-screen Android app around the QuadBot control page. It does what opening `http://192.168.4.1` in a browser does, plus a few things a browser can't:

- Full screen, locked to landscape, and the screen stays on while you drive.
- Forces the app's traffic over the robot's WiFi. Android otherwise tends to send traffic over mobile data when a WiFi network has no internet.
- A **"tap to join QuadBot WiFi"** button that connects to the robot from inside the app. It disappears once the page is connected.
- The control page is bundled inside the app, so it opens instantly.

The robot firmware does not change. The app talks to the same WebSocket (`ws://192.168.4.1:81`) as the browser page.

## Requirements

- Android 10 or newer
- Android Studio (free) to build and install
- The robot running `web_version_quadbot.ino`

## Build and install

1. Open Android Studio, choose **File → Open**, and select this `QuadBotApp` folder.
2. Wait for the Gradle sync to finish (the first sync downloads a lot). Accept any prompt to install the Android SDK Platform or to update the Android Gradle plugin.
3. On your phone, enable **Developer options** (Settings → About phone → tap *Build number* 7 times), then turn on **USB debugging**.
4. Plug the phone in, allow the debugging prompt, pick the phone at the top of Android Studio, and press **Run** (the green triangle).

To get an APK instead: **Build → Build Bundle(s) / APK(s) → Build APK(s)**, then send the file to your phone and open it (you will need to allow installing from that source).

## Using it

1. Power the robot.
2. Open the **QuadBot** app.
3. Tap **"Not connected - tap to join QuadBot WiFi"** and approve the system prompt. If you have already joined the QuadBot network in WiFi settings, the page connects on its own.

The connection made by the button belongs to the app and ends when the app closes. Your phone's normal WiFi and mobile data are left alone.

## Settings

All in `app/src/main/java/com/quadbot/controller/MainActivity.java`:

| Setting | Meaning |
|---|---|
| `ROBOT_SSID`, `ROBOT_PASS` | Must match `AP_SSID` and `AP_PASS` in the firmware |
| `PAGE_URL` | The bundled page by default. Set to `"http://192.168.4.1/"` to always load the page stored on the robot |

Other things you may want to change:

- **Allow portrait:** delete `android:screenOrientation="sensorLandscape"` in `AndroidManifest.xml`.
- **Update the page:** if you change `PAGE` in the firmware, copy the HTML into `app/src/main/assets/index.html`.

## Troubleshooting

- **The join button does nothing, or says it could not join:** make sure the robot is on, then join **QuadBot** in the phone's WiFi settings yourself and reopen the app.
- **Gradle sync fails or asks for updates:** accept the offered updates (Android Gradle plugin, SDK Platform). The project uses no third-party libraries.
- **The page stays on "Connecting...":** only one phone can drive at a time. Close the browser tab or any other phone that has the page open.
- **The robot stops when you switch apps:** this is intended. When the app is in the background the page stops sending, and the robot's deadman timer makes it stand still.
