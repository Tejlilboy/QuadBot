/*
 * ============================================================================
 *  Xbox-controlled quadruped walker with an animated OLED face
 * ============================================================================
 *
 *  An 8-servo, 4-legged robot driven by an ESP32, steered wirelessly with an
 *  Xbox controller (Bluetooth, via Bluepad32). A 128x64 OLED shows an animated
 *  face that reacts to everything the robot does.
 *
 *    "Home" for a lift = foot on the ground. The stand pose ("X stance") puts
 *    every hip at 45 (mid-stroke) with all feet down.
 *
 *  CONTROLS
 *    Left stick up / down      walk forward / backward   (further push = faster)
 *    Left stick left / right   turn on the spot (left = counter-clockwise)
 *    Right stick               move the eyes (idle and sit faces)
 *    Right trigger (RT)        sprint: shorter frames, speed lines on the face
 *    Left trigger  (LT)        sneak: slower, shorter strides, ninja mask face
 *    Left / right bumper       crab walk left / right
 *    A                         sit / stand (toggle)
 *    X                         play dead (toggle)
 *    D-pad down                sleep (toggle)
 *    Y (hold)                  wave
 *    B (hold)                  dance
 *    D-pad up (hold)           push-ups
 *    D-pad left (hold)         shake
 *    D-pad right (hold)        bow
 *    No input for 60 s         yawns, then lies down and sleeps; any input wakes it
 *
 *
 *  HOW THE FILE IS ORGANISED (in order)
 *    1. Servo driver          MyServo class, smooth (rate-limited) servo motion
 *    2. Settings              speeds, frame times, walk drift correction
 *    3. Controller            Bluepad32 callbacks, rumble, button/stick -> action
 *    4. OLED face             drawing helpers, idle animations, the face engine
 *    5. Timing helpers        trigger speed scaling, waiting while staying responsive
 *    6. Poses and movements   pose tables and the functions that play them
 *    7. State                 toggles (sit / sleep / dead) and auto-sleep
 *    8. setup() and loop()
 *
 *  POSE TABLES
 *    Every pose is 8 angles in this column order:
 *        R1  R2  L1  L2  R3  R4  L3  L4
 *    (hips first, then lifts). Movements are just lists of these rows played in
 *    sequence, so a motion can be tuned by editing numbers in its table.
 *
 * ============================================================================
 */

#include <Bluepad32.h>          // Bluetooth gamepad support (ESP32 + Bluepad32 board package)
#include <Wire.h>               // I2C bus for the OLED
#include <Adafruit_GFX.h>        // drawing primitives (lines, circles, text)
#include <Adafruit_SSD1306.h>    // SSD1306 OLED driver

// ============================================================================
//  1. SERVO DRIVER
// ============================================================================
// Drives hobby servos straight from the ESP32 LEDC PWM hardware (50 Hz, 14-bit),
// so no Servo library is needed. Every servo gets its OWN LEDC channel (0-7);
// two servos sharing a channel would copy each other's movement.
//
// write() only sets a TARGET angle. step() (called from servosUpdate()) then moves
// the servo toward that target at a limited speed, which makes all motion smooth.
// Pulse width is 544-2400 us for 0-180 degrees, the same range as the Servo library.
class MyServo {
public:
  // Configures PWM on `pin` using LEDC `channel` (each servo needs a unique channel).
  void attach(int pin, int channel) {
    _pin = pin; _ch = channel;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pin, 50, 14, channel);
#else
    ledcSetup(channel, 50, 14);
    ledcAttachPin(pin, channel);
#endif
  }
  // Sets the target angle (0-180). The very first call jumps straight there so the
  // servo doesn't sweep from an unknown start position.
  void write(int angle) {
    _target = constrain(angle, 0, 180);
    if (!_init) { _init = true; _cur = _target; out((int)_cur); }
  }
  // Moves toward the target by at most maxDelta degrees. Call this often.
  void step(float maxDelta) {
    if (!_init) return;
    float d = _target - _cur;
    if (d > maxDelta) d = maxDelta;
    if (d < -maxDelta) d = -maxDelta;
    _cur += d;
    int a = (int)(_cur + 0.5);
    if (a != _last) out(a);
  }
private:
  // Converts an angle to a pulse width / PWM duty value and sends it to the hardware.
  void out(int angle) {
    _last = angle;
    int us = 544 + (long)(2400 - 544) * angle / 180;
    uint32_t duty = (uint32_t)((uint64_t)us * 16383 / 20000);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_pin, duty);
#else
    ledcWrite(_ch, duty);
#endif
  }
  int _pin = -1, _ch = 0, _last = -1;
  float _cur = 90, _target = 90;
  bool _init = false;
};

// The eight servos (see the servo map in the file header).
MyServo R1, R2, L1, L2;   // hips  (front-right, rear-right, front-left, rear-left)
MyServo R3, R4, L3, L4;   // lifts (front-right, rear-right, front-left, rear-left)

// ============================================================================
//  2. SETTINGS AND TUNING
// ============================================================================
// Smooth motion: servos glide toward their target angle at this speed (degrees per
// millisecond). 0.6 = a 90-degree swing in 150 ms. Raise for snappier motion, lower
// for gentler motion. A very high value (e.g. 5) effectively turns smoothing off.

const float SPEED_GAIT  = 0.6;    // walking, turning, crab, dance, etc.
const float SPEED_SIT   = 0.30;   // sitting down
const float SPEED_SLEEP = 0.22;   // lowering to sleep
const float SPEED_DEAD  = 0.50;   // play dead flop
float servoSpeed = SPEED_GAIT;   // current glide speed, chosen from the robot's state in loop()

// Advances all 8 servos toward their targets. Must be called often (every loop() pass
// and inside every wait) - the time since the last call decides how far they move.
void servosUpdate() {
  static unsigned long last = 0;
  unsigned long n = millis();
  unsigned long dt = n - last;
  if (dt == 0) return;
  last = n;
  if (dt > 50) dt = 50;
  float m = servoSpeed * dt;
  R1.step(m); R2.step(m); L1.step(m); L2.step(m);
  R3.step(m); R4.step(m); L3.step(m); L4.step(m);
}

int frameDelay = 300;     // ms pause after returning to the stand pose (switching actions, waking up)

// Walking speed from the left stick: the further it is pushed, the shorter each gait frame
const int SLOW_MS = 450;  // frame time when the stick is barely past its threshold
const int FAST_MS = 220;  // frame time at full push (don't go much lower, servos need time to arrive)

// Walk drift correction. Mechanical differences between the two sides make the robot
// veer, so one side takes a shorter stride. Value is the fraction removed from that stride:
//   positive = shorten the RIGHT side (fixes veering left)
//   negative = shorten the LEFT side  (fixes veering right)
//   0        = no correction. Applies to walking forward and backward only.
const float steer = 0.25;

// ============================================================================
//  3. CONTROLLER
// ============================================================================
// Bluepad32 callbacks, rumble, and turning the sticks/buttons into one action code.
ControllerPtr myController = nullptr;   // the connected controller (nullptr = none)
int connMode = 0;                 // 0 searching, 1 just connected, 2 just lost (drives the connection faces)
unsigned long connAt = 0;         // when connMode last changed (millis)

// Bluepad32 calls this when a controller connects.
void onConnectedController(ControllerPtr ctl) {
  myController = ctl;
  Serial.println("Controller connected!");
  connMode = 1; connAt = millis();
  ctl->playDualRumble(0, 300, 0x80, 0x40);   // welcome buzz
}

// Bluepad32 calls this when the controller disconnects.
void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) myController = nullptr;
  Serial.println("Controller disconnected!");
  connMode = 2; connAt = millis();
}

// Short controller rumble. weak / strong are the two motor strengths (0-255).
void buzz(int ms, int weak, int strong) {
  if (myController && myController->isConnected()) myController->playDualRumble(0, ms, weak, strong);
}

// Action codes returned by readDirection(). Plain ints, not an enum (see file header).
const int D_NONE   = 0,    // no input: hold the stand pose
          D_FWD    = 1,    // left stick up
          D_BACK   = 2,    // left stick down
          D_LEFT   = 3,    // left stick left  (turn counter-clockwise)
          D_RIGHT  = 4,    // left stick right (turn clockwise)
          D_WAVE   = 5,    // Y held
          D_DANCE  = 6,    // B held
          D_PUSHUP = 7,    // D-pad up held
          D_CRABL  = 8,    // left bumper held
          D_CRABR  = 9,    // right bumper held
          D_SHAKE  = 10,   // D-pad left held
          D_BOW    = 11;   // D-pad right held

// Reads the controller and returns ONE action code. Held buttons win over the stick.
// Returns D_NONE if no controller is connected or nothing is pressed.
// Stick range is about -512..512; thresholds below are the dead zones.
int readDirection() {
  if (!myController || !myController->isConnected()) return D_NONE;

  // Held buttons have priority over the stick: Y = wave, B = dance, D-pad UP = push-ups
  if (myController->y()) return D_WAVE;
  if (myController->b()) return D_DANCE;
  if (myController->dpad() & 0x01) return D_PUSHUP;   // DPAD_UP
  if (myController->l1()) return D_CRABL;             // left bumper = crab walk left
  if (myController->r1()) return D_CRABR;             // right bumper = crab walk right
  if (myController->dpad() & 0x08) return D_SHAKE;    // DPAD_LEFT  = shake
  if (myController->dpad() & 0x04) return D_BOW;      // DPAD_RIGHT = bow (held)

  int x = myController->axisX();
  int y = myController->axisY();

  // dead zones: 120 on the Y axis, 100 on the X axis (stick up is negative Y)
  bool fwd   = (y <= -120);
  bool back  = (y >=  120);
  bool left  = (x <= -100);
  bool right = (x >=  100);

  // if both axes are pushed, the bigger push wins
  if ((fwd || back) && (left || right)) {
    if (abs(y) >= abs(x)) { left = false; right = false; }
    else                  { fwd = false;  back = false;  }
  }

  if (fwd)   return D_FWD;
  if (back)  return D_BACK;
  if (left)  return D_LEFT;
  if (right) return D_RIGHT;
  return D_NONE;
}

// ============================================================================
//  4. OLED FACE
// ============================================================================
// Every face is drawn with code (rectangles, circles, lines) - no image files.
//
// How it works:
//   face   = the face the robot WANTS to show (set from loop())
//   shown  = the face actually on screen (inside updateFace()). When `face` changes,
//            the lids blink shut, `shown` swaps, and the lids open again.
//   Redraws are rate-limited to ~16 fps (faster during a blink) so the display never
//   holds up the servos.
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOk = false;   // false if the display was not found at startup (the face is then skipped)

// Face codes. The names point to where the face LOOKS on screen.

const int F_IDLE   = 0,    // random idle animations (see drawIdle)
          F_FWD    = 1,    // walking forward: determined, bouncing eyes
          F_BACK   = 2,    // walking backward: worried, nervous eyes
          F_LEFT   = 3,    // turning: eyes and arrows to the left (dizzy after a long spin)
          F_RIGHT  = 4,    // turning: eyes and arrows to the right
          F_SIT    = 5,    // sitting: relaxed, little smile
          F_WAVE   = 6,    // waving: happy ^ ^ eyes
          F_SLEEP  = 7,    // sleeping: breathing eyes, floating z's
          F_DANCE  = 8,    // dancing: swaying eyes, music notes
          F_PUSHUP = 9,    // push-ups: squeezed eyes, gritted teeth, sweat
          F_DEAD   = 10,   // playing dead: X eyes, tongue out
          F_CRABL  = 11,   // crab walk: sly side-eye, smirk
          F_CRABR  = 12,
          F_SHAKE  = 13,   // shaking: vibrating eyes
          F_BOW    = 14,   // bowing: polite closed eyes
          F_SEARCH = 15,   // no controller: eyes scan, dots count up
          F_HELLO  = 16,   // controller just connected: happy greeting
          F_SAD    = 17;   // controller just lost: droopy eyes, frown, tear
bool sprinting = false, sneaking = false;   // right / left trigger held past ~30% (draws overlays on movement faces)
int face = F_IDLE;                           // the face we WANT to show; updateFace() blinks over to it

// Right stick eye control: eyeActive while the stick is pushed; eyeTX/eyeTY are the
// target offsets in pixels and eyeX/eyeY ease toward them for smooth movement.
bool eyeActive = false;
float eyeTX = 0, eyeTY = 0, eyeX = 0, eyeY = 0;
int forceIdleAnim = 0;      // set to an idle animation number to play it next (used for the pre-sleep yawn)

const int EYE_LX = 36, EYE_RX = 92;   // x centre of the left and right eye

// ---- drawing helpers ----

// Basic eye: a white rounded rectangle centred on (cx, cy), w wide and h tall.
void eye(int cx, int cy, int w, int h) {
  if (h < 4) h = 4;
  display.fillRoundRect(cx - w / 2, cy - h / 2, w, h, min(10, h / 2), SSD1306_WHITE);
}

// Black triangle over one top corner of an eye = brow.
// innerDown=true: angry/determined brows.  innerDown=false: worried brows.
void brow(int cx, int top, int w, bool leftEye, bool innerDown, int depth) {
  int x0 = cx - w / 2 - 1, x1 = cx + w / 2 + 1;
  bool coverRight = leftEye ? innerDown : !innerDown;
  if (coverRight) display.fillTriangle(x0, top - 1, x1, top - 1, x1, top + depth, SSD1306_BLACK);
  else            display.fillTriangle(x0, top - 1, x1, top - 1, x0, top + depth, SSD1306_BLACK);
}

// Happy eye: a thick ^ shape centred on (cx, cy).
void happyEye(int cx, int cy) {
  for (int d = -1; d <= 1; d++) {
    display.drawLine(cx - 14, cy + 8 + d, cx, cy - 8 + d, SSD1306_WHITE);
    display.drawLine(cx, cy - 8 + d, cx + 14, cy + 8 + d, SSD1306_WHITE);
  }
}

// Three arrows along the bottom with one lit in turn, running in the turn direction.
// `col` lets the arrows be drawn black (on top of the ninja mask) or white.
void chevronsCol(bool left, unsigned long t, uint16_t col) {
  int active = (t / 150) % 3;
  for (int i = 0; i < 3; i++) {
    int x = 50 + i * 14;
    int phase = left ? 2 - i : i;
    if (left) {
      if (phase == active) display.fillTriangle(x, 58, x + 8, 53, x + 8, 63, col);
      else                 display.drawTriangle(x, 58, x + 8, 53, x + 8, 63, col);
    } else {
      if (phase == active) display.fillTriangle(x + 8, 58, x, 53, x, 63, col);
      else                 display.drawTriangle(x + 8, 58, x, 53, x, 63, col);
    }
  }
}

// White version of chevronsCol().
void chevrons(bool left, unsigned long t) { chevronsCol(left, t, SSD1306_WHITE); }

// Dizzy eye: two rings and a spinning line (`ccw` sets the spin direction).
void dizzyEye(int cx, int cy, unsigned long t, bool ccw) {
  display.drawCircle(cx, cy, 13, SSD1306_WHITE);
  display.drawCircle(cx, cy, 7, SSD1306_WHITE);
  float a = (t / 90.0) * (ccw ? -1 : 1);
  display.drawLine(cx, cy, cx + (int)(13 * cos(a)), cy + (int)(13 * sin(a)), SSD1306_WHITE);
}

// ============================================================================
//  IDLE ANIMATIONS
// ============================================================================
// Played at random when there is no input. The calm base face glances around and
// blinks; every few seconds one of 24 short animations plays, then it returns to calm.
// Smile: a white disc with a slightly raised black disc cut out of it leaves a crescent.
void smile(int cx, int cy, int r) {                 // white disc minus a shifted black disc = smile
  display.fillCircle(cx, cy, r, SSD1306_WHITE);
  display.fillCircle(cx, cy - 3, r, SSD1306_BLACK);
}

// Heart shape made of two circles and a triangle (used by the heart-eyes animation).
void heart(int cx, int cy, int r) {
  display.fillCircle(cx - r + 1, cy - r / 2, r, SSD1306_WHITE);
  display.fillCircle(cx + r - 1, cy - r / 2, r, SSD1306_WHITE);
  display.fillTriangle(cx - 2 * r + 2, cy - r / 2 + 2, cx + 2 * r - 2, cy - r / 2 + 2, cx, cy + 2 * r, SSD1306_WHITE);
}

// Eye that reacts to the right stick (eyeX / eyeY, about +-20 / +-14 px) with some depth.
// `leftEye` says which of the two eyes this is, so the near and far eye can differ:
//  - the eye on the side you look toward grows, the far eye shrinks (perspective)
//  - looking up opens the eyes wider, looking down droops the lid
//  - a pupil with a tiny shine slides inside the eye and grows as you look further
void lookEye(int cx, int cy, int w, int h, bool leftEye) {
  float nx = constrain(eyeX / 20.0, -1.0, 1.0);
  float ny = constrain(eyeY / 14.0, -1.0, 1.0);
  float ax = fabs(nx);
  bool toward = leftEye ? (nx < 0) : (nx > 0);
  if (toward) { w += (int)(4 * ax); h += (int)(4 * ax); }
  else        { w -= (int)(8 * ax); h -= (int)(4 * ax); }
  if (ny < 0) h += (int)(-ny * 6);
  else        h -= (int)(ny * 6);
  if (h < 4) h = 4;
  int x = constrain(cx + (int)eyeX, w / 2 + 1, 127 - w / 2);
  int y = constrain(cy + (int)eyeY, h / 2, 63 - h / 2);
  eye(x, y, w, h);
  if (h > 14) {
    if (ny > 0) display.fillRect(x - w / 2, y - h / 2, w, (int)(ny * 6), SSD1306_BLACK);   // drooping lid
    float mag = ax; if (fabs(ny) > mag) mag = fabs(ny);
    float k = mag * 1.6; if (k > 1.0) k = 1.0;
    int pr = (int)(5 * k);
    if (pr >= 1) {
      int px = x + (int)(nx * 8), py = y + (int)(ny * 6);
      display.fillCircle(px, py, pr, SSD1306_BLACK);
      if (pr >= 3) display.fillCircle(px - 1, py - 1, 1, SSD1306_WHITE);   // shine
    }
  }
}

// Idle animation numbers (index into IDLE_DUR):
//   0 calm base face            1 yawn              2 curious           3 happy bounce
//   4 wink                      5 eye roll          6 surprised         7 nervous
//   8 suspicious                9 whistling        10 heart eyes       11 bored
//  12 thinking                 13 sneeze           14 excited wiggle   15 stretch
//  16 startled hop             17 looking at feet  18 daydreaming      19 giggle
//  20 sparkle eyes             21 smug             22 pout             23 chasing a fly
//  24 hiccups
const int IDLE_COUNT = 25;   // 0 = calm base face, 1..24 = the random extras
// How long each animation plays, in milliseconds (index = animation number)
const unsigned long IDLE_DUR[IDLE_COUNT] = { 0, 2600, 3200, 2600, 2200, 2800, 2000, 3000, 3000, 3200, 2400, 3600, 3600, 2600, 2400,
                                             3000, 1800, 3200, 4000, 2400, 2600, 3000, 3000, 4500, 3400 };

// Draws the current idle frame. A small state machine decides what plays:
//   calm face -> (random wait) -> random extra animation -> calm face -> ...
// t = current time (ms), age = how long the idle face has been on screen.
// The right stick pauses the extras so you can steer the eyes by hand.
void drawIdle(unsigned long t, unsigned long age) {
  static int anim = 0, lastAnim = 0;           // current animation (0 = calm) and the last extra played
  static unsigned long start = 0, nextAt = 0;  // when the current one started / when the next may start

  // just came back to idle: calm face first, then the first extra after a few seconds
  if (age < 100 || nextAt == 0) { anim = 0; nextAt = t + random(2500, 5000); }

  // right stick = take over the eyes: cancel any extra animation and hold off new ones
  if (eyeActive) { anim = 0; nextAt = t + 3000; }
  // a queued animation (used for the pre-sleep yawn) starts right away
  else if (forceIdleAnim > 0 && anim == 0) { anim = forceIdleAnim; start = t; forceIdleAnim = 0; }

  // time for a new extra: pick a random one, never the same as last time
  if (anim == 0 && t >= nextAt) {
    int pick;
    do { pick = random(1, IDLE_COUNT); } while (pick == lastAnim);
    anim = pick; start = t;
  } else if (anim != 0 && (t - start) > IDLE_DUR[anim]) {
    lastAnim = anim; anim = 0; nextAt = t + random(3000, 7000);
  }
  unsigned long a = t - start;   // ms into the current animation

  switch (anim) {
    case 0: {                                       // calm: glances around and blinks
      const int gx[6] = { 0, 0, -8, 0, 8, 0 };
      int lx = eyeActive ? 0 : gx[(t / 1500) % 6];
      int h = 36;
      if ((t % 4200) < 140) h = 6;
      lookEye(EYE_LX + lx, 32, 32, h, true);
      lookEye(EYE_RX + lx, 32, 32, h, false);
      break;
    }
    case 1: {                                       // yawn: eyes droop shut, mouth opens, then recover
      float q = a / 2600.0;
      int h, mr;
      if (q < 0.35)      { float r = q / 0.35;         h = 36 - (int)(28 * r); mr = (int)(9 * r); }
      else if (q < 0.7)  { h = 8; mr = 9; }
      else               { float r = (q - 0.7) / 0.3;  h = 8 + (int)(28 * r);  mr = 9 - (int)(9 * r); }
      eye(EYE_LX, 26, 32, h);
      eye(EYE_RX, 26, 32, h);
      if (mr > 1) display.fillCircle(64, 52, mr, SSD1306_WHITE);
      break;
    }
    case 2: {                                       // curious: looks up and to one side, then the other
      int side = ((a / 1500) % 2) ? 1 : -1;
      int lx = side * 8;
      eye(EYE_LX + lx, 24, 32, side < 0 ? 40 : 30);
      eye(EYE_RX + lx, 24, 32, side < 0 ? 30 : 40);
      break;
    }
    case 3: {                                       // happy bounce
      int bounce = ((a / 180) % 2) ? -3 : 0;
      happyEye(EYE_LX, 26 + bounce);
      happyEye(EYE_RX, 26 + bounce);
      smile(64, 52, 10);
      break;
    }
    case 4: {                                       // wink and smirk
      bool wink = (a > 500 && a < 1500);
      eye(EYE_LX, 30, 32, 36);
      if (wink) happyEye(EYE_RX, 30); else eye(EYE_RX, 30, 32, 36);
      if (wink) smile(70, 52, 8);
      break;
    }
    case 5: {                                       // eye roll
      float ang = a / 280.0;
      int lx = (int)(10 * cos(ang)), ly = (int)(7 * sin(ang));
      eye(EYE_LX + lx, 30 + ly, 32, 34);
      eye(EYE_RX + lx, 30 + ly, 32, 34);
      break;
    }
    case 6: {                                       // surprised: jolt, tall eyes, little "o", exclamation mark
      int j = (a < 300) ? random(-2, 3) : 0;
      eye(EYE_LX + j, 26, 30, 44);
      eye(EYE_RX + j, 26, 30, 44);
      display.fillCircle(64, 56, 6, SSD1306_WHITE);
      display.fillCircle(64, 56, 3, SSD1306_BLACK);
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(114, 2);
      display.print("!");
      break;
    }
    case 7: {                                       // nervous: darting glances, worried brows
      const int nx[4] = { -10, 10, -6, 8 };
      int lx = nx[(a / 350) % 4];
      eye(EYE_LX + lx, 32, 30, 34);
      eye(EYE_RX + lx, 32, 30, 34);
      brow(EYE_LX + lx, 15, 30, true, false, 10);
      brow(EYE_RX + lx, 15, 30, false, false, 10);
      break;
    }
    case 8: {                                       // suspicious squint, side-eye
      int lx = ((a / 1200) % 2) ? 10 : -10;
      eye(EYE_LX + lx, 32, 32, 20);
      eye(EYE_RX + lx, 32, 32, 20);
      brow(EYE_LX + lx, 22, 32, true, true, 6);
      brow(EYE_RX + lx, 22, 32, false, true, 6);
      display.fillRect(54, 52, 20, 2, SSD1306_WHITE);
      break;
    }
    case 9: {                                       // whistling a tune
      happyEye(EYE_LX, 26);
      happyEye(EYE_RX, 26);
      display.fillCircle(64, 52, 6, SSD1306_WHITE);
      display.fillCircle(64, 52, 3, SSD1306_BLACK);
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(84 + (int)(6 * sin(a / 200.0)), 44 - (int)((a / 50) % 36));
      display.write(14);
      break;
    }
    case 10: {                                      // heart eyes
      int sz = 6 + (((a / 200) % 2) ? 1 : 0);
      heart(EYE_LX, 28, sz);
      heart(EYE_RX, 28, sz);
      smile(64, 54, 9);
      break;
    }
    case 11: {                                      // bored: heavy lids, drifting eyes, slow dots
      int lx = (int)(6 * sin(a / 900.0));
      eye(EYE_LX + lx, 34, 32, 30);
      eye(EYE_RX + lx, 34, 32, 30);
      display.fillRect(EYE_LX + lx - 17, 18, 34, 16, SSD1306_BLACK);
      display.fillRect(EYE_RX + lx - 17, 18, 34, 16, SSD1306_BLACK);
      display.fillRect(54, 54, 20, 2, SSD1306_WHITE);
      for (int i = 0; i < (int)((a / 600) % 4); i++) display.fillRect(92 + i * 8, 58, 3, 3, SSD1306_WHITE);
      break;
    }
    case 12: {                                      // thinking: eyes up and away, thought bubbles grow
      eye(EYE_LX - 8, 26, 32, 34);
      eye(EYE_RX - 8, 26, 32, 34);
      int n = (a / 500) % 4;
      if (n >= 1) display.fillCircle(108, 40, 2, SSD1306_WHITE);
      if (n >= 2) display.fillCircle(114, 28, 3, SSD1306_WHITE);
      if (n >= 3) display.fillCircle(118, 12, 5, SSD1306_WHITE);
      display.fillRect(54, 54, 12, 2, SSD1306_WHITE);
      break;
    }
    case 13: {                                      // sneeze: build up, "> <" burst, recover
      if (a < 1200) {
        float r = a / 1200.0;
        int h = 36 - (int)(14 * r);
        int sh = (int)(r * 3);
        int j = random(-sh, sh + 1);
        eye(EYE_LX + j, 30, 32, h);
        eye(EYE_RX + j, 30, 32, h);
        display.fillCircle(64, 52, 3 + (int)(4 * r), SSD1306_WHITE);
      } else if (a < 1700) {
        for (int d = -1; d <= 1; d++) {
          display.drawLine(EYE_LX - 12, 18 + d, EYE_LX + 8, 30 + d, SSD1306_WHITE);
          display.drawLine(EYE_LX + 8, 30 + d, EYE_LX - 12, 42 + d, SSD1306_WHITE);
          display.drawLine(EYE_RX + 12, 18 + d, EYE_RX - 8, 30 + d, SSD1306_WHITE);
          display.drawLine(EYE_RX - 8, 30 + d, EYE_RX + 12, 42 + d, SSD1306_WHITE);
        }
        display.fillCircle(64, 52, 9, SSD1306_WHITE);
        display.drawLine(22, 44, 8, 48, SSD1306_WHITE);
        display.drawLine(106, 44, 120, 48, SSD1306_WHITE);
        display.drawLine(26, 54, 12, 60, SSD1306_WHITE);
        display.drawLine(102, 54, 116, 60, SSD1306_WHITE);
      } else {
        float r = (a - 1700) / 900.0;
        int h = 8 + (int)(28 * r);
        eye(EYE_LX, 30, 32, h);
        eye(EYE_RX, 30, 32, h);
      }
      break;
    }
    case 14: {                                      // excited wiggle
      int ox = ((a / 120) % 2) ? 6 : -6;
      eye(EYE_LX + ox, 26, 30, 40);
      eye(EYE_RX + ox, 26, 30, 40);
      display.fillCircle(64 + ox, 52, 9, SSD1306_WHITE);        // open smile = half disc
      display.fillRect(54 + ox, 40, 22, 12, SSD1306_BLACK);
      break;
    }
    case 15: {                                      // stretch: squeeze, mouth opens wide, relax
      float q = a / 3000.0;
      int h = 36, mr = 0;
      bool squeeze = false;
      if (q < 0.25)     { float r = q / 0.25;        h = 36 - (int)(26 * r); mr = (int)(8 * r); }
      else if (q < 0.7) { squeeze = true; mr = 10; }
      else              { float r = (q - 0.7) / 0.3; h = 10 + (int)(26 * r); mr = 10 - (int)(10 * r); }
      if (squeeze) {
        for (int d = -1; d <= 1; d++) {
          display.drawLine(EYE_LX - 12, 16 + d, EYE_LX + 8, 28 + d, SSD1306_WHITE);
          display.drawLine(EYE_LX + 8, 28 + d, EYE_LX - 12, 40 + d, SSD1306_WHITE);
          display.drawLine(EYE_RX + 12, 16 + d, EYE_RX - 8, 28 + d, SSD1306_WHITE);
          display.drawLine(EYE_RX - 8, 28 + d, EYE_RX + 12, 40 + d, SSD1306_WHITE);
        }
        display.drawLine(6, 18, 16, 22, SSD1306_WHITE);  display.drawLine(122, 18, 112, 22, SSD1306_WHITE);
        display.drawLine(6, 36, 16, 32, SSD1306_WHITE);  display.drawLine(122, 36, 112, 32, SSD1306_WHITE);
      } else {
        eye(EYE_LX, 28, 32, h);
        eye(EYE_RX, 28, 32, h);
      }
      if (mr > 1) display.fillCircle(64, 52, mr, SSD1306_WHITE);
      break;
    }
    case 16: {                                      // startled hop, sweat drop
      int hop = (a < 500) ? -(int)(6 * sin(a / 500.0 * 3.14159)) : 0;
      int j = (a < 250) ? random(-2, 3) : 0;
      eye(EYE_LX + j, 28 + hop, 30, 40);
      eye(EYE_RX + j, 28 + hop, 30, 40);
      display.fillCircle(64, 56, 5, SSD1306_WHITE);
      display.fillCircle(64, 56, 2, SSD1306_BLACK);
      if (a > 300) display.fillCircle(112, 8 + (int)((a / 35) % 22), 2, SSD1306_WHITE);
      break;
    }
    case 17: {                                      // looking down at its own feet
      int lx = ((a / 800) % 2) ? -8 : 8;
      eye(EYE_LX + lx, 40, 32, 22);
      eye(EYE_RX + lx, 40, 32, 22);
      brow(EYE_LX + lx, 29, 32, true, false, 6);
      brow(EYE_RX + lx, 29, 32, false, false, 6);
      display.fillRect(56, 56, 16, 2, SSD1306_WHITE);
      break;
    }
    case 18: {                                      // daydreaming: dreamy half-lids, bubbles drift up
      int lx = 6 + (int)(3 * sin(a / 600.0));
      eye(EYE_LX + lx, 34, 32, 28);
      eye(EYE_RX + lx, 34, 32, 28);
      display.fillRect(EYE_LX + lx - 17, 19, 34, 10, SSD1306_BLACK);
      display.fillRect(EYE_RX + lx - 17, 19, 34, 10, SSD1306_BLACK);
      smile(64, 56, 7);
      for (int i = 0; i < 3; i++) {
        unsigned long ph = (a + i * 1300) % 3900;
        int by = 56 - (int)(ph / 70);
        int bx = 108 + (int)(4 * sin((a + i * 500) / 300.0));
        if (by > 2) display.drawCircle(bx, by, 2 + i, SSD1306_WHITE);
      }
      break;
    }
    case 19: {                                      // giggle: shaking happy eyes, open grin
      int sh = ((a / 60) % 2) ? 2 : -2;
      int bn = ((a / 120) % 2) ? -2 : 0;
      happyEye(EYE_LX + sh, 24 + bn);
      happyEye(EYE_RX + sh, 24 + bn);
      display.fillCircle(64 + sh, 52, 11, SSD1306_WHITE);
      display.fillRect(50 + sh, 38, 28, 14, SSD1306_BLACK);
      break;
    }
    case 20: {                                      // sparkle eyes: sees something shiny
      int sz = 5 + (((a / 150) % 2) ? 3 : 0);
      eye(EYE_LX, 28, 34, 40);
      eye(EYE_RX, 28, 34, 40);
      for (int e = 0; e < 2; e++) {
        int cx = e ? EYE_RX : EYE_LX;
        display.drawLine(cx - sz, 28, cx + sz, 28, SSD1306_BLACK);
        display.drawLine(cx, 28 - sz, cx, 28 + sz, SSD1306_BLACK);
        display.drawLine(cx - sz / 2, 28 - sz / 2, cx + sz / 2, 28 + sz / 2, SSD1306_BLACK);
        display.drawLine(cx - sz / 2, 28 + sz / 2, cx + sz / 2, 28 - sz / 2, SSD1306_BLACK);
      }
      smile(64, 56, 9);
      break;
    }
    case 21: {                                      // smug: half-lids, one raised brow, side smirk
      int lx = (int)(5 * sin(a / 700.0));
      eye(EYE_LX + lx, 32, 32, 30);
      eye(EYE_RX + lx, 32, 32, 30);
      display.fillRect(EYE_LX + lx - 17, 16, 34, 14, SSD1306_BLACK);
      display.fillRect(EYE_RX + lx - 17, 16, 34, 14, SSD1306_BLACK);
      for (int d = 0; d < 2; d++) display.drawLine(EYE_LX + lx - 14, 12 + d, EYE_LX + lx + 14, 6 + d, SSD1306_WHITE);
      smile(74, 54, 9);
      break;
    }
    case 22: {                                      // pout: looks away, puffed cheeks
      int lx = ((a / 1500) % 2) ? 8 : -8;
      eye(EYE_LX + lx, 30, 32, 24);
      eye(EYE_RX + lx, 30, 32, 24);
      brow(EYE_LX + lx, 18, 32, true, true, 10);
      brow(EYE_RX + lx, 18, 32, false, true, 10);
      display.drawCircle(14, 46, 9, SSD1306_WHITE);
      display.drawCircle(114, 46, 9, SSD1306_WHITE);
      display.fillRect(56, 52, 16, 2, SSD1306_WHITE);
      break;
    }
    case 23: {                                      // chasing a fly with its eyes
      float fa = a / 500.0;
      int fx = 64 + (int)(44 * sin(fa));
      int fy = 14 + (int)(10 * sin(fa * 2.3));
      int lx = constrain((fx - 64) / 4, -12, 12);
      int ly = constrain((fy - 30) / 3, -8, 0);
      eye(EYE_LX + lx, 34 + ly, 32, 34);
      eye(EYE_RX + lx, 34 + ly, 32, 34);
      display.fillRect(fx - 1, fy - 1, 5, 5, SSD1306_BLACK);
      display.fillRect(fx, fy, 3, 3, SSD1306_WHITE);
      if ((a / 60) % 2) { display.drawPixel(fx - 2, fy - 1, SSD1306_WHITE); display.drawPixel(fx + 4, fy - 1, SSD1306_WHITE); }
      break;
    }
    case 24: {                                      // hiccups
      unsigned long ph = a % 850;
      bool hic = (ph < 140);
      int hop = hic ? -5 : 0;
      eye(EYE_LX, 32 + hop, 32, hic ? 20 : 36);
      eye(EYE_RX, 32 + hop, 32, hic ? 20 : 36);
      if (hic) { display.fillCircle(64, 54, 4, SSD1306_WHITE); display.fillCircle(64, 54, 2, SSD1306_BLACK); }
      if (ph < 300) {
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(96, 50);
        display.print("hic");
      }
      break;
    }
  }
}

// The face engine. Call it often (it throttles itself). Each call:
//   1. runs the blink transition if `face` changed,
//   2. draws the face for `shown`,
//   3. adds overlays (sprint speed lines, ninja mask for sneak) and the blink lids,
//   4. pushes the frame to the display.
void updateFace() {
  if (!oledOk) return;
  static unsigned long last = 0;
  unsigned long t = millis();
  static int shown = F_IDLE;           // the face actually on screen (lags `face` during a blink)
  static int tstate = 0;               // blink transition: 0 none, 1 closing, 2 opening
  static unsigned long tStart = 0;
  const unsigned long BLINK_HALF = 140;   // ms for lids to close, and again to open
  if (t - last < (tstate ? 30 : 60)) return;   // ~16 fps, faster during a blink
  last = t;
  eyeX += (eyeTX - eyeX) * 0.4;
  eyeY += (eyeTY - eyeY) * 0.4;

  float k = 0;                         // how closed the lids are, 0..1
  if (tstate == 0 && face != shown) { tstate = 1; tStart = t; }
  if (tstate == 1) {
    k = (t - tStart) / (float)BLINK_HALF;
    if (k >= 1) { k = 1; shown = face; tstate = 2; tStart = t; }
  } else if (tstate == 2) {
    k = 1.0 - (t - tStart) / (float)BLINK_HALF;
    if (k <= 0) { k = 0; tstate = 0; }
    else if (face != shown) { tstate = 1; tStart = t - (unsigned long)(k * BLINK_HALF); }   // changed again mid-blink
  }

  static int lastFace = -1;
  static unsigned long faceSince = 0;
  if (shown != lastFace) { lastFace = shown; faceSince = t; }
  unsigned long age = t - faceSince;

  display.clearDisplay();
  int lookX = 0, lookY = 0, w = 32, h = 36, cy = 32;   // shared scratch values for the face cases

  // draw the face that is currently on screen
  switch (shown) {
    case F_IDLE: {                                  // random idle animations
      drawIdle(t, age);
      break;
    }
    case F_FWD: {                                   // determined, bouncing
      w = 34; h = 38;
      lookY = -6 + (((t / 150) % 2) ? 2 : -2);
      eye(EYE_LX, cy + lookY, w, h);
      eye(EYE_RX, cy + lookY, w, h);
      int top = cy + lookY - h / 2;
      brow(EYE_LX, top, w, true, true, 12);
      brow(EYE_RX, top, w, false, true, 12);
      break;
    }
    case F_BACK: {                                  // worried, nervous shuffle
      w = 30; h = 34;
      lookY = 4;
      lookX = ((t / 120) % 2) ? 2 : -2;
      eye(EYE_LX + lookX, cy + lookY, w, h);
      eye(EYE_RX + lookX, cy + lookY, w, h);
      int top = cy + lookY - h / 2;
      brow(EYE_LX + lookX, top, w, true, false, 12);
      brow(EYE_RX + lookX, top, w, false, false, 12);
      break;
    }
    case F_LEFT:
    case F_RIGHT: {                                 // focused turn, speed trails, gets dizzy if it keeps spinning
      bool left = (shown == F_LEFT);
      int dir = left ? -1 : 1;
      if (age > 2500) {                             // dizzy after a long spin
        dizzyEye(EYE_LX, 26, t, left);
        dizzyEye(EYE_RX, 26, t, left);
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(56, 50 - (int)((t / 120) % 8)); display.print("?");   // floating question mark
      } else {
        lookX = dir * 10;
        cy = 24; h = 26;
        int lean = dir * (((t / 160) % 2) ? 1 : 0);        // little head-lean wobble
        // ghost outlines trailing behind = motion
        display.drawRoundRect(EYE_LX + lookX - dir * 9, cy - h / 2, w, h, 8, SSD1306_WHITE);
        display.drawRoundRect(EYE_RX + lookX - dir * 9, cy - h / 2, w, h, 8, SSD1306_WHITE);
        eye(EYE_LX + lookX + lean, cy, w, h);
        eye(EYE_RX + lookX + lean, cy, w, h);
        int top = cy - h / 2;
        brow(EYE_LX + lookX + lean, top, w, true, true, 8);    // focused brows
        brow(EYE_RX + lookX + lean, top, w, false, true, 8);
        display.fillRect(58 + dir * 6, 46, 12, 2, SSD1306_WHITE);   // determined little mouth
      }
      chevrons(left, t);
      break;
    }
    case F_SIT: {                                   // relaxed: eyes glance slowly, little smile
      w = 30; h = 28; cy = 24;
      const int gx[4] = { 0, -6, 0, 6 };
      lookX = eyeActive ? 0 : gx[(t / 1400) % 4];
      if ((t % 4600) < 140) h = 6;                  // blink
      lookEye(EYE_LX + lookX, cy, w, h, true);
      lookEye(EYE_RX + lookX, cy, w, h, false);
      display.fillCircle(64, 48, 11, SSD1306_WHITE);   // smile = white disc minus a black disc
      display.fillCircle(64, 43, 11, SSD1306_BLACK);
      break;
    }
    case F_SLEEP: {                                 // sleepy, breathing
      w = 34;
      h = 16 + (int)(2.5 * sin(t / 500.0));
      cy = 38;
      eye(EYE_LX, cy, w, h);
      eye(EYE_RX, cy, w, h);
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      int z = (t / 800) % 3;
      if (z >= 1) { display.setCursor(98, 16); display.print("z"); }
      if (z >= 2) { display.setTextSize(2); display.setCursor(108, 2); display.print("Z"); }
      break;
    }
    case F_DANCE: {                                 // happy, swaying, music notes
      int sway = (int)(6 * sin(t / 200.0));
      happyEye(EYE_LX + sway, 26);
      happyEye(EYE_RX + sway, 26);
      display.fillCircle(64 + sway, 52, 9, SSD1306_WHITE);   // big smile
      display.fillCircle(64 + sway, 47, 9, SSD1306_BLACK);
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(2 + (((t / 300) % 2) ? 4 : 0), 2);   display.write(14);
      display.setCursor(106 - (((t / 300) % 2) ? 4 : 0), 34); display.write(13);
      break;
    }
    case F_PUSHUP: {                                // squeezed eyes, gritted teeth, sweat drop
      eye(EYE_LX, 22, 34, 8);
      eye(EYE_RX, 22, 34, 8);
      display.drawRect(50, 44, 28, 12, SSD1306_WHITE);
      for (int x = 57; x < 78; x += 7) display.drawLine(x, 44, x, 56, SSD1306_WHITE);
      display.drawLine(50, 50, 78, 50, SSD1306_WHITE);
      int drop = (t / 40) % 24;
      display.fillCircle(112, 6 + drop, 3, SSD1306_WHITE);
      break;
    }
    case F_DEAD: {                                  // X eyes and a tongue
      for (int d = -1; d <= 1; d++) {
        display.drawLine(EYE_LX - 12 + d, 14, EYE_LX + 12 + d, 38, SSD1306_WHITE);
        display.drawLine(EYE_LX + 12 + d, 14, EYE_LX - 12 + d, 38, SSD1306_WHITE);
        display.drawLine(EYE_RX - 12 + d, 14, EYE_RX + 12 + d, 38, SSD1306_WHITE);
        display.drawLine(EYE_RX + 12 + d, 14, EYE_RX - 12 + d, 38, SSD1306_WHITE);
      }
      display.drawLine(50, 50, 78, 50, SSD1306_WHITE);
      display.fillRoundRect(62, 50, 10, 12, 4, SSD1306_WHITE);
      break;
    }
    case F_CRABL:
    case F_CRABR: {                                 // sneaky side-step: eyes dart sideways, sly squint, smirk
      bool left = (shown == F_CRABL);
      int dir = left ? -1 : 1;
      int jitter = ((t / 100) % 2) ? 2 : -2;        // quick scuttle shake
      lookX = dir * 12 + jitter;
      cy = 24;
      int lead = 32, trail = 22;                    // leading eye wide open, trailing eye narrowed
      int leadX  = (dir < 0 ? EYE_LX : EYE_RX) + lookX;
      int trailX = (dir < 0 ? EYE_RX : EYE_LX) + lookX;
      eye(leadX, cy, 32, lead);
      eye(trailX, cy, 32, trail);
      display.fillRect(trailX - 17, cy - trail / 2 - 1, 34, 7, SSD1306_BLACK);   // heavy lid = sly look
      display.fillCircle(64 + dir * 10, 44, 7, SSD1306_WHITE);                   // smirk, pulled to the side
      display.fillCircle(64 + dir * 10, 41, 7, SSD1306_BLACK);
      chevrons(left, t);
      break;
    }
    case F_SHAKE: {                                 // vibrating eyes, wobbly mouth
      int j1 = random(-3, 4), j2 = random(-3, 4);
      eye(EYE_LX + j1, 28 + j2, 30, 30);
      eye(EYE_RX - j1, 28 - j2, 30, 30);
      for (int x = 44; x < 84; x += 8) display.drawLine(x, 52 + (((x / 8) % 2) ? 3 : -3) + j2, x + 8, 52 + (((x / 8) % 2) ? -3 : 3) + j2, SSD1306_WHITE);
      break;
    }
    case F_BOW: {                                   // polite closed eyes, little smile
      for (int d = -1; d <= 1; d++) {
        display.drawLine(EYE_LX - 14, 24 + d, EYE_LX, 38 + d, SSD1306_WHITE);
        display.drawLine(EYE_LX, 38 + d, EYE_LX + 14, 24 + d, SSD1306_WHITE);
        display.drawLine(EYE_RX - 14, 24 + d, EYE_RX, 38 + d, SSD1306_WHITE);
        display.drawLine(EYE_RX, 38 + d, EYE_RX + 14, 24 + d, SSD1306_WHITE);
      }
      display.fillCircle(64, 52, 8, SSD1306_WHITE);
      display.fillCircle(64, 49, 8, SSD1306_BLACK);
      break;
    }
    case F_SEARCH: {                                // waiting for the controller: eyes scan, dots count up
      int lx = (int)(14 * sin(t / 500.0));
      int ly = (int)(4 * sin(t / 770.0));
      eye(EYE_LX + lx, 26 + ly, 32, 34);
      eye(EYE_RX + lx, 26 + ly, 32, 34);
      int n = (t / 400) % 4;
      for (int i = 0; i < n; i++) display.fillCircle(52 + i * 12, 56, 3, SSD1306_WHITE);
      break;
    }
    case F_HELLO: {                                 // controller just connected: happy greeting
      int bounce = ((t / 150) % 2) ? -3 : 0;
      happyEye(EYE_LX, 26 + bounce);
      happyEye(EYE_RX, 26 + bounce);
      display.fillCircle(64, 52, 12, SSD1306_WHITE);
      display.fillRect(48, 38, 32, 14, SSD1306_BLACK);
      int sp = (t / 200) % 2;
      for (int i = 0; i < 2; i++) {
        int sx = i ? 112 : 14, sy = 14 + sp * 4;
        display.drawLine(sx - 4, sy, sx + 4, sy, SSD1306_WHITE);
        display.drawLine(sx, sy - 4, sx, sy + 4, SSD1306_WHITE);
      }
      break;
    }
    case F_SAD: {                                   // controller lost: droopy eyes, frown, a falling tear
      eye(EYE_LX, 34, 32, 30);
      eye(EYE_RX, 34, 32, 30);
      brow(EYE_LX, 19, 32, true, false, 12);
      brow(EYE_RX, 19, 32, false, false, 12);
      display.fillCircle(64, 58, 10, SSD1306_WHITE);
      display.fillCircle(64, 61, 10, SSD1306_BLACK);
      display.fillCircle(EYE_LX + 10, 50 + (int)((t / 50) % 12), 2, SSD1306_WHITE);
      break;
    }
    case F_WAVE: {                                  // happy
      int bounce = ((t / 200) % 2) ? -3 : 0;
      happyEye(EYE_LX, cy + bounce);
      happyEye(EYE_RX, cy + bounce);
      break;
    }
  }
  // ---- overlays ----
  // sprint (right trigger): speed lines and a sweat drop on movement faces
  bool moving = (shown == F_FWD || shown == F_BACK || shown == F_LEFT || shown == F_RIGHT || shown == F_CRABL || shown == F_CRABR);
  if (moving && sprinting) {                        // speed lines and a sweat drop
    for (int i = 0; i < 3; i++) display.drawLine(0, 18 + i * 12, 10 + ((t / 60) % 3) * 4, 18 + i * 12, SSD1306_WHITE);
    display.fillCircle(118, 6 + (int)((t / 40) % 24), 3, SSD1306_WHITE);
  }
  if ((moving || shown == F_IDLE) && sneaking) {     // ninja: headband over the eyes, mask over the mouth
    display.fillRect(0, 0, 128, 12, SSD1306_BLACK);                  // eyes squint under the band
    display.fillRect(0, 12, 128, 3, SSD1306_WHITE);                  // headband
    display.fillRect(0, 15, 128, 1, SSD1306_BLACK);
    display.fillRoundRect(30, 42, 68, 22, 8, SSD1306_WHITE);         // mask
    display.drawLine(30, 46, 20, 36, SSD1306_WHITE);                 // ties up to the ears
    display.drawLine(31, 46, 21, 36, SSD1306_WHITE);
    display.drawLine(98, 46, 108, 36, SSD1306_WHITE);
    display.drawLine(97, 46, 107, 36, SSD1306_WHITE);
    display.drawLine(64, 44, 54, 51, SSD1306_BLACK);                 // pinch folds
    display.drawLine(64, 44, 74, 51, SSD1306_BLACK);
    if (shown == F_LEFT || shown == F_RIGHT || shown == F_CRABL || shown == F_CRABR)
      chevronsCol(shown == F_LEFT || shown == F_CRABL, t, SSD1306_BLACK);   // arrows ride on the mask
  }
  // blink transition: lids close over the old face, the face swaps, lids open on the new one
  if (k > 0) {
    int hgt = (int)(k * 32);
    display.fillRect(0, 0, 128, hgt, SSD1306_BLACK);
    display.fillRect(0, 64 - hgt, 128, hgt, SSD1306_BLACK);
    if (k >= 0.85) {
      display.fillRect(EYE_LX - 14, 31, 28, 2, SSD1306_WHITE);
      display.fillRect(EYE_RX - 14, 31, 28, 2, SSD1306_WHITE);
    }
  }
  display.display();
}

// ============================================================================
//  5. TIMING HELPERS
// ============================================================================
// Frame-time scaling from the triggers, and waiting functions that keep the robot
// responsive (servos gliding, controller read, face animating) while they wait.
// Scales a frame time with the triggers: RT = faster (up to 35% shorter),
// LT = slower (up to 60% longer). Result is kept between 170 and 900 ms.
int scaleMs(int ms) {
  if (!myController) return ms;
  float rt = myController->throttle() / 1023.0;
  float lt = myController->brake() / 1023.0;
  float f = 1.0 - 0.35 * rt + 0.60 * lt;
  return constrain((int)(ms * f), 170, 900);
}

// Frame time for the stick direction d: barely pushed = SLOW_MS, fully pushed = FAST_MS,
// then scaled by the triggers. Read fresh every frame, so speed changes mid-step.
int stickFrameMs(int d) {
  if (!myController) return SLOW_MS;
  int mag, thr;
  if (d == D_FWD || d == D_BACK) { mag = abs(myController->axisY()); thr = 120; }
  else                           { mag = abs(myController->axisX()); thr = 100; }
  mag = constrain(mag, thr, 512);
  return scaleMs(map(mag, thr, 512, SLOW_MS, FAST_MS));
}

// Waits `ms` milliseconds without freezing: servos keep gliding, the controller is read
// and the face keeps animating. Use this instead of delay() for any pause.
void waitMs(int ms) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    BP32.update();
    servosUpdate();
    updateFace();
    delay(5);
  }
}

// Waits one gait frame (`ms`) like waitMs(), but also checks the action is still held.
// Returns false as soon as readDirection() is no longer `d`, so every movement can stop
// the instant the stick or button is released. Returns true if the full time passed.
bool pressingCheck(int d, int ms) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    BP32.update();
    servosUpdate();
    updateFace();
    if (readDirection() != d) return false;
    delay(5);
  }
  return true;
}

// ============================================================================
//  6. POSES AND MOVEMENTS
// ============================================================================
// A pose is 8 servo angles. A movement is a table of poses played one per gait frame.
// Column order in every row:  R1  R2  L1  L2  R3  R4  L3  L4  (hips, then lifts).
// Edit the numbers in a table to change that motion.
// Stand pose ("X stance"): all hips mid-stroke, all feet down.
const int STAND[8] = { 45, 45, 45, 45, 180, 0, 0, 180 };

// Walk: a diagonal trot. Leg pair A = front-right + rear-left (R1/R3 + L2/L4),
// pair B = rear-right + front-left (R2/R4 + L1/L3). While one pair pushes on the
// ground the other is lifted and swinging forward. Each row is the whole body pose at
// the end of one frame; walking BACKWARD plays the same rows in reverse (6 to 1).
const int WALK[6][8] = {
  {   0, 90,  0, 90, 135, 45,  0, 180 },   // frame 1
  {  90, 90,  0,  0, 135,  0,  0, 135 },   // frame 2
  {  90,  0, 90,  0, 135,  0,  0, 135 },   // frame 3: pair B power stroke
  {  90,  0, 90,  0, 135, 45,  0, 180 },   // frame 4
  {  90, 90,  0,  0, 180, 45, 45, 180 },   // frame 5
  {   0, 90,  0, 90, 180, 45, 45, 180 }    // frame 6: pair A power stroke
};

// Turn counter-clockwise on the spot. Two leg sets take turns: lift, swing hips,
// plant, push back to mid-stroke (the push against the ground rotates the body).
// Rows 1-4 = set 1 (R1/R3 + L2/L4), rows 5-8 = set 2 (R2/R4 + L1/L3). Hips swing to 90.
const int TURN_CCW[8][8] = {
  { 45, 45, 45, 45, 135,  0,  0, 135 },   // set 1: lift
  { 90, 45, 45, 90, 135,  0,  0, 135 },   // set 1: swing
  { 90, 45, 45, 90, 180,  0,  0, 180 },   // set 1: plant
  { 45, 45, 45, 45, 180,  0,  0, 180 },   // set 1: push
  { 45, 45, 45, 45, 180, 45, 45, 180 },   // set 2: lift
  { 45, 90, 90, 45, 180, 45, 45, 180 },   // set 2: swing
  { 45, 90, 90, 45, 180,  0,  0, 180 },   // set 2: plant
  { 45, 45, 45, 45, 180,  0,  0, 180 }    // set 2: push
};

// Turn clockwise on the spot: same sequence as above with the hips swinging to 0.
const int TURN_CW[8][8] = {
  { 45, 45, 45, 45, 135,  0,  0, 135 },   // set 1: lift
  {  0, 45, 45,  0, 135,  0,  0, 135 },   // set 1: swing
  {  0, 45, 45,  0, 180,  0,  0, 180 },   // set 1: plant
  { 45, 45, 45, 45, 180,  0,  0, 180 },   // set 1: push
  { 45, 45, 45, 45, 180, 45, 45, 180 },   // set 2: lift
  { 45,  0,  0, 45, 180, 45, 45, 180 },   // set 2: swing
  { 45,  0,  0, 45, 180,  0,  0, 180 },   // set 2: plant
  { 45, 45, 45, 45, 180,  0,  0, 180 }    // set 2: push
};

// Sends a pose (8 angles, column order above) to the servos as new targets.
void setPose(const int* p) {
  R1.write(p[0]); R2.write(p[1]); L1.write(p[2]); L2.write(p[3]);
  R3.write(p[4]); R4.write(p[5]); L3.write(p[6]); L4.write(p[7]);
}

// Goes to the stand pose.
void standPose() {
  setPose(STAND);
}

// Shrinks a hip angle toward mid-stroke (45) by factor k, so that side takes a
// shorter stride (k = 1 leaves it unchanged).
int scaleHip(int h, float k) {
  float v = 45 + (h - 45) * k;
  return (int)(v + (v >= 0 ? 0.5 : -0.5));
}

// Same as setPose(), but for walking: applies the left/right drift correction (steer)
// and the sneak stride shortening (left trigger) to the four hips.
void setWalkPose(const int* p) {
  float sk = myController ? 1.0 - 0.30 * (myController->brake() / 1023.0) : 1.0;   // sneak = shorter strides
  float rightK = ((steer > 0) ? 1.0 - steer : 1.0) * sk;
  float leftK  = ((steer < 0) ? 1.0 + steer : 1.0) * sk;
  R1.write(scaleHip(p[0], rightK)); R2.write(scaleHip(p[1], rightK));
  L1.write(scaleHip(p[2], leftK));  L2.write(scaleHip(p[3], leftK));
  R3.write(p[4]); R4.write(p[5]); L3.write(p[6]); L4.write(p[7]);
}

// ============================================================================
//  MOVEMENTS
// ============================================================================
// Each function plays ONE cycle of its motion and returns early if the control is
// released (pressingCheck() returns false). loop() calls them again while held.
// Walk forward: plays the WALK frames 1 to 6.
void walkForward() {
  for (int i = 0; i < 6; i++) {
    setWalkPose(WALK[i]);
    if (!pressingCheck(D_FWD, stickFrameMs(D_FWD))) return;
  }
}

// Walk backward: plays the WALK frames in reverse, 6 to 1.
void walkBackward() {
  for (int i = 5; i >= 0; i--) {
    setWalkPose(WALK[i]);
    if (!pressingCheck(D_BACK, stickFrameMs(D_BACK))) return;
  }
}

// Turn on the spot, counter-clockwise (left stick left).
void turnCounterClockwise() {
  for (int i = 0; i < 8; i++) {
    setPose(TURN_CCW[i]);
    if (!pressingCheck(D_LEFT, stickFrameMs(D_LEFT))) return;
  }
}

// Turn on the spot, clockwise (left stick right).
void turnClockwise() {
  for (int i = 0; i < 8; i++) {
    setPose(TURN_CW[i]);
    if (!pressingCheck(D_RIGHT, stickFrameMs(D_RIGHT))) return;
  }
}

// Y button (hold): wave. L4 drops to 90 so the body tips back, then R3 extends to 0
// (full reach) and returns to 90, repeating while held. The first reach gets extra
// time because it travels from the stand pose (180) all the way to 0.
void waveFront() {
  static bool primed = false;   // true once R3 has reached max after Y was pressed
  L4.write(90);
  if (!primed) {
    R3.write(0);                                        // first time: go ALL the way to max
    if (!pressingCheck(D_WAVE, 700)) { primed = false; return; }
    primed = true;
  }
  R3.write(90);                                         // back to middle
  if (!pressingCheck(D_WAVE, 300)) { primed = false; return; }
  R3.write(0);                                          // extend to max
  if (!pressingCheck(D_WAVE, 300)) { primed = false; return; }
}


// B button (hold): dance. The body sways side to side, then bounces on the front and
// rear legs. Hips stay at mid-stroke; only the lifts move.
const int DANCE[6][8] = {
  { 45, 45, 45, 45, 135, 45,  0, 180 },   // right side up
  { 45, 45, 45, 45, 180,  0, 45, 135 },   // left side up
  { 45, 45, 45, 45, 135, 45,  0, 180 },
  { 45, 45, 45, 45, 180,  0, 45, 135 },
  { 45, 45, 45, 45, 135,  0, 45, 180 },   // both front legs up
  { 45, 45, 45, 45, 180, 45,  0, 135 }    // both rear legs up
};

void danceMove() {
  for (int i = 0; i < 6; i++) {
    setPose(DANCE[i]);
    if (!pressingCheck(D_DANCE, 250)) return;
  }
}

// D-pad UP (hold): push-ups. The front hips reach forward, then the front lifts dip to
// 90 (chest down) and rise back to home (chest up), repeating while held.
const int PUSHUP[2][8] = {
  { 90, 45,  0, 45, 180, 0,  0, 180 },    // up
  { 90, 45,  0, 45,  90, 0, 90, 180 }     // down
};

void pushupMove() {
  setPose(PUSHUP[0]);
  if (!pressingCheck(D_PUSHUP, 500)) return;
  setPose(PUSHUP[1]);
  if (!pressingCheck(D_PUSHUP, 600)) return;
}

// Bumpers (hold): crab walk. A 3-frame ratchet: the leading side lifts first, the sides
// swap, then all feet plant. Which side leads decides the direction. Hips stay mid-stroke.
const int CRAB_LEFT[3][8] = {
  { 45, 45, 45, 45, 180,  0, 45, 135 },   // left legs out
  { 45, 45, 45, 45, 135, 45,  0, 180 },   // swap: right legs out, left planted
  { 45, 45, 45, 45, 180,  0,  0, 180 }    // all planted (pull)
};
const int CRAB_RIGHT[3][8] = {
  { 45, 45, 45, 45, 135, 45,  0, 180 },   // right legs out
  { 45, 45, 45, 45, 180,  0, 45, 135 },   // swap: left legs out, right planted
  { 45, 45, 45, 45, 180,  0,  0, 180 }    // all planted (pull)
};
const int CRAB_MS = 280;   // ms per crab frame (scaled by the triggers)

// Plays one crab cycle from `tbl`. `d` is the action code that must stay held.
void crabMove(int d, const int (*tbl)[8]) {
  for (int i = 0; i < 3; i++) {
    setPose(tbl[i]);
    if (!pressingCheck(d, scaleMs(CRAB_MS))) return;
  }
}

// D-pad LEFT (hold): shake. Front hips and front lifts jiggle quickly.
const int SHAKE[2][8] = {
  { 60, 45, 30, 45, 150, 0, 30, 180 },
  { 30, 45, 60, 45, 180, 0,  0, 180 }
};

void shakeMove() {
  for (int i = 0; i < 2; i++) {
    setPose(SHAKE[i]);
    if (!pressingCheck(D_SHAKE, 130)) return;
  }
}

// D-pad RIGHT (hold): bow. The front dips halfway, then fully, and stays down while
// held; it stands up when released.
const int BOW_MID[8]  = { 45, 45, 45, 45, 135, 0, 45, 180 };
const int BOW_FULL[8] = { 45, 45, 45, 45,  90, 0, 90, 180 };

void bowMove() {
  setPose(BOW_MID);
  if (!pressingCheck(D_BOW, 300)) return;
  setPose(BOW_FULL);
  while (pressingCheck(D_BOW, 100)) { }
}

// ============================================================================
//  7. STATE (toggles and auto-sleep)
// ============================================================================
// Sit, sleep and play dead are toggles: each press flips the state. The *Prev flags
// remember the button's last state so a held button only counts once.
// Sit (toggle with A): front hips to home (R1 0, L1 90), rear lifts out by 90 (R4 90, L4 90)
bool sitting = false;
bool aPrev = false;   // A button state last pass

// Holds the sit pose (rear lowered, front up).
void sitPose() {
  R1.write(0);   L1.write(90);        // front hips to home
  R4.write(90);  L4.write(90);        // rear lifts extended by 90
  R2.write(45);  L2.write(45);        // rear hips mid
  R3.write(180); L3.write(0);         // front feet down
}

// Sleep (toggle with D-pad DOWN): all four lift servos extend out by 90 so the bot lies on its belly
bool sleeping = false;
bool dpadDownPrev = false;   // D-pad down state last pass

// Holds the sleep pose (all four lifts out, body on its belly).
void sleepPose() {
  R3.write(90);  L3.write(90);        // front lifts: 180 -> 90 (R3), 0 -> 90 (L3)
  R4.write(90);  L4.write(90);        // rear lifts:  0 -> 90 (R4), 180 -> 90 (L4)
  R1.write(45);  R2.write(45);        // hips mid
  L1.write(45);  L2.write(45);
}

// Play dead (toggle with X): all lifts out by 90 and every hip flops to home
bool dead = false;
bool xPrev = false;   // X button state last pass

// Holds the play-dead pose (like sleep, but every hip flops to home so the legs sprawl).
void deadPose() {
  R3.write(90);  L3.write(90);
  R4.write(90);  L4.write(90);
  R1.write(0);   R2.write(90);        // hips at home (your table)
  L1.write(90);  L2.write(0);
}

// Auto-sleep: no input for 60 s -> yawn at 56 s, lie down at 60 s, wake on any input.
// (Manual sleep from D-pad down is separate and only D-pad down wakes it.)
const unsigned long AUTO_YAWN_MS  = 56000;   // idle time before the yawn animation
const unsigned long AUTO_SLEEP_MS = 60000;   // idle time before lying down
unsigned long lastInput = 0;                 // when input was last seen (millis)
bool autoSlept = false;                      // true if the current sleep was automatic (any input wakes it)
bool yawnQueued = false;                     // the pre-sleep yawn has been triggered
bool sprintPrev = false;                     // sprint trigger state last pass (for the one-time buzz)

// ============================================================================
//  8. SETUP AND LOOP
// ============================================================================
// One-time startup: serial, servos, OLED, stand pose, then Bluetooth.
void setup() {
  Serial.begin(115200);

  // attach each servo to its pin and its own LEDC channel (0-7)
  R1.attach(D0, 0);
  R2.attach(D2, 1);
  L1.attach(D7, 2);
  L2.attach(D9, 3);
  R3.attach(D1, 4);
  R4.attach(D3, 5);
  L3.attach(D8, 6);
  L4.attach(D10, 7);

  // OLED on I2C: SDA = D4, SCL = D5, address 0x3C
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) { display.clearDisplay(); display.display(); }
  else Serial.println("OLED not found");

  standPose();              // start standing
  lastInput = millis();     // start the auto-sleep timer

  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("Waiting for controller...");
}

int lastDir = D_NONE;   // the action code from the previous pass (to detect a change)

// Main loop. Each pass runs these steps in order:
//   1. glide the servos          6. update sprint / sneak flags
//   2. right stick -> eyes       7. handle the toggle buttons (sit, sleep, play dead)
//   3. track input activity      8. hold sleep / dead / sit if active
//   4. sprint rumble             9. auto-sleep check
//   5. wake from auto-sleep     10. read the action, pick a face, run the movement
// The movement functions block while they play, but stay responsive through
// pressingCheck() / waitMs(), which keep servos, controller and face updating.
void loop() {
  BP32.update();

  // 1. Servos: choose the glide speed for the current state, then advance them
  servoSpeed = sleeping ? SPEED_SLEEP : dead ? SPEED_DEAD : sitting ? SPEED_SIT : SPEED_GAIT;
  servosUpdate();

  bool conn = myController && myController->isConnected();

  // 2. Right stick -> eye position (only visible on the idle and sit faces)
  int rx = conn ? myController->axisRX() : 0;
  int ry = conn ? myController->axisRY() : 0;
  eyeActive = (abs(rx) > 80 || abs(ry) > 80);
  eyeTX = eyeActive ? -constrain(rx, -512, 512) / 512.0 * 20.0 : 0;   // minus = flipped left/right
  eyeTY = eyeActive ? constrain(ry, -512, 512) / 512.0 * 14.0 : 0;

  // 3. Activity: any input at all resets the auto-sleep timer
  bool anyInput = conn && (myController->buttons() || myController->dpad()
                  || myController->throttle() > 40 || myController->brake() > 40
                  || abs(myController->axisX()) > 100 || abs(myController->axisY()) > 120 || eyeActive);
  if (anyInput) { lastInput = millis(); yawnQueued = false; forceIdleAnim = 0; }

  // 4. Rumble once when the sprint trigger is first pressed
  if (sprinting && !sprintPrev) buzz(80, 0x40, 0x20);
  sprintPrev = sprinting;

  // 5. Wake up from auto-sleep: any input stands the robot back up
  if (sleeping && autoSlept && anyInput) {
    sleeping = false; autoSlept = false;
    if (conn) { aPrev = myController->a(); xPrev = myController->x(); dpadDownPrev = (myController->dpad() & 0x02); }  // don't also trigger those buttons
    buzz(120, 0x40, 0x30);
    standPose();
    lastDir = D_NONE;
    waitMs(frameDelay);
  }

  // 6. Trigger flags for the face overlays (RT = sprint, LT = sneak)
  if (myController && myController->isConnected()) {
    sprinting = myController->throttle() > 300;
    sneaking  = myController->brake() > 300;
  } else {
    sprinting = false; sneaking = false;
  }

  // 7. Toggle buttons (each acts on the press, not while held)
  // A button toggles sit / stand
  if (myController && myController->isConnected()) {
    bool aNow = myController->a();
    if (aNow && !aPrev && !sleeping && !dead) {
      sitting = !sitting;
      if (!sitting) {              // standing back up
        standPose();
        lastDir = D_NONE;
        waitMs(frameDelay);
      }
    }
    aPrev = aNow;
  } else {
    aPrev = false;
  }

  // D-pad DOWN toggles sleep (acts on the press, not while held)
  if (myController && myController->isConnected()) {
    bool downNow = (myController->dpad() & 0x02);   // DPAD_DOWN
    if (downNow && !dpadDownPrev) {
      sleeping = !sleeping;
      autoSlept = false;
      sitting = false;
      dead = false;
      if (!sleeping) {             // waking up
        buzz(120, 0x40, 0x30);
        standPose();
        lastDir = D_NONE;
        waitMs(frameDelay);
      }
    }
    dpadDownPrev = downNow;
  } else {
    dpadDownPrev = false;
  }

  // X toggles play dead (acts on the press, not while held)
  if (myController && myController->isConnected()) {
    bool xNow = myController->x();
    if (xNow && !xPrev && !sleeping) {
      dead = !dead;
      sitting = false;
      if (!dead) {                 // getting back up
        standPose();
        lastDir = D_NONE;
        waitMs(frameDelay);
      }
    }
    xPrev = xNow;
  } else {
    xPrev = false;
  }

  // 8. Held states: while active, show their face, hold their pose and skip normal control
  if (dead) {                      // flop, show X eyes, ignore stick and buttons
    face = F_DEAD;
    updateFace();
    deadPose();
    delay(1);
    return;
  }

  if (sleeping) {                  // lie on belly, play the sleep face, ignore stick and Y
    face = F_SLEEP;
    updateFace();
    sleepPose();
    delay(1);
    return;
  }

  if (sitting) {                   // hold the sit pose, ignore stick and Y
    face = F_SIT;
    updateFace();
    sitPose();
    delay(1);
    return;
  }

  // 9. Auto-sleep: only from the standing idle (not while sitting / playing dead)
  if (!anyInput) {
    unsigned long idleFor = millis() - lastInput;
    if (idleFor > AUTO_YAWN_MS && !yawnQueued) { forceIdleAnim = 1; yawnQueued = true; }   // 1 = yawn
    if (idleFor > AUTO_SLEEP_MS) { sleeping = true; autoSlept = true; return; }
  }

  // 10. Normal control: read what is being asked for
  int dir = readDirection();

  // pick the face for it. (Turn and crab faces are mapped to the opposite side because
  // of how the display is mounted.) With no input: connection faces, else idle.
  if      (dir == D_FWD)   face = F_FWD;
  else if (dir == D_BACK)  face = F_BACK;
  else if (dir == D_LEFT)  face = F_RIGHT;
  else if (dir == D_RIGHT) face = F_LEFT;
  else if (dir == D_WAVE)  face = F_WAVE;
  else if (dir == D_DANCE) face = F_DANCE;
  else if (dir == D_PUSHUP) face = F_PUSHUP;
  else if (dir == D_CRABL) face = F_CRABR;
  else if (dir == D_CRABR) face = F_CRABL;
  else if (dir == D_SHAKE) face = F_SHAKE;
  else if (dir == D_BOW)   face = F_BOW;
  else if (!conn)          face = (connMode == 2 && millis() - connAt < 3000) ? F_SAD : F_SEARCH;
  else if (connMode == 1 && millis() - connAt < 2500) face = F_HELLO;
  else                     face = F_IDLE;

  // when the action changes (including returning to centre), pass through the stand pose
  if (dir != lastDir) {
    standPose();
    if (dir != D_NONE) waitMs(frameDelay);
    lastDir = dir;
  }

  // run one cycle of the matching movement
  if (dir == D_FWD) {
    walkForward();
  } else if (dir == D_BACK) {
    walkBackward();
  } else if (dir == D_LEFT) {
    turnCounterClockwise();
  } else if (dir == D_RIGHT) {
    turnClockwise();
  } else if (dir == D_WAVE) {
    waveFront();
  } else if (dir == D_DANCE) {
    danceMove();
  } else if (dir == D_PUSHUP) {
    pushupMove();
  } else if (dir == D_CRABL) {
    crabMove(D_CRABL, CRAB_LEFT);
  } else if (dir == D_CRABR) {
    crabMove(D_CRABR, CRAB_RIGHT);
  } else if (dir == D_SHAKE) {
    shakeMove();
  } else if (dir == D_BOW) {
    bowMove();
  } else {
    standPose();   // no input / stick in deadzone: hold the X stand formation
  }

  updateFace();
  delay(1);
}
