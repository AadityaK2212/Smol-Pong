// Pong on an 8x8 LED matrix, driven by an ATtiny1616.
// Two players, two buttons each (UP / DOWN), wired as an analog
// resistor ladder so each player only burns one MCU pin.
//
// Hardware notes (from the schematic):
//   - Rows  -> PA0..PA7  (Row1..Row8) -- these are the LED CATHODES
//   - Cols  -> PB0..PB5  (Col1..Col6)
//             PC0 -> Col7
//             PC1 -> Col8                -- columns are the LED ANODES
//   - PC2   -> P1 button ADC node
//   - PC3   -> P2 button ADC node
//
// To light pixel (row, col): drive that column HIGH, that row LOW,
// every other row Hi-Z (or HIGH), every other column LOW.
// We multiplex one column at a time at a few hundred Hz so the whole
// frame looks stable to the eye.
//
// Button decoding:
//   Idle (nothing pressed) the ADC pin floats up to ~VCC through the
//   internal pull-up. Pressing UP shorts it through 10k to GND, pressing
//   DOWN shorts it through 4.7k to GND. With the ATtiny's ~30-50k
//   internal pull-up that gives roughly:
//       idle  -> ~1023
//       UP    -> ~250-350
//       DOWN  -> ~120-180
//   We just pick two thresholds with comfortable margin.

#include <Arduino.h>

// ---------- pin map ----------
static const uint8_t ROW_PINS[8] = {
  PIN_PA0, PIN_PA1, PIN_PA2, PIN_PA3,
  PIN_PA4, PIN_PA5, PIN_PA6, PIN_PA7
};
static const uint8_t COL_PINS[8] = {
  PIN_PB0, PIN_PB1, PIN_PB2, PIN_PB3,
  PIN_PB4, PIN_PB5, PIN_PC0, PIN_PC1
};

static const uint8_t P1_ADC_PIN = PIN_PC2;
static const uint8_t P2_ADC_PIN = PIN_PC3;

// ---------- ADC thresholds for the resistor-ladder buttons ----------
// Anything below DOWN_MAX -> down, between -> up, above -> nothing.
// Tweak if your particular pull-up reads differently.
static const uint16_t DOWN_MAX = 220;   // 4.7k pull-down
static const uint16_t UP_MAX   = 500;   // 10k  pull-down
// (>UP_MAX is treated as "no press")

// ---------- framebuffer ----------
// frame[col] is a bitmask of which rows are lit in that column.
// Bit r set -> row r is lit when we light up that column.
static volatile uint8_t frame[8];

// ---------- game state ----------
static const uint8_t PADDLE_LEN = 3;     // pixels tall
static int8_t  p1_y = 3;                 // top row of P1 paddle (col 0)
static int8_t  p2_y = 3;                 // top row of P2 paddle (col 7)

// Ball, stored in 1/8-pixel units so we can have fractional speeds.
// A whole pixel is 8 sub-units.
static int16_t ball_x = 4 * 8;
static int16_t ball_y = 4 * 8;
static int16_t ball_vx = -2;             // sub-units per tick
static int16_t ball_vy =  1;

static uint8_t p1_score = 0;
static uint8_t p2_score = 0;
static const uint8_t WIN_SCORE = 5;

// ---------- timing ----------
static uint32_t last_tick_ms     = 0;
static uint32_t last_input_ms    = 0;
static uint32_t last_repeat_ms[2]= {0, 0};
static const uint16_t TICK_MS    = 90;   // game tick (ball moves about 11 Hz)
static const uint16_t INPUT_MS   = 25;   // poll buttons ~40 Hz
static const uint16_t REPEAT_MS  = 70;   // hold-to-move repeat rate

enum GameMode : uint8_t {
  MODE_PLAY,
  MODE_WIN_P1,
  MODE_WIN_P2,
  MODE_RESET_HOLD
};
static uint8_t mode = MODE_PLAY;
static uint32_t mode_started_ms = 0;

// ===================================================================
// LED MATRIX
// ===================================================================
//
// We park every pin as INPUT (Hi-Z) at start and only drive the pins
// for the column that's currently being shown. That keeps unused
// LEDs fully dark and avoids fighting the ADC pins.

static void matrix_init() {
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(ROW_PINS[i], INPUT);
    pinMode(COL_PINS[i], INPUT);
  }
  for (uint8_t i = 0; i < 8; i++) frame[i] = 0;
}

// Show a single column for a brief moment, then turn it back off.
static void show_column(uint8_t c) {
  uint8_t mask = frame[c];

  // Set every row that should be lit to OUTPUT-LOW (sink current),
  // others to INPUT (Hi-Z).
  for (uint8_t r = 0; r < 8; r++) {
    if (mask & (1 << r)) {
      pinMode(ROW_PINS[r], OUTPUT);
      digitalWrite(ROW_PINS[r], LOW);
    } else {
      pinMode(ROW_PINS[r], INPUT);
    }
  }

  // Drive the chosen column HIGH (source current).
  pinMode(COL_PINS[c], OUTPUT);
  digitalWrite(COL_PINS[c], HIGH);

  // Hold it on. Longer = brighter, but eats CPU time.
  delayMicroseconds(700);

  // Turn the column back off and release the rows.
  digitalWrite(COL_PINS[c], LOW);
  pinMode(COL_PINS[c], INPUT);
  for (uint8_t r = 0; r < 8; r++) {
    if (mask & (1 << r)) pinMode(ROW_PINS[r], INPUT);
  }
}

// Run one full pass over all 8 columns. Call this in a tight loop.
static void refresh_once() {
  for (uint8_t c = 0; c < 8; c++) show_column(c);
}

// Helpers used by the game logic to draw into the framebuffer.
static inline void fb_clear() {
  for (uint8_t c = 0; c < 8; c++) frame[c] = 0;
}
static inline void fb_set(int8_t x, int8_t y) {
  if (x < 0 || x > 7 || y < 0 || y > 7) return;
  frame[x] |= (uint8_t)(1 << y);
}

// ===================================================================
// BUTTONS
// ===================================================================

enum BtnState : uint8_t { BTN_NONE = 0, BTN_UP = 1, BTN_DOWN = 2 };

static uint8_t read_button(uint8_t pin) {
  uint16_t v = analogRead(pin);
  if (v < DOWN_MAX) return BTN_DOWN;
  if (v < UP_MAX)   return BTN_UP;
  return BTN_NONE;
}

static void buttons_init() {
  // Internal pull-up makes the resistor-ladder math work and gives us
  // a clean "nothing pressed -> full scale" reading.
  pinMode(P1_ADC_PIN, INPUT_PULLUP);
  pinMode(P2_ADC_PIN, INPUT_PULLUP);
  analogReadResolution(10);
}

// ===================================================================
// GAME
// ===================================================================

static void reset_ball(int8_t toward_player) {
  ball_x = 4 * 8;
  ball_y = (random(2, 6)) * 8;
  ball_vx = (toward_player == 1) ? -2 : 2;
  ball_vy = (random(0, 2) ? 1 : -1) * (random(1, 3));
}

static void new_game() {
  p1_score = 0;
  p2_score = 0;
  p1_y = 3;
  p2_y = 3;
  mode = MODE_PLAY;
  reset_ball(random(0, 2) ? 1 : 2);
}

static void clamp_paddle(int8_t &y) {
  if (y < 0) y = 0;
  if (y > 8 - PADDLE_LEN) y = 8 - PADDLE_LEN;
}

// Move a paddle in response to a button, with simple repeat-on-hold.
static void handle_player(uint8_t player_idx, uint8_t btn, uint32_t now) {
  if (btn == BTN_NONE) {
    last_repeat_ms[player_idx] = 0;
    return;
  }
  // First press fires immediately; subsequent fires throttled by REPEAT_MS.
  if (last_repeat_ms[player_idx] != 0 &&
      (now - last_repeat_ms[player_idx]) < REPEAT_MS) {
    return;
  }
  last_repeat_ms[player_idx] = now;

  int8_t &y = (player_idx == 0) ? p1_y : p2_y;
  if (btn == BTN_UP)   y--;
  if (btn == BTN_DOWN) y++;
  clamp_paddle(y);
}

static bool paddle_hits(int8_t paddle_y, int8_t bx, int8_t by) {
  // bx is the column adjacent to a paddle wall, by is integer ball row.
  if (bx != 0 && bx != 7) return false;
  return (by >= paddle_y) && (by < paddle_y + PADDLE_LEN);
}

static void physics_tick() {
  ball_x += ball_vx;
  ball_y += ball_vy;

  // Bounce off top/bottom.
  if (ball_y < 0)        { ball_y = 0;        ball_vy = -ball_vy; }
  if (ball_y > 7 * 8)    { ball_y = 7 * 8;    ball_vy = -ball_vy; }

  int8_t bx = ball_x / 8;
  int8_t by = ball_y / 8;

  // Left wall (P1).
  if (ball_x <= 0) {
    if (paddle_hits(p1_y, 0, by)) {
      ball_x = 0;
      ball_vx = -ball_vx;
      // Add a little english based on where it hit the paddle.
      int8_t hit = by - (p1_y + PADDLE_LEN / 2);
      ball_vy += hit;
      if (ball_vy >  3) ball_vy =  3;
      if (ball_vy < -3) ball_vy = -3;
    } else {
      // P2 scores.
      p2_score++;
      if (p2_score >= WIN_SCORE) {
        mode = MODE_WIN_P2;
        mode_started_ms = millis();
      } else {
        reset_ball(1);
      }
    }
  }

  // Right wall (P2).
  if (ball_x >= 7 * 8) {
    if (paddle_hits(p2_y, 7, by)) {
      ball_x = 7 * 8;
      ball_vx = -ball_vx;
      int8_t hit = by - (p2_y + PADDLE_LEN / 2);
      ball_vy += hit;
      if (ball_vy >  3) ball_vy =  3;
      if (ball_vy < -3) ball_vy = -3;
    } else {
      p1_score++;
      if (p1_score >= WIN_SCORE) {
        mode = MODE_WIN_P1;
        mode_started_ms = millis();
      } else {
        reset_ball(2);
      }
    }
  }
}

// ===================================================================
// RENDERING
// ===================================================================

static void render_play() {
  fb_clear();

  // Paddles (column 0 and column 7).
  for (uint8_t i = 0; i < PADDLE_LEN; i++) {
    fb_set(0, p1_y + i);
    fb_set(7, p2_y + i);
  }

  // Ball.
  fb_set(ball_x / 8, ball_y / 8);
}

// Cute little win animation: fill the winner's half of the board,
// then fade it back out, repeat.
static void render_win(uint8_t winner) {
  uint32_t t = millis() - mode_started_ms;
  uint8_t  phase = (t / 150) & 0x07;
  fb_clear();
  if (winner == 1) {
    for (uint8_t c = 0; c <= phase && c < 8; c++) frame[c] = 0xFF;
  } else {
    for (uint8_t c = 0; c < 8; c++) {
      if ((7 - c) <= phase) frame[c] = 0xFF;
    }
  }
  // After ~3 seconds, drop into "hold any button to restart" mode.
  if (t > 3000) mode = MODE_RESET_HOLD;
}

static void render_reset_prompt() {
  // Blink a single dot in the center -- "press anything to play again".
  fb_clear();
  if (((millis() / 400) & 1) == 0) {
    fb_set(3, 3);
    fb_set(4, 3);
    fb_set(3, 4);
    fb_set(4, 4);
  }
}

// ===================================================================
// MAIN
// ===================================================================

void setup() {
  matrix_init();
  buttons_init();
  // Seed the RNG from a floating analog read -- not crypto, just enough
  // to keep the ball from going the same way every power-on.
  randomSeed(analogRead(P1_ADC_PIN) ^ (analogRead(P2_ADC_PIN) << 4) ^ micros());
  new_game();
  last_tick_ms  = millis();
  last_input_ms = millis();
}

void loop() {
  uint32_t now = millis();

  // ---- input ----
  if ((uint32_t)(now - last_input_ms) >= INPUT_MS) {
    last_input_ms = now;
    uint8_t b1 = read_button(P1_ADC_PIN);
    uint8_t b2 = read_button(P2_ADC_PIN);

    if (mode == MODE_PLAY) {
      handle_player(0, b1, now);
      handle_player(1, b2, now);
    } else if (mode == MODE_RESET_HOLD) {
      if (b1 != BTN_NONE || b2 != BTN_NONE) new_game();
    }
  }

  // ---- physics ----
  if (mode == MODE_PLAY && (uint32_t)(now - last_tick_ms) >= TICK_MS) {
    last_tick_ms = now;
    physics_tick();
  }

  // ---- render ----
  switch (mode) {
    case MODE_PLAY:        render_play();          break;
    case MODE_WIN_P1:      render_win(1);          break;
    case MODE_WIN_P2:      render_win(2);          break;
    case MODE_RESET_HOLD:  render_reset_prompt();  break;
  }

  // One pass over the matrix. Call this as often as possible so the
  // display doesn't flicker.
  refresh_once();
}
