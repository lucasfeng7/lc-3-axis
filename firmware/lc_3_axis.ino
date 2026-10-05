/*
 * LC 3-Axis force sensor firmware
 *
 * Target: Teensy 4.0 with a TCA9548A I2C multiplexer and four NAU7802
 * 24-bit ADCs, one per 500 g load cell.
 *
 * The four load cells provide Fz and the bending moments used to infer Fx
 * and Fy. Calibration fits a single 3x4 linear map from tared cell counts
 * to (Fx, Fy, Fz), absorbing cell gain, polarity, placement, and coupling.
 *
 * Calibration samples are zero-bracketed to reduce thermal drift. Lateral
 * calibration assumes the load is applied at the calibrated tip height;
 * off-center normal loads also generate moment and cannot be distinguished
 * from lateral force by this geometry.
 *
 * Multiplexer channels are discovered at startup and stored with calibration.
 * Serial interface: 115200 baud, newline terminated. Send 'h' for help.
 */


#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NAU7802.h>

// Config
#define TCA_ADDR      0x70            // TCA9548A (A2:A0 = GND)
#define NAU_ADDR      0x2A            // NAU7802, fixed
#define NUM_CELLS     4
#define NPAR          5               // 4 count coefficients + 1 bias

#define I2C_HZ        400000          // within spec for both devices
#define ADC_RATE      NAU7802_RATE_320SPS
#define ADC_GAIN      NAU7802_GAIN_128

#define G0            9.80665f        // m/s^2
#define CELL_RATING_G 500.0f          // per cell; loads above the array rating are refused

#define WARMUP_DISCARD   8            // settling samples discarded after init
#define READ_TIMEOUT_MS  250
#define FRAME_TIMEOUT_MS 50           // emit a frame even if a cell stops updating
#define ACQ_TIMEOUT_MS   20000        // timeout for one averaged block

#define DEF_CAL_SAMPLES  512          // per cell, per read (512 @ 320 SPS = 1.6 s)
#define MIN_CAL_SAMPLES  64
#define MAX_CAL_SAMPLES  4096
#define NOISE_SAMPLES    512          // used at tare to measure the noise floor
#define STABLE_K         3.0          // reject a block noisier than K * tare noise
#define DRIFT_K          4.0          // reject a block trending > K sigma of its mean
#define ZERO_DRIFT_K     6.0          // warn if the bracket's two zeros disagree
#define ACQ_RETRIES      3
#define SETTLE_MS        1500         // wait for initial load-cell creep before sampling

#define MAX_SAMPLES   24              // bounded by EEPROM size, see EEPROM_BYTES
#define COUNT_SCALE   1e-6            // counts are ~1e6; scaled for conditioning
#define RIDGE_REL     1e-6            // ridge lambda, relative to mean diagonal

#define EEPROM_MAGIC  0x3A01          // change whenever the stored layout changes
#define EEPROM_ADDR   0

// State
Adafruit_NAU7802 nau;

uint8_t muxChan[NUM_CELLS];           // discovered at boot, stored with the calibration
uint8_t nFound    = 0;
int8_t  curChan   = -1;               // currently selected channel, to skip redundant writes

double  zeroCount[NUM_CELLS];         // live tare, raw counts
double  noiseSd[NUM_CELLS];           // unloaded standard deviation at tare, counts

int32_t rawCount[NUM_CELLS];          // newest streamed sample
bool    hasFresh[NUM_CELLS];

// Fx, Fy, Fz = M * counts + bias, in newtons.
// Defaults to the nominal square layout, so an uncalibrated board still
// streams output with the expected sign pattern.
float M[3][NUM_CELLS] = {
  { 1, -1, -1,  1},                   // Fx  (nominal)
  { 1,  1, -1, -1},                   // Fy  (nominal)
  { 1,  1,  1,  1},                   // Fz  = sum
};
float bias[3] = {0, 0, 0};
bool  fitted  = false;

// Types
// Keep these declarations above the first function definition because the
// Arduino build generates prototypes before compiling the sketch.

struct Sample {
  float   c[NUM_CELLS];               // zero-bracketed counts
  float   f[3];                       // applied force, newtons
  uint8_t axis;                       // 0=X 1=Y 2=Z, for reporting only
};
Sample  samples[MAX_SAMPLES];
uint8_t nSamples = 0;

// One averaged acquisition, with the statistics used by the stability check.
struct Block {
  double mean[NUM_CELLS];
  double sd[NUM_CELLS];
  double drift[NUM_CELLS];            // second-half mean minus first-half mean
};

struct FitReport {
  double rms[3];        // in-sample RMS residual, N
  double loo[3];        // leave-one-out RMS residual, N
  double worst[3];      // largest in-sample residual, N
  double ev[NUM_CELLS]; // eigenvalues of the scaled count Gram matrix, descending
  bool   ok;
};
FitReport lastFit;

// EEPROM footprint; the Teensy 4.0 provides 1080 bytes.
static const int EEPROM_BYTES = sizeof(uint16_t) + NUM_CELLS + 1 + 1
                              + NUM_CELLS * (int)sizeof(float)
                              + 3 * NUM_CELLS * (int)sizeof(float)
                              + 3 * (int)sizeof(float)
                              + MAX_SAMPLES * (int)sizeof(Sample);
static_assert(EEPROM_BYTES <= 1080, "calibration will not fit in Teensy 4.0 EEPROM");

bool     streaming  = false;
bool     showCells  = false;
bool     showStamp  = false;
bool     useNewton  = true;           // false = gram-force
float    emaAlpha   = 1.0f;           // 1 = unfiltered
uint16_t calSamples = DEF_CAL_SAMPLES;

uint32_t lastFrameMs = 0;

// Utilities
// Float formatter. %f is not reliable with newlib-nano on Teensy, and
// formatting into a buffer lets each line go out in a single Serial.write.
int fmtF(char *b, double v, int dec) {
  int i = 0;
  if (isnan(v))       { b[0] = 'n'; b[1] = 'a'; b[2] = 'n'; return 3; }
  if (fabs(v) > 1e9)  { b[0] = v < 0 ? '-' : '+'; b[1] = b[2] = b[3] = '9'; return 4; }
  if (v < 0) { b[i++] = '-'; v = -v; }
  double round = 0.5;
  for (int k = 0; k < dec; k++) round *= 0.1;
  v += round;
  uint32_t ip = (uint32_t)v;
  double   fp = v - ip;
  char t[12];
  int  n = 0;
  if (!ip) t[n++] = '0';
  while (ip) { t[n++] = '0' + ip % 10; ip /= 10; }
  while (n)  b[i++] = t[--n];
  if (dec > 0) {
    b[i++] = '.';
    for (int k = 0; k < dec; k++) {
      fp *= 10;
      int d = (int)fp;
      if (d > 9) d = 9;
      b[i++] = '0' + d;
      fp -= d;
    }
  }
  return i;
}

// Right-aligned float in a fixed-width column.
void pf(double v, int width, int dec) {
  char b[32];
  int n = fmtF(b, v, dec);
  for (int k = n; k < width; k++) Serial.print(' ');
  Serial.write(b, n);
}

float toDisplay(float newtons) { return useNewton ? newtons : newtons * 1000.0f / G0; }
const __FlashStringHelper *unitName() { return useNewton ? F("N") : F("gf"); }

// Mux
bool tcaSelect(uint8_t channel) {
  if (curChan == (int8_t)channel) return true;   // already selected
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) { curChan = -1; return false; }
  curChan = channel;
  return true;
}

// ADC
bool initNAU() {
  if (!nau.begin(&Wire)) return false;
  nau.setLDO(NAU7802_3V0);
  nau.setGain(ADC_GAIN);
  nau.setRate(ADC_RATE);
  // Gain and rate changes invalidate the internal calibration, so redo it.
  nau.calibrate(NAU7802_CALMOD_INTERNAL);
  nau.calibrate(NAU7802_CALMOD_OFFSET);
  for (uint8_t i = 0; i < WARMUP_DISCARD; i++) {
    uint32_t t0 = millis();
    while (!nau.available()) if (millis() - t0 > READ_TIMEOUT_MS) return true;
    nau.read();
  }
  return true;
}

// Discover NAU7802 devices in mux-channel order. Store the channel map with
// calibration so missing hardware cannot silently change cell indexing.
bool discoverCells(bool verbose) {
  curChan = -1;
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.print(F("# TCA9548A NOT found at 0x")); Serial.println(TCA_ADDR, HEX);
    Serial.println(F("# check SDA/SCL (Teensy 4.0: 18/19), 3V3, GND, pull-ups, A0-A2"));
    nFound = 0;
    return false;
  }

  nFound = 0;
  for (uint8_t ch = 0; ch < 8 && nFound < NUM_CELLS; ch++) {
    if (!tcaSelect(ch)) continue;
    delay(2);
    Wire.beginTransmission(NAU_ADDR);
    if (Wire.endTransmission() != 0) continue;
    if (!initNAU()) continue;
    muxChan[nFound] = ch;
    zeroCount[nFound] = 0;
    noiseSd[nFound]   = 0;
    rawCount[nFound]  = 0;
    nFound++;
  }

  if (verbose) {
    Serial.print(F("# found ")); Serial.print(nFound);
    Serial.print(F(" of ")); Serial.print(NUM_CELLS); Serial.print(F(" cells on mux ch"));
    for (uint8_t c = 0; c < nFound; c++) { Serial.print(' '); Serial.print(muxChan[c]); }
    Serial.println();
    if (nFound < NUM_CELLS)
      Serial.println(F("# ! all four cells are required for 3-axis output - 'i' to rescan"));
  }
  for (uint8_t c = 0; c < NUM_CELLS; c++) hasFresh[c] = (c >= nFound);
  lastFrameMs = millis();
  return nFound == NUM_CELLS;
}

// Acquisition
// Acquire an averaged block using round-robin polling so all cells cover the
// same time window. Also compute per-cell noise and intra-block drift.
bool acquire(Block &blk, uint16_t want) {
  if (nFound == 0) return false;
  double   sum[NUM_CELLS] = {0}, sum2[NUM_CELLS] = {0}, firstSum[NUM_CELLS] = {0};
  double   ref[NUM_CELLS] = {0};
  uint16_t n[NUM_CELLS]   = {0};
  const uint16_t half = want / 2;
  uint32_t t0 = millis();

  for (;;) {
    bool done = true;
    for (uint8_t c = 0; c < nFound; c++) {
      if (n[c] >= want) continue;
      done = false;
      if (!tcaSelect(muxChan[c]) || !nau.available()) continue;
      double v = (double)nau.read();
      if (n[c] == 0) ref[c] = v;             // offset by the first reading before
      double d = v - ref[c];                 // squaring to preserve precision in sum2
      sum[c]  += d;
      sum2[c] += d * d;
      if (n[c] < half) firstSum[c] += d;
      n[c]++;
    }
    if (done) break;
    if (millis() - t0 > ACQ_TIMEOUT_MS) return false;
  }

  for (uint8_t c = 0; c < nFound; c++) {
    double m  = sum[c] / n[c];
    double var = sum2[c] / n[c] - m * m;
    blk.mean[c] = ref[c] + m;
    blk.sd[c]   = var > 0 ? sqrt(var) : 0;
    double f = firstSum[c] / half;
    double s = (sum[c] - firstSum[c]) / (n[c] - half);
    blk.drift[c] = s - f;
  }
  return true;
}

// Retry averaged reads while the signal is settling. Stability thresholds are
// relative to the noise floor measured during tare.
bool acquireStable(Block &blk, const __FlashStringHelper *what) {
  delay(SETTLE_MS);                    // allow load-cell creep to settle
  for (uint8_t attempt = 1; attempt <= ACQ_RETRIES; attempt++) {
    if (!acquire(blk, calSamples)) {
      Serial.println(F("# ! a cell stopped responding - 'i' to rescan"));
      return false;
    }
    int8_t noisy = -1, moving = -1;
    for (uint8_t c = 0; c < nFound; c++) {
      double floorSd = noiseSd[c] > 1 ? noiseSd[c] : 1;
      // K times the standard error of the difference between half-block means.
      double driftLim = DRIFT_K * blk.sd[c] * sqrt(2.0 / (calSamples / 2));
      if (driftLim < 0.5) driftLim = 0.5;
      if (blk.sd[c] > STABLE_K * floorSd)          noisy  = c;
      if (fabs(blk.drift[c]) > driftLim)           moving = c;
    }
    if (noisy < 0 && moving < 0) return true;

    Serial.print(F("# still settling ("));
    Serial.print(what);
    if (noisy >= 0) {
      Serial.print(F(", cell ")); Serial.print(noisy);
      Serial.print(F(" noise ")); pf(blk.sd[noisy], 0, 0);
      Serial.print(F(" vs ")); pf(STABLE_K * (noiseSd[noisy] > 1 ? noiseSd[noisy] : 1), 0, 0);
      Serial.print(F(" counts"));
    }
    if (moving >= 0) {
      Serial.print(F(", cell ")); Serial.print(moving);
      Serial.print(F(" drifting ")); pf(blk.drift[moving], 0, 0);
      Serial.print(F(" counts"));
    }
    Serial.print(F(") - retry ")); Serial.print(attempt);
    Serial.print('/'); Serial.println(ACQ_RETRIES);
  }
  Serial.println(F("# ! never settled - taking it anyway, treat this point with suspicion"));
  return true;
}

// Streaming
void pollCells() {
  for (uint8_t c = 0; c < nFound; c++) {
    if (!tcaSelect(muxChan[c])) continue;
    if (nau.available()) { rawCount[c] = nau.read(); hasFresh[c] = true; }
  }
}

bool frameReady() {
  for (uint8_t c = 0; c < NUM_CELLS; c++) if (!hasFresh[c]) return false;
  return true;
}

void applyMatrix(const float c[NUM_CELLS], float f[3]) {
  for (uint8_t r = 0; r < 3; r++) {
    double a = bias[r];
    for (uint8_t i = 0; i < NUM_CELLS; i++) a += (double)M[r][i] * c[i];
    f[r] = (float)a;
  }
}

// Apply the matrix and the EMA filter, then send one CSV frame.
void emitFrame() {
  static float filt[3] = {0, 0, 0};
  static bool  primed  = false;

  float c[NUM_CELLS], f[3];
  for (uint8_t i = 0; i < NUM_CELLS; i++) c[i] = (float)(rawCount[i] - zeroCount[i]);
  applyMatrix(c, f);

  if (!primed) { for (uint8_t r = 0; r < 3; r++) filt[r] = f[r]; primed = true; }
  else for (uint8_t r = 0; r < 3; r++) filt[r] += emaAlpha * (f[r] - filt[r]);

  char buf[160];
  int  n = 0;
  if (showStamp) n += snprintf(buf + n, sizeof(buf) - n, "%lu,", (unsigned long)micros());
  for (uint8_t r = 0; r < 3; r++) {
    n += fmtF(buf + n, toDisplay(filt[r]), 4);
    buf[n++] = (r < 2 || showCells) ? ',' : '\n';
  }
  if (showCells)
    for (uint8_t i = 0; i < NUM_CELLS; i++)
      n += snprintf(buf + n, sizeof(buf) - n, "%ld%c", (long)(rawCount[i] - (long)zeroCount[i]),
                    i < NUM_CELLS - 1 ? ',' : '\n');
  Serial.write(buf, n);

  for (uint8_t c2 = 0; c2 < NUM_CELLS; c2++) hasFresh[c2] = (c2 >= nFound);
  lastFrameMs = millis();
}

// Tare
// Tare all cells and record each cell's noise floor, which the stability
// checks are scaled against.
void tareAll() {
  Serial.println(F("# taring - keep the sensor UNLOADED and still"));
  Block b;
  if (!acquire(b, NOISE_SAMPLES)) { Serial.println(F("# ! tare failed")); return; }
  for (uint8_t c = 0; c < nFound; c++) { zeroCount[c] = b.mean[c]; noiseSd[c] = b.sd[c]; }

  Serial.println(F("# cell      zero     noise sd"));
  for (uint8_t c = 0; c < nFound; c++) {
    Serial.print(F("#    ")); Serial.print(c);
    pf(zeroCount[c], 11, 0);
    pf(noiseSd[c], 11, 1);
    Serial.println();
  }
  if (fitted) {
    // Propagate independent cell noise through each force row in quadrature.
    const char *nm[3] = {"Fx ", "  Fy ", "  Fz "};
    Serial.print(F("# 1-sigma single-sample resolution ~ "));
    for (uint8_t r = 0; r < 3; r++) {
      double q = 0;
      for (uint8_t i = 0; i < nFound; i++) {
        const double t = (double)M[r][i] * noiseSd[i];
        q += t * t;
      }
      Serial.print(nm[r]);
      pf(toDisplay((float)sqrt(q)), 0, 4);
    }
    Serial.print(' '); Serial.println(unitName());
  }
  for (uint8_t c = 0; c < NUM_CELLS; c++) hasFresh[c] = (c >= nFound);
  lastFrameMs = millis();
}

// Unloaded drift test. Reports each cell's drift rate in counts/s, so a cell
// that drifts on its own can be identified before calibration rather than
// through rejected samples.
void driftTest(uint16_t seconds) {
  if (nFound == 0) { Serial.println(F("# ! no cells")); return; }

  Serial.print(F("# --- drift test, ")); Serial.print(seconds);
  Serial.println(F(" s. UNLOADED, hands off, any key stops. ---"));
  Block b;
  if (!acquire(b, 320)) { Serial.println(F("# ! read failed")); return; }

  double first[NUM_CELLS], last[NUM_CELLS], lo[NUM_CELLS], hi[NUM_CELLS];
  for (uint8_t c = 0; c < nFound; c++)
    first[c] = last[c] = lo[c] = hi[c] = b.mean[c];

  const uint32_t t0 = millis();
  Serial.println(F("#    t/s        c0        c1        c2        c3   (change from start)"));
  while ((millis() - t0) / 1000 < seconds) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    if (!acquire(b, 320)) break;                 // ~1 s per row
    Serial.print(F("# ")); pf((millis() - t0) / 1000.0, 6, 0);
    for (uint8_t c = 0; c < nFound; c++) {
      last[c] = b.mean[c];
      if (b.mean[c] < lo[c]) lo[c] = b.mean[c];
      if (b.mean[c] > hi[c]) hi[c] = b.mean[c];
      pf(last[c] - first[c], 10, 0);
    }
    Serial.println();
  }

  const double el = (millis() - t0) / 1000.0;
  if (el < 1) return;
  Serial.println(F("# cell     total      rate      span   verdict"));
  double worstRate = 0;
  int8_t worstCell = -1;
  for (uint8_t c = 0; c < nFound; c++) {
    const double rate = (last[c] - first[c]) / el;
    Serial.print(F("#    ")); Serial.print(c);
    pf(last[c] - first[c], 10, 0);
    pf(rate, 10, 1);
    pf(hi[c] - lo[c], 10, 0);
    // Flag a cell whose net drift over the test exceeds 3x its noise floor.
    Serial.println(fabs(rate) * el > 3 * (noiseSd[c] > 1 ? noiseSd[c] : 1)
                   ? F("   DRIFTING") : F("   ok"));
    if (fabs(rate) > fabs(worstRate)) { worstRate = rate; worstCell = c; }
  }
  Serial.println(F("#   total/span in counts, rate in counts/s"));
  if (worstCell >= 0 && fabs(worstRate) * el > 3 * (noiseSd[worstCell] > 1 ? noiseSd[worstCell] : 1)) {
    Serial.print(F("#   ! cell ")); Serial.print(worstCell);
    Serial.println(F(" is the worst. Swap its load cell onto another NAU7802:"));
    Serial.println(F("#     drift follows the CELL = cell/wiring, stays on the BOARD = ADC."));
  }
}

// Linear algebra
// Gauss-Jordan inverse of a small symmetric positive-definite matrix. The
// full inverse, not just a solve, is needed for the leave-one-out residuals.
bool invertN(double A[NPAR][NPAR], double inv[NPAR][NPAR]) {
  double a[NPAR][2 * NPAR];
  for (uint8_t i = 0; i < NPAR; i++) {
    for (uint8_t j = 0; j < NPAR; j++) { a[i][j] = A[i][j]; a[i][NPAR + j] = (i == j); }
  }
  for (uint8_t col = 0; col < NPAR; col++) {
    uint8_t piv = col;
    for (uint8_t r = col + 1; r < NPAR; r++)
      if (fabs(a[r][col]) > fabs(a[piv][col])) piv = r;
    if (fabs(a[piv][col]) < 1e-18) return false;
    if (piv != col) for (uint8_t k = 0; k < 2 * NPAR; k++) {
      double s = a[col][k]; a[col][k] = a[piv][k]; a[piv][k] = s;
    }
    double d = a[col][col];
    for (uint8_t k = 0; k < 2 * NPAR; k++) a[col][k] /= d;
    for (uint8_t r = 0; r < NPAR; r++) {
      if (r == col) continue;
      double fac = a[r][col];
      if (fac == 0) continue;
      for (uint8_t k = 0; k < 2 * NPAR; k++) a[r][k] -= fac * a[col][k];
    }
  }
  for (uint8_t i = 0; i < NPAR; i++)
    for (uint8_t j = 0; j < NPAR; j++) inv[i][j] = a[i][NPAR + j];
  return true;
}

// Cyclic Jacobi eigenvalues of a 4x4 symmetric matrix, sorted descending.
// Used only to report how well the sample set spans the measurable space.
void eigenvalues4(const double Ain[NUM_CELLS][NUM_CELLS], double ev[NUM_CELLS]) {
  double a[NUM_CELLS][NUM_CELLS];
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    for (uint8_t j = 0; j < NUM_CELLS; j++) a[i][j] = Ain[i][j];

  for (uint8_t sweep = 0; sweep < 30; sweep++) {
    double off = 0;
    for (uint8_t p = 0; p < NUM_CELLS; p++)
      for (uint8_t q = p + 1; q < NUM_CELLS; q++) off += a[p][q] * a[p][q];
    if (off < 1e-30) break;
    for (uint8_t p = 0; p < NUM_CELLS; p++) {
      for (uint8_t q = p + 1; q < NUM_CELLS; q++) {
        if (fabs(a[p][q]) < 1e-300) continue;
        double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1));
        double c = 1 / sqrt(t * t + 1), s = t * c;
        for (uint8_t k = 0; k < NUM_CELLS; k++) {
          double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (uint8_t k = 0; k < NUM_CELLS; k++) {
          double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
      }
    }
  }
  for (uint8_t i = 0; i < NUM_CELLS; i++) ev[i] = a[i][i];
  for (uint8_t i = 0; i < NUM_CELLS; i++)          // sort descending
    for (uint8_t j = i + 1; j < NUM_CELLS; j++)
      if (ev[j] > ev[i]) { double s = ev[i]; ev[i] = ev[j]; ev[j] = s; }
}

// Fitting
// Ridge least squares of each force row on [c0 c1 c2 c3 1]. Only the four
// count columns are regularized, to suppress the unexcited twist mode; the
// bias column is not penalized.
bool fitMatrix(FitReport &rep, bool verbose) {
  rep.ok = false;
  if (nSamples < NPAR) {
    if (verbose) {
      Serial.print(F("# need >= ")); Serial.print(NPAR);
      Serial.print(F(" samples, have ")); Serial.println(nSamples);
    }
    return false;
  }

  // Require at least one sample per axis. Lateral pulls excite only Mx and
  // My, so without Z samples Fz is unconstrained. All three rows share one
  // inverse, so the resulting error would reach every axis; refuse to fit.
  {
    uint8_t n[3]; bool pos[3], neg[3];
    axisCensus(n, pos, neg);
    if (n[0] == 0 || n[1] == 0 || n[2] == 0) {
      if (verbose) {
        Serial.print(F("# ! cannot fit: X=")); Serial.print(n[0]);
        Serial.print(F(" Y=")); Serial.print(n[1]);
        Serial.print(F(" Z=")); Serial.print(n[2]);
        Serial.println(F(" - every axis needs at least one sample."));
        if (n[2] == 0)
          Serial.println(F("#   No Z samples: stack a mass on the tip CENTRE, e.g. 'z 200'."));
      }
      return false;
    }
  }

  // Design matrix rows, scaled for conditioning.
  double X[MAX_SAMPLES][NPAR];
  for (uint8_t s = 0; s < nSamples; s++) {
    for (uint8_t i = 0; i < NUM_CELLS; i++) X[s][i] = samples[s].c[i] * COUNT_SCALE;
    X[s][NUM_CELLS] = 1.0;
  }

  double XtX[NPAR][NPAR] = {{0}};
  for (uint8_t s = 0; s < nSamples; s++)
    for (uint8_t i = 0; i < NPAR; i++)
      for (uint8_t j = 0; j < NPAR; j++) XtX[i][j] += X[s][i] * X[s][j];

  double gram[NUM_CELLS][NUM_CELLS];
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    for (uint8_t j = 0; j < NUM_CELLS; j++) gram[i][j] = XtX[i][j];
  eigenvalues4(gram, rep.ev);

  double trace = 0;
  for (uint8_t i = 0; i < NUM_CELLS; i++) trace += XtX[i][i];
  double lambda = RIDGE_REL * trace / NUM_CELLS;
  if (!(lambda > 0)) { if (verbose) Serial.println(F("# no signal in the samples")); return false; }

  double A[NPAR][NPAR], Ainv[NPAR][NPAR];
  for (uint8_t i = 0; i < NPAR; i++)
    for (uint8_t j = 0; j < NPAR; j++)
      A[i][j] = XtX[i][j] + ((i == j && i < NUM_CELLS) ? lambda : 0.0);
  if (!invertN(A, Ainv)) { if (verbose) Serial.println(F("# fit failed (singular)")); return false; }

  // Leverage h_ii = x_i' Ainv x_i, shared by all three rows.
  double lev[MAX_SAMPLES];
  for (uint8_t s = 0; s < nSamples; s++) {
    double h = 0;
    for (uint8_t i = 0; i < NPAR; i++) {
      double t = 0;
      for (uint8_t j = 0; j < NPAR; j++) t += Ainv[i][j] * X[s][j];
      h += X[s][i] * t;
    }
    lev[s] = h;
  }

  // Fit into temporaries so a degenerate sample set cannot overwrite a
  // previously valid matrix.
  float Mnew[3][NUM_CELLS], bNew[3];

  double maxApplied = 0;
  for (uint8_t s = 0; s < nSamples; s++)
    for (uint8_t r = 0; r < 3; r++)
      if (fabs(samples[s].f[r]) > maxApplied) maxApplied = fabs(samples[s].f[r]);

  for (uint8_t r = 0; r < 3; r++) {
    double Xty[NPAR] = {0}, beta[NPAR] = {0};
    for (uint8_t s = 0; s < nSamples; s++)
      for (uint8_t i = 0; i < NPAR; i++) Xty[i] += X[s][i] * samples[s].f[r];
    for (uint8_t i = 0; i < NPAR; i++)
      for (uint8_t j = 0; j < NPAR; j++) beta[i] += Ainv[i][j] * Xty[j];

    double sse = 0, sseLoo = 0, worst = 0;
    for (uint8_t s = 0; s < nSamples; s++) {
      double pred = 0;
      for (uint8_t i = 0; i < NPAR; i++) pred += X[s][i] * beta[i];
      double e = pred - samples[s].f[r];
      sse += e * e;
      if (fabs(e) > worst) worst = fabs(e);
      double den = 1.0 - lev[s];
      double eL  = den > 1e-6 ? e / den : e;      // leverage near 1: use the raw residual
      sseLoo += eL * eL;
    }
    rep.rms[r]   = sqrt(sse / nSamples);
    rep.loo[r]   = sqrt(sseLoo / nSamples);
    rep.worst[r] = worst;

    for (uint8_t i = 0; i < NUM_CELLS; i++) Mnew[r][i] = (float)(beta[i] * COUNT_SCALE);
    bNew[r] = (float)beta[NUM_CELLS];
  }

  // Reject a fit that cannot reproduce its own training data. This catches
  // degenerate sample sets without a rig-specific threshold.
  for (uint8_t r = 0; r < 3; r++) {
    if (!isfinite(rep.rms[r]) || rep.rms[r] > 2 * maxApplied + 0.5) {
      if (verbose) {
        Serial.print(F("# ! fit rejected: F")); Serial.print("xyz"[r]);
        Serial.print(F(" residual ")); pf(rep.rms[r], 0, 1);
        Serial.print(F(" N against loads of at most ")); pf(maxApplied, 0, 2);
        Serial.println(F(" N."));
        Serial.println(F("#   The sample set does not constrain all three axes. Matrix kept."));
      }
      return false;
    }
    for (uint8_t i = 0; i < NUM_CELLS; i++)
      if (!isfinite(Mnew[r][i])) { if (verbose) Serial.println(F("# ! fit produced NaN")); return false; }
  }

  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t i = 0; i < NUM_CELLS; i++) M[r][i] = Mnew[r][i];
    bias[r] = bNew[r];
  }
  fitted = true;
  rep.ok = true;
  return true;
}

// Count samples per axis and record which load signs are present.
void axisCensus(uint8_t n[3], bool pos[3], bool neg[3]) {
  for (uint8_t a = 0; a < 3; a++) { n[a] = 0; pos[a] = neg[a] = false; }
  for (uint8_t s = 0; s < nSamples; s++) {
    uint8_t a = samples[s].axis;
    if (a > 2) continue;
    n[a]++;
    if (samples[s].f[a] > 0) pos[a] = true;
    if (samples[s].f[a] < 0) neg[a] = true;
  }
}

// Crosstalk table: for each load axis, the mean response on every output axis
// as a percentage of the applied load. Only single-axis samples are used.
void reportCrosstalk() {
  const char *nm = "XYZ";
  Serial.println(F("# crosstalk (mean off-axis reading, % of applied load)"));
  Serial.println(F("#  load      -> Fx %      Fy %      Fz %"));
  for (uint8_t a = 0; a < 3; a++) {
    double acc[3] = {0, 0, 0};
    double appl = 0;
    uint8_t n = 0;
    for (uint8_t s = 0; s < nSamples; s++) {
      if (samples[s].axis != a) continue;
      if (!isPureLoad(samples[s])) continue;   // mixed loads have real off-axis content
      float f[3];
      applyMatrix(samples[s].c, f);
      for (uint8_t r = 0; r < 3; r++) acc[r] += fabs(f[r]);
      appl += fabs(samples[s].f[a]);
      n++;
    }
    if (!n || appl <= 0) continue;
    Serial.print(F("#  ")); Serial.print(nm[a]); Serial.print(F("  (n="));
    Serial.print(n); Serial.print(F(")"));
    for (uint8_t r = 0; r < 3; r++) pf(100.0 * acc[r] / appl, 10, 1);
    Serial.println();
  }
  Serial.println(F("#  on-axis should read ~100, the other two ~0"));
}

void reportFit() {
  if (!lastFit.ok) { Serial.println(F("# not fitted")); return; }
  const char *nm[3] = {"Fx", "Fy", "Fz"};

  Serial.println(F("# axis   in-sample RMS   leave-one-out    worst"));
  for (uint8_t r = 0; r < 3; r++) {
    Serial.print(F("#   ")); Serial.print(nm[r]);
    pf(toDisplay(lastFit.rms[r]), 16, 4);
    pf(toDisplay(lastFit.loo[r]), 16, 4);
    pf(toDisplay(lastFit.worst[r]), 9, 4);
    Serial.println();
  }
  Serial.print(F("#   units "));  Serial.println(unitName());
  Serial.println(F("#   leave-one-out is the number to quote: in-sample is"));
  Serial.println(F("#   optimistic because the fit has 5 free parameters."));

  // Mode excitation: expect three strong modes and one near zero.
  double top = lastFit.ev[0] > 0 ? lastFit.ev[0] : 1;
  Serial.print(F("# mode excitation (normalized):"));
  for (uint8_t i = 0; i < NUM_CELLS; i++) { Serial.print(' '); pf(lastFit.ev[i] / top, 0, 5); }
  Serial.println();
  if (lastFit.ev[2] / top < 1e-3) {
    Serial.println(F("#   ! only 2 modes excited - one axis is missing or too weak"));
  } else if (lastFit.ev[3] / top > 0.02) {
    // On a rigid rig the twist mode stays near 1e-5. A percent-level value
    // means the structure seats differently between samples, and the fit
    // will partly depend on it.
    Serial.println(F("#   3 strong modes: good. But the 4th is the TWIST mode, which no"));
    Serial.println(F("#   external load can excite - on a rigid rig it reads ~0.00001."));
    Serial.println(F("#   Percent-sized means the structure seats differently between"));
    Serial.println(F("#   samples (preload, warping). Lean on the verify step, not the fit."));
  } else {
    Serial.println(F("#   3 strong + 1 near-zero is correct: the 4th is the unloadable twist mode"));
  }

  uint8_t n[3]; bool pos[3], neg[3];
  axisCensus(n, pos, neg);
  const char *an = "XYZ";
  for (uint8_t a = 0; a < 3; a++) {
    if (n[a] == 0) {
      Serial.print(F("#   ! no ")); Serial.print(an[a]); Serial.println(F(" samples at all"));
    } else if (a < 2 && !(pos[a] && neg[a])) {
      Serial.print(F("#   ! ")); Serial.print(an[a]);
      Serial.println(F(" pulled one way only - asymmetry is hiding in the slope"));
    }
  }
  reportCrosstalk();
}

// Sample capture
// Blocking prompt. Returns false if the operator aborts with 'q'.
bool prompt(const __FlashStringHelper *msg) {
  Serial.print(F("# ")); Serial.print(msg); Serial.println(F("  [ENTER, or q to abort]"));
  while (Serial.available()) Serial.read();
  for (;;) {
    while (!Serial.available()) {}
    char ch = Serial.read();
    if (ch == 'q' || ch == 'Q') {
      while (Serial.available()) Serial.read();
      Serial.println(F("# aborted"));
      return false;
    }
    if (ch == '\n' || ch == '\r') {
      while (Serial.available()) Serial.read();
      return true;
    }
  }
}

// Capture zero-load-zero and subtract the mean zero to suppress linear drift.
bool captureBracketed(const __FlashStringHelper *loadMsg, float out[NUM_CELLS]) {
  Block z0, load, z1;

  // Noise statistics are not persisted, so calibration requires a fresh tare.
  if (noiseSd[0] == 0) {
    Serial.println(F("# ! tare first ('t') - the stability check needs the noise floor"));
    return false;
  }

  if (!prompt(F("remove ALL load"))) return false;
  if (!acquireStable(z0, F("zero"))) return false;

  if (!prompt(loadMsg)) return false;
  if (!acquireStable(load, F("load"))) return false;

  if (!prompt(F("remove the load again"))) return false;
  if (!acquireStable(z1, F("zero"))) return false;

  double worstDrift = 0;
  int8_t worstCell = 0;
  for (uint8_t c = 0; c < nFound; c++) {
    out[c] = (float)(load.mean[c] - 0.5 * (z0.mean[c] + z1.mean[c]));
    double d = fabs(z1.mean[c] - z0.mean[c]);
    if (d > worstDrift) { worstDrift = d; worstCell = c; }
  }
  for (uint8_t c = nFound; c < NUM_CELLS; c++) out[c] = 0;

  // Find the cell with the largest response to the load.
  uint8_t k = 0;
  double  peak = 0;
  for (uint8_t c = 0; c < nFound; c++) {
    double sig = fabs(load.mean[c] - z0.mean[c]);
    if (sig > peak) { peak = sig; k = c; }
  }
  const double se = (noiseSd[k] > 1 ? noiseSd[k] : 1) / sqrt((double)calSamples);

  // Reject captures with no measurable applied load.
  if (peak < 20 * se) {
    Serial.print(F("# ! no load detected - peak was only ")); pf(peak, 0, 0);
    Serial.println(F(" counts. Nothing applied? Sample discarded."));
    return false;
  }
  // Reject captures where the closing zero indicates the load was not removed.
  if (fabs(z1.mean[k] - load.mean[k]) < 0.25 * peak) {
    Serial.println(F("# ! the closing zero still reads LOADED - the weight was left on."));
    Serial.println(F("#   Tell-tale: zero drift comes out at 2x the counts. That would"));
    Serial.println(F("#   bake a 2x gain error into the matrix. Sample discarded, redo it."));
    return false;
  }

  // Report zero shift relative to signal magnitude as a drift diagnostic.
  Serial.print(F("# zero drift over the bracket: ")); pf(worstDrift, 0, 0);
  Serial.print(F(" counts on cell ")); Serial.print(worstCell);
  Serial.print(F(" = ")); pf(100.0 * worstDrift / peak, 0, 1);
  Serial.print(F("% of the signal"));
  if (fitted) {
    float dc[NUM_CELLS], df[3];
    for (uint8_t i = 0; i < NUM_CELLS; i++) dc[i] = (i == worstCell) ? (float)worstDrift : 0;
    applyMatrix(dc, df);
    Serial.print(F("  (~"));
    pf(toDisplay(fabsf(df[2] - bias[2])), 0, 4);
    Serial.print(' '); Serial.print(unitName()); Serial.print(F(" of Fz)"));
  }
  Serial.println();
  if (worstDrift > 0.10 * peak)
    Serial.println(F("#   ! over 10% of the signal - this point is mostly drift, redo it"));
  else if (worstDrift > ZERO_DRIFT_K * se)
    Serial.println(F("#   ! larger than the noise - let it warm up, or work faster"));

  for (uint8_t c = 0; c < nFound; c++) zeroCount[c] = z1.mean[c];   // re-tare from the closing zero
  return true;
}

// Convert a known mass and load angle to a force vector in newtons.
// angleDeg is measured from +Z: 0 deg is axial and 90 deg is purely lateral.
void loadVector(uint8_t axis, float grams, float angleDeg, float f[3]) {
  const float mag = fabsf(grams) / 1000.0f * G0;
  f[0] = f[1] = f[2] = 0;
  if (axis == 2) {
    f[2] = grams / 1000.0f * G0;                // straight down, sign kept
    return;
  }
  const float th = angleDeg * 0.0174532925f;
  f[axis] = (grams < 0 ? -mag : mag) * sinf(th);
  f[2]    = mag * cosf(th);
}

// True for a single-axis load; the crosstalk table skips mixed loads.
bool isPureLoad(const Sample &s) {
  uint8_t nz = 0;
  for (uint8_t r = 0; r < 3; r++) if (fabsf(s.f[r]) > 1e-6f) nz++;
  return nz <= 1;
}

// Capture one calibration sample, then refit once enough samples exist.
// axis: 0=X 1=Y 2=Z. Negative grams apply the load in the opposite direction.
void addSample(uint8_t axis, float grams, float angleDeg) {
  if (nFound != NUM_CELLS) {
    Serial.println(F("# ! all four cells must be present to calibrate"));
    return;
  }
  if (nSamples >= MAX_SAMPLES) {
    Serial.println(F("# sample buffer full - 'k a' to clear, or 'k <i>' to drop one"));
    return;
  }
  if (noiseSd[0] == 0) {
    Serial.println(F("# ! tare first ('t') - the stability check needs the noise floor"));
    return;
  }

  const char an = "XYZ"[axis];
  const bool positive = grams > 0;
  Serial.print(F("# --- sample ")); Serial.print(nSamples + 1);
  Serial.print(F(": ")); pf(fabs(grams), 0, 1);
  Serial.print(F(" g along ")); Serial.print(positive ? '+' : '-');
  Serial.print(an);
  if (axis != 2) { Serial.print(F(" at ")); pf(angleDeg, 0, 1); Serial.print(F(" deg")); }
  Serial.println(F(" ---"));

  char msg[80];
  if (axis == 2) {
    snprintf(msg, sizeof(msg), "stack %d g on the tip, CENTERED", (int)fabsf(grams));
  } else if (fabsf(angleDeg - 90.0f) < 0.05f) {
    snprintf(msg, sizeof(msg), "hang %d g on the string, pulling %c%c, string LEVEL",
             (int)fabsf(grams), positive ? '+' : '-', an);
  } else {
    snprintf(msg, sizeof(msg), "apply %d g toward %c%c at %d deg from +Z, angle EXACT",
             (int)fabsf(grams), positive ? '+' : '-', an, (int)angleDeg);
  }

  float c[NUM_CELLS];
  // Formatted at runtime, so it cannot use the F() macro.
  Serial.print(F("# ")); Serial.println(msg);
  if (!captureBracketed(F("apply the load as described above"), c)) return;

  Sample &sm = samples[nSamples];
  for (uint8_t i = 0; i < NUM_CELLS; i++) sm.c[i] = c[i];
  loadVector(axis, grams, angleDeg, sm.f);
  sm.axis = axis;
  nSamples++;

  if (axis != 2 && fabsf(angleDeg - 90.0f) >= 0.05f) {
    Serial.print(F("# target: F")); Serial.print(an); Serial.print(F(" "));
    pf(toDisplay(sm.f[axis]), 0, 4);
    Serial.print(F("  Fz ")); pf(toDisplay(sm.f[2]), 0, 4);
    Serial.print(' '); Serial.println(unitName());
  }

  Serial.print(F("# counts:"));
  for (uint8_t i = 0; i < NUM_CELLS; i++) pf(sm.c[i], 11, 0);
  Serial.println();

  uint8_t n[3]; bool pos[3], neg[3];
  axisCensus(n, pos, neg);
  if (nSamples < NPAR || n[0] == 0 || n[1] == 0 || n[2] == 0) {
    Serial.print(F("# have X=")); Serial.print(n[0]);
    Serial.print(F(" Y=")); Serial.print(n[1]);
    Serial.print(F(" Z=")); Serial.print(n[2]);
    Serial.print(F(", need all three axes and >= ")); Serial.print(NPAR);
    Serial.println(F(" samples before it can fit"));
    return;
  }
  if (fitMatrix(lastFit, true)) reportFit();
}

// Measure a known load without adding it to the fit, to check the calibration
// against data it was not fitted to.
void verifySample(uint8_t axis, float grams, float angleDeg) {
  if (!fitted) { Serial.println(F("# fit something first ('f')")); return; }

  const char an = "XYZ"[axis];
  Serial.print(F("# --- verify: ")); pf(fabs(grams), 0, 1);
  Serial.print(F(" g along ")); Serial.print(grams > 0 ? '+' : '-');
  Serial.print(an);
  if (axis != 2) { Serial.print(F(" at ")); pf(angleDeg, 0, 1); Serial.print(F(" deg")); }
  Serial.println(F(" (not added to the fit) ---"));

  float c[NUM_CELLS];
  if (!captureBracketed(F("apply the verification load"), c)) return;

  float f[3], want[3];
  loadVector(axis, grams, angleDeg, want);
  applyMatrix(c, f);

  const char *nm[3] = {"Fx", "Fy", "Fz"};
  Serial.println(F("# axis   applied  measured     error"));
  for (uint8_t r = 0; r < 3; r++) {
    Serial.print(F("#   ")); Serial.print(nm[r]);
    pf(toDisplay(want[r]), 10, 4);
    pf(toDisplay(f[r]), 10, 4);
    pf(toDisplay(f[r] - want[r]), 10, 4);
    Serial.println();
  }
  Serial.print(F("#   units ")); Serial.print(unitName());
  if (fabs(want[axis]) > 0) {
    Serial.print(F(", on-axis error "));
    pf(100.0 * (f[axis] - want[axis]) / want[axis], 0, 2);
    Serial.print(F(" %"));
  }
  Serial.println();
}

// Axis finder
// Interactive X/Y orientation aid. Re-tares on entry and reports the cell
// response pattern for a lateral push.
void axisFinder() {
  if (nFound != NUM_CELLS) { Serial.println(F("# ! need all four cells")); return; }

  Serial.println(F("# --- axis finder: keep it UNLOADED while it zeroes ---"));
  Block b;
  if (!acquire(b, NOISE_SAMPLES)) { Serial.println(F("# ! read failed")); return; }
  double noiseMean = 0;
  for (uint8_t c = 0; c < nFound; c++) {
    zeroCount[c] = b.mean[c];
    noiseSd[c]   = b.sd[c];
    noiseMean   += b.sd[c];
  }
  noiseMean /= nFound;
  const double threshold = 8 * (noiseMean > 1 ? noiseMean : 1);

  Serial.println(F("# push the TIP sideways and HOLD. Any key stops."));
  Serial.println(F("#        c0        c1        c2        c3   pattern"));
  while (Serial.available()) Serial.read();

  for (;;) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    if (!acquire(b, 32)) break;                    // ~0.1 s per update

    float c[NUM_CELLS];
    double mag = 0;
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      c[i] = (float)(b.mean[i] - zeroCount[i]);
      mag += fabs(c[i]);
    }
    mag /= NUM_CELLS;

    Serial.print(F("# "));
    for (uint8_t i = 0; i < NUM_CELLS; i++) pf(c[i], 10, 0);
    if (mag < threshold) { Serial.println(F("   (push harder)")); continue; }

    Serial.print(F("   up:"));
    for (uint8_t i = 0; i < NUM_CELLS; i++) if (c[i] > 0) { Serial.print(' '); Serial.print(i); }
    Serial.print(F("  down:"));
    for (uint8_t i = 0; i < NUM_CELLS; i++) if (c[i] <= 0) { Serial.print(' '); Serial.print(i); }

    if (fitted) {
      float f[3];
      applyMatrix(c, f);
      Serial.print(F("   Fx ")); pf(toDisplay(f[0]), 0, 3);
      Serial.print(F("  Fy ")); pf(toDisplay(f[1]), 0, 3);
      Serial.print(F("  bearing ")); pf(atan2(f[1], f[0]) * 57.2957795, 0, 0);
      Serial.print(F(" deg"));
    }
    Serial.println();
  }

  Serial.println(F("# stopped. The pair that split is one axis; push 90 deg round"));
  Serial.println(F("# for the other. Mark the housing so +X means the same tomorrow."));
}

// List all samples with their on-axis residuals.
void dumpSamples() {
  if (!nSamples) { Serial.println(F("# no samples")); return; }
  Serial.println(F("#  i ax  deg        c0         c1         c2         c3      applied     measured        resid"));
  for (uint8_t s = 0; s < nSamples; s++) {
    float f[3];
    applyMatrix(samples[s].c, f);
    uint8_t a = samples[s].axis;
    Serial.print(F("# ")); pf(s, 2, 0);
    Serial.print(F("  ")); Serial.print(samples[s].f[a] >= 0 ? '+' : '-');
    Serial.print("XYZ"[a]);
    // Direction of the applied load from +Z: 90 = purely lateral.
    const float lat = sqrtf(samples[s].f[0] * samples[s].f[0] +
                            samples[s].f[1] * samples[s].f[1]);
    pf(atan2f(lat, samples[s].f[2]) * 57.2957795f, 5, 0);
    for (uint8_t i = 0; i < NUM_CELLS; i++) pf(samples[s].c[i], 11, 0);
    pf(toDisplay(samples[s].f[a]), 13, 4);
    pf(toDisplay(f[a]), 13, 4);
    pf(toDisplay(f[a] - samples[s].f[a]), 13, 4);
    Serial.println();
  }
  Serial.print(F("# applied/measured/resid are the ON-AXIS component, in "));
  Serial.println(unitName());
}

// Remove sample idx and refit, or clear all samples if idx < 0.
void dropSample(int idx) {
  if (idx < 0) {                                  // drop all
    nSamples = 0;
    fitted = false;
    lastFit.ok = false;
    Serial.println(F("# all samples dropped (matrix kept until refit - 'c' clears it)"));
    return;
  }
  if (idx >= nSamples) { Serial.println(F("# no such sample - 'D' to list")); return; }
  for (uint8_t s = idx; s + 1 < nSamples; s++) samples[s] = samples[s + 1];
  nSamples--;
  Serial.print(F("# dropped sample ")); Serial.println(idx);
  if (fitMatrix(lastFit, false)) reportFit();
  else { fitted = false; Serial.println(F("# too few samples left to refit")); }
}

// Text backup
// Export calibration samples as text for backup and restore. The matrix is
// refit on import rather than serialized independently.
void exportCal() {
  Serial.println(F("# ===== LC 3-Axis calibration export ====="));
  Serial.println(F("# Paste this whole block back in to restore, then 'w' to save."));
  Serial.print(F("# ")); Serial.print(nSamples); Serial.print(F(" sample(s), mux ch"));
  for (uint8_t c = 0; c < nFound; c++) { Serial.print(' '); Serial.print(muxChan[c]); }
  Serial.println();
  Serial.println(F("# fields: !s <axis 0=X 1=Y 2=Z> <c0> <c1> <c2> <c3> <Fx> <Fy> <Fz N>"));

  Serial.println(F("!k"));
  Serial.print(F("!c"));
  for (uint8_t c = 0; c < nFound; c++) { Serial.print(' '); Serial.print(muxChan[c]); }
  Serial.println();

  for (uint8_t s = 0; s < nSamples; s++) {
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "!s %u", (unsigned)samples[s].axis);
    for (uint8_t i = 0; i < NUM_CELLS; i++) { buf[n++] = ' '; n += fmtF(buf + n, samples[s].c[i], 1); }
    for (uint8_t r = 0; r < 3; r++)       { buf[n++] = ' '; n += fmtF(buf + n, samples[s].f[r], 6); }
    buf[n++] = '\n';
    Serial.write(buf, n);
  }
  Serial.println(F("!e"));
  Serial.println(F("# ===== end ====="));
}

// Parse one '!' line of a pasted export. '#' lines are ignored by the command
// parser, so the whole block, headers included, can be pasted verbatim.
void importLine(char *s) {
  while (*s == ' ') s++;
  const char kind = *s ? *s++ : 0;
  char *end;

  switch (kind) {
    case 'k':
      streaming = false;                       // pause streaming during the import
      nSamples  = 0;
      fitted    = false;
      lastFit.ok = false;
      Serial.println(F("# import: samples cleared"));
      break;

    case 'c': {                                // channel map the samples were taken on
      uint8_t got[NUM_CELLS], n = 0;
      while (n < NUM_CELLS) {
        long v = strtol(s, &end, 10);
        if (end == s) break;
        got[n++] = (uint8_t)v;
        s = end;
      }
      bool same = (n == nFound);
      for (uint8_t i = 0; i < n && same; i++) same = (got[i] == muxChan[i]);
      if (!same)
        Serial.println(F("# ! import: mux map differs from this hardware - cell order"
                         " may not correspond, counts could be scrambled"));
      break;
    }

    case 's': {
      if (nSamples >= MAX_SAMPLES) { Serial.println(F("# ! import: buffer full")); return; }
      double v[8];
      for (uint8_t i = 0; i < 8; i++) {
        v[i] = strtod(s, &end);
        if (end == s) { Serial.println(F("# ! import: malformed !s line, skipped")); return; }
        s = end;
      }
      if (v[0] < 0 || v[0] > 2) { Serial.println(F("# ! import: bad axis, skipped")); return; }
      Sample &sm = samples[nSamples];
      sm.axis = (uint8_t)v[0];
      for (uint8_t i = 0; i < NUM_CELLS; i++) sm.c[i] = (float)v[1 + i];
      for (uint8_t r = 0; r < 3; r++)        sm.f[r] = (float)v[5 + r];
      nSamples++;
      break;
    }

    case 'e':
      Serial.print(F("# import: ")); Serial.print(nSamples);
      Serial.println(F(" sample(s) restored"));
      if (fitMatrix(lastFit, true)) {
        reportFit();
        Serial.println(F("# refitted - 'w' to save to EEPROM, 't' to re-tare"));
      }
      break;

    default:
      Serial.println(F("# ? unrecognised '!' line"));
  }
}

// EEPROM
void saveCal() {
  int a = EEPROM_ADDR;
  EEPROM.put(a, (uint16_t)EEPROM_MAGIC);                       a += sizeof(uint16_t);
  for (uint8_t c = 0; c < NUM_CELLS; c++) { EEPROM.put(a, muxChan[c]);   a += 1; }
  EEPROM.put(a, (uint8_t)(fitted ? 1 : 0));                    a += 1;
  EEPROM.put(a, nSamples);                                     a += 1;
  for (uint8_t c = 0; c < NUM_CELLS; c++) { EEPROM.put(a, (float)zeroCount[c]); a += sizeof(float); }
  for (uint8_t r = 0; r < 3; r++)
    for (uint8_t i = 0; i < NUM_CELLS; i++) { EEPROM.put(a, M[r][i]); a += sizeof(float); }
  for (uint8_t r = 0; r < 3; r++) { EEPROM.put(a, bias[r]); a += sizeof(float); }
  for (uint8_t s = 0; s < nSamples; s++) { EEPROM.put(a, samples[s]); a += sizeof(Sample); }
  Serial.print(F("# saved ")); Serial.print(a - EEPROM_ADDR);
  Serial.print(F(" bytes, ")); Serial.print(nSamples); Serial.println(F(" sample(s)"));
}

bool loadCal() {
  int a = EEPROM_ADDR;
  uint16_t magic;
  EEPROM.get(a, magic);                                        a += sizeof(uint16_t);
  if (magic != EEPROM_MAGIC) { Serial.println(F("# no saved calibration")); return false; }

  uint8_t chan[NUM_CELLS], f8, n8;
  for (uint8_t c = 0; c < NUM_CELLS; c++) { EEPROM.get(a, chan[c]); a += 1; }
  EEPROM.get(a, f8);                                           a += 1;
  EEPROM.get(a, n8);                                           a += 1;

  // Refuse calibration data if the discovered channel map has changed.
  bool sameMap = (nFound == NUM_CELLS);
  for (uint8_t c = 0; c < NUM_CELLS && sameMap; c++) sameMap = (chan[c] == muxChan[c]);
  if (!sameMap) {
    Serial.print(F("# ! saved calibration was for mux ch"));
    for (uint8_t c = 0; c < NUM_CELLS; c++) { Serial.print(' '); Serial.print(chan[c]); }
    Serial.println(F(" - hardware does not match, NOT loading"));
    return false;
  }

  float v;
  for (uint8_t c = 0; c < NUM_CELLS; c++) { EEPROM.get(a, v); zeroCount[c] = v; a += sizeof(float); }
  for (uint8_t r = 0; r < 3; r++)
    for (uint8_t i = 0; i < NUM_CELLS; i++) { EEPROM.get(a, M[r][i]); a += sizeof(float); }
  for (uint8_t r = 0; r < 3; r++) { EEPROM.get(a, bias[r]); a += sizeof(float); }
  nSamples = (n8 <= MAX_SAMPLES) ? n8 : 0;
  for (uint8_t s = 0; s < nSamples; s++) { EEPROM.get(a, samples[s]); a += sizeof(Sample); }

  for (uint8_t r = 0; r < 3; r++)
    for (uint8_t i = 0; i < NUM_CELLS; i++)
      if (!isfinite(M[r][i])) { Serial.println(F("# ! corrupt matrix, ignoring")); return false; }

  fitted = f8 != 0;
  if (nSamples >= NPAR) fitMatrix(lastFit, false);   // rebuild the residual report
  Serial.print(F("# loaded calibration ("));
  Serial.print(nSamples); Serial.println(F(" sample(s)) - 't' to re-tare, zeros drift"));
  return true;
}

// Reset the in-RAM calibration to the nominal matrix. This must not touch
// EEPROM: setup() calls it to set defaults before loadCal() runs.
void resetCalRAM() {
  const float nomX[NUM_CELLS] = { 1, -1, -1,  1};
  const float nomY[NUM_CELLS] = { 1,  1, -1, -1};
  const float nomZ[NUM_CELLS] = { 1,  1,  1,  1};
  for (uint8_t i = 0; i < NUM_CELLS; i++) { M[0][i] = nomX[i]; M[1][i] = nomY[i]; M[2][i] = nomZ[i]; }
  for (uint8_t r = 0; r < 3; r++) bias[r] = 0;
  for (uint8_t c = 0; c < NUM_CELLS; c++) zeroCount[c] = 0;
  nSamples = 0;
  fitted = false;
  lastFit.ok = false;
}

// 'c' command: reset the calibration in RAM and erase it from EEPROM.
// Requires explicit confirmation ('c yes').
void clearCal(bool confirmed) {
  if (!confirmed) {
    Serial.println(F("# 'c' erases the calibration in RAM AND in EEPROM."));
    Serial.print(F("#   ")); Serial.print(nSamples);
    Serial.println(F(" sample(s) would be lost. Type 'c yes' if you mean it."));
    Serial.println(F("#   'E' first if you want a text backup."));
    return;
  }
  resetCalRAM();
  EEPROM.put(EEPROM_ADDR, (uint16_t)0);
  Serial.println(F("# calibration cleared (matrix, zeros, samples, EEPROM)"));
}

// Reports
void status() {
  Serial.print(F("# cells ")); Serial.print(nFound);
  Serial.print(F("/")); Serial.print(NUM_CELLS); Serial.print(F(" on mux ch"));
  for (uint8_t c = 0; c < nFound; c++) { Serial.print(' '); Serial.print(muxChan[c]); }
  Serial.print(F("   gain 128, 320 SPS, I2C ")); Serial.print(I2C_HZ / 1000);
  Serial.println(F(" kHz"));

  Serial.println(F("# cell        zero    noise sd         raw"));
  for (uint8_t c = 0; c < nFound; c++) {
    Serial.print(F("#    ")); Serial.print(c);
    pf(zeroCount[c], 12, 0);
    pf(noiseSd[c], 12, 1);
    pf(rawCount[c], 12, 0);
    Serial.println();
  }

  Serial.println(fitted ? F("# matrix: FITTED") : F("# matrix: nominal guess, NOT calibrated"));
  const char *nm[3] = {"Fx", "Fy", "Fz"};
  for (uint8_t r = 0; r < 3; r++) {
    Serial.print(F("#   ")); Serial.print(nm[r]); Serial.print(F(" ="));
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      Serial.print(F("  ")); pf(M[r][i] * 1e6, 0, 4); Serial.print(F("e-6*c")); Serial.print(i);
    }
    Serial.print(F("  + ")); pf(bias[r], 0, 5);
    Serial.println(F("   [N]"));
  }

  uint8_t n[3]; bool pos[3], neg[3];
  axisCensus(n, pos, neg);
  Serial.print(F("# samples ")); Serial.print(nSamples);
  Serial.print(F("/")); Serial.print(MAX_SAMPLES);
  Serial.print(F("  (X=")); Serial.print(n[0]);
  Serial.print(F(" Y=")); Serial.print(n[1]);
  Serial.print(F(" Z=")); Serial.print(n[2]); Serial.println(')');
  if (lastFit.ok) reportFit();

  Serial.print(F("# output ")); Serial.print(unitName());
  Serial.print(showCells ? F(", +cells") : F(""));
  Serial.print(showStamp ? F(", +micros") : F(""));
  Serial.print(F(", ema ")); pf(emaAlpha, 0, 2);
  Serial.print(F(", cal avg ")); Serial.print(calSamples);
  Serial.println(streaming ? F(", RUNNING") : F(", PAUSED"));
}

void help() {
  Serial.println(F("# == LC 3-Axis =="));
  Serial.println(F("# stream:  r run   p pause   d +cells   m +micros   u N/gf   e A  ema alpha"));
  Serial.println(F("# setup:   i rescan bus   t tare (UNLOADED)   n N  averaging per cal read"));
  Serial.println(F("#          o axis finder - push the tip to see which pair of cells splits"));
  Serial.println(F("#          b S  drift test - S seconds unloaded, per-cell counts/s"));
  Serial.println(F("# calibrate (string + known masses):"));
  Serial.println(F("#   x W    pull the tip along X with W grams   (negative W = -X)"));
  Serial.println(F("#   y W    pull the tip along Y with W grams   (negative W = -Y)"));
  Serial.println(F("#   z W    press W grams straight down on the tip CENTER"));
  Serial.println(F("#   x W A  same, but the load acts at A deg from +Z (90 = level pull,"));
  Serial.println(F("#          70 = tilted, giving cos70 = 0.34 of the load as Fz)"));
  Serial.println(F("#   f      refit and report      D  list samples + residuals"));
  Serial.println(F("#   v <axis> W   apply a known load WITHOUT fitting it - the real test"));
  Serial.println(F("#   k <i>  drop sample i         k a  drop all"));
  Serial.println(F("# store:   w save   l load   c yes = clear    s status   h help"));
  Serial.println(F("# backup:  E export as text - copy to a file. Paste it back to restore."));
  Serial.println(F("# each sample is zero-bracketed: unload, load, unload. 'q' aborts a prompt."));
  Serial.print(F("# stream format: "));
  if (showStamp) Serial.print(F("micros,"));
  Serial.print(F("Fx,Fy,Fz"));
  if (showCells) Serial.print(F(",c0,c1,c2,c3"));
  Serial.print(F("   in ")); Serial.print(unitName());
  Serial.println(F("   ('#' lines are messages)"));
}

// Command parsing
// Commands are newline-delimited; the buffer also accommodates imported
// calibration sample records.
char lineBuf[160];
uint8_t lineLen = 0;

// "<grams> [angle]"; the angle defaults to 90 deg (purely lateral).
bool parseLoad(const char *s, float &grams, float &angleDeg) {
  char *end;
  grams = strtod(s, &end);
  if (end == s || grams == 0) return false;
  angleDeg = 90.0f;
  const char *p = end;
  float a = strtod(p, &end);
  if (end != p) {
    if (a < 0 || a > 180) return false;
    angleDeg = a;
  }
  return true;
}

void runCommand(char *s) {
  while (*s == ' ') s++;
  char cmd = *s;
  if (!cmd) return;
  s++;

  // Axis letter to index (0=X 1=Y 2=Z), or -1 if it is not an axis.
  auto axisOf = [](char ch) -> int8_t {
    if (ch == 'x' || ch == 'X') return 0;
    if (ch == 'y' || ch == 'Y') return 1;
    if (ch == 'z' || ch == 'Z') return 2;
    return -1;
  };

  switch (cmd) {
    case 'h': help(); break;
    case 's': status(); break;
    case 'i': discoverCells(true); break;
    case 'r': streaming = true;  lastFrameMs = millis(); Serial.println(F("# running")); break;
    case 'p': streaming = false; Serial.println(F("# paused")); break;
    case 't': tareAll(); break;
    case 'd': showCells = !showCells;
              Serial.println(showCells ? F("# cells on") : F("# cells off")); break;
    case 'm': showStamp = !showStamp;
              Serial.println(showStamp ? F("# micros on") : F("# micros off")); break;
    case 'u': useNewton = !useNewton;
              Serial.print(F("# units ")); Serial.println(unitName()); break;
    case 'e': {
      float a = strtod(s, nullptr);
      if (!(a > 0 && a <= 1)) { Serial.println(F("# usage: e <0..1>, 1 = no filter")); break; }
      emaAlpha = a;
      Serial.print(F("# ema alpha ")); pf(emaAlpha, 0, 3); Serial.println();
      break;
    }
    case 'n': {
      long v = strtol(s, nullptr, 10);
      if (v < MIN_CAL_SAMPLES || v > MAX_CAL_SAMPLES) {
        Serial.print(F("# usage: n <")); Serial.print(MIN_CAL_SAMPLES);
        Serial.print(F("..")); Serial.print(MAX_CAL_SAMPLES); Serial.println(F(">"));
        break;
      }
      calSamples = (uint16_t)v;
      Serial.print(F("# averaging ")); Serial.print(calSamples);
      Serial.print(F(" samples per read (~")); pf(calSamples / 320.0, 0, 1);
      Serial.println(F(" s)"));
      break;
    }
    case 'x': case 'y': case 'z': {
      int8_t axis = axisOf(cmd);
      float g, ang;
      if (!parseLoad(s, g, ang)) {
        Serial.println(F("# usage: x 200   x -200   x 200 70   (grams, then deg from +Z)"));
        break;
      }
      if (fabsf(g) > CELL_RATING_G * NUM_CELLS) {
        Serial.println(F("# that exceeds the array rating - refusing"));
        break;
      }
      addSample((uint8_t)axis, g, ang);
      break;
    }
    case 'v': {
      while (*s == ' ') s++;
      int8_t axis = axisOf(*s);
      float g, ang;
      if (axis < 0 || !parseLoad(s + 1, g, ang)) {
        Serial.println(F("# usage: v x 150   or   v x 150 70"));
        break;
      }
      verifySample((uint8_t)axis, g, ang);
      break;
    }
    case 'f': if (fitMatrix(lastFit, true)) reportFit(); break;
    case 'o': axisFinder(); break;
    case 'b': {
      long v = strtol(s, nullptr, 10);
      if (v <= 0) v = 60;
      if (v < 5) v = 5;
      if (v > 600) v = 600;
      driftTest((uint16_t)v);
      break;
    }
    case 'D': dumpSamples(); break;
    case 'k': {
      while (*s == ' ') s++;
      if (*s == 'a' || *s == 'A') { dropSample(-1); break; }
      if (*s < '0' || *s > '9') { Serial.println(F("# usage: k <index>  or  k a")); break; }
      dropSample((int)strtol(s, nullptr, 10));
      break;
    }
    case 'w': saveCal(); break;
    case 'l': loadCal(); break;
    case 'c': {
      while (*s == ' ') s++;
      clearCal(s[0] == 'y');
      break;
    }
    case 'E': exportCal(); break;
    case '!': importLine(s); break;
    case '#': break;                  // comment line from a pasted export block
    default:  Serial.print(F("# ? '")); Serial.print(cmd); Serial.println(F("' - h for help"));
  }
}

void serviceSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) { lineBuf[lineLen] = 0; runCommand(lineBuf); lineLen = 0; }
      return;                                   // one command per pass
    }
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = ch;
  }
}

// Arduino entry points
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Wire.begin();
  Wire.setClock(I2C_HZ);

  resetCalRAM();                 // defaults only; does not touch EEPROM
  discoverCells(true);
  loadCal();
  help();
  Serial.println(F("# 't' to tare, then 'r' to stream. Calibration starts at 'x'/'y'/'z'."));
}

void loop() {
  serviceSerial();
  if (!streaming || nFound == 0) return;

  pollCells();
  if (frameReady()) {
    emitFrame();
  } else if (millis() - lastFrameMs > FRAME_TIMEOUT_MS) {
    emitFrame();                                // a cell stopped updating; keep the stream alive
  }
}
