#include "Speaker.h"
#include <driver/ledc.h>   // ledc api
#include "GPIOManager.hpp"
#include "ControlConfig.hpp"


struct Note { uint16_t freq; uint16_t dur_ms; };
struct Melody { const Note* notes; uint8_t len; };

//melodies played for each state
static const Note MEL_STOP[]    = { {1200,120},{900,120},{700,160},{0,80} };
static const Note MEL_FORWARD[] = { {784,120},{880,120},{988,140},{1047,220},{0,80} };
static const Note MEL_LEFT[]    = { {659,90},{0,60},{659,90},{0,60},{523,180},{0,60} };
static const Note MEL_RIGHT[]   = { {740,120},{932,120},{1175,140},{1109,160},{0,80} };

static const Melody MELODIES[] = {
  {MEL_STOP,    (uint8_t)(sizeof(MEL_STOP)/sizeof(Note))},
  {MEL_FORWARD, (uint8_t)(sizeof(MEL_FORWARD)/sizeof(Note))},
  {MEL_LEFT,    (uint8_t)(sizeof(MEL_LEFT)/sizeof(Note))},
  {MEL_RIGHT,   (uint8_t)(sizeof(MEL_RIGHT)/sizeof(Note))}
};

static uint8_t       s_audioPin   = 255;                 // set in begin
// static ledc_channel_t s_channel    = LEDC_CHANNEL_0;     // 0..7 per speed mode ; changed to channel 2
// static ledc_timer_t   s_timer      = LEDC_TIMER_0;       // use one timer for audio ; changed to timer 2
static ledc_mode_t    s_speedMode  = LEDC_LOW_SPEED_MODE;
static uint8_t        s_resBits    = 10;
static uint32_t       s_maxDuty    = 1023;
static uint32_t       s_playDuty   = 512;

static uint8_t        s_melIndex   = 0;
static bool           s_melActive  = false;
static unsigned long  s_melNextMs  = 0;
static int            s_currentDir = 0;

//used to set up duty cycle
static void applyDuty(uint32_t duty) {
  GPIOManager& gpio = GPIOManager::getInstance();
  gpio.writePWM(s_audioPin, duty);

  // ledc_set_duty(s_speedMode, s_channel, duty);
  // ledc_update_duty(s_speedMode, s_channel);
}

static void stopTone() {
  applyDuty(0);
}

// plays certain note
static void playFreq(uint16_t freq) {
  if (freq == 0) { stopTone(); return; }
  // ledc_set_freq(s_speedMode, s_timer, freq);
  GPIOManager::getInstance().setFrequency(s_audioPin, freq);
  applyDuty(s_playDuty);
}

// sets up speaker
void speakerBegin(uint8_t audioPin, uint8_t ledcChannel, uint8_t resolutionBits, uint8_t dutyPercent)
{
  ControlConfig& config = ControlConfig::getInstance();
  s_audioPin  = audioPin; // would need to be set to 23

  // Use GPIOManager to configure pin as output
  GPIOManager& gpio = GPIOManager::getInstance();
  gpio.configurePWMPin(s_audioPin, config.feedback.audioFrequency, resolutionBits); // Let GPIOManager handle PWM setup


  // s_channel   = static_cast<ledc_channel_t>(ledcChannel);  // 0..7
  s_resBits   = resolutionBits;
  s_maxDuty   = (1UL << s_resBits) - 1;
  if (dutyPercent > 100) dutyPercent = 100;
  s_playDuty  = (s_maxDuty * dutyPercent) / 100;

  // sets up ledc so that it can play tones
  // ledc_timer_config_t tcfg = {};
  // tcfg.speed_mode       = s_speedMode;
  // tcfg.timer_num        = s_timer;            
  // tcfg.duty_resolution  = (ledc_timer_bit_t)s_resBits;
  // tcfg.freq_hz          = 1000;               // initial freq used
  // tcfg.clk_cfg          = LEDC_AUTO_CLK;
  // ledc_timer_config(&tcfg);

  // sets up channel so tones can actually work on it
  // ledc_channel_config_t ccfg = {};
  // ccfg.gpio_num       = s_audioPin;
  // ccfg.speed_mode     = s_speedMode;
  // ccfg.channel        = s_channel;
  // ccfg.intr_type      = LEDC_INTR_DISABLE;
  // ccfg.timer_sel      = s_timer;
  // ccfg.duty           = 0;                    // start silent
  // ccfg.hpoint         = 0;
  // ledc_channel_config(&ccfg);

}

// starts melody based on direction
void startMelodyForDirection(int direction) {
  if (direction < 0 || direction > 3) return;
  s_currentDir = direction;

  const Melody& m = MELODIES[direction];
  if (m.len == 0) { s_melActive = false; return; }

  s_melIndex  = 0;
  s_melActive = true;

  playFreq(m.notes[0].freq);
  s_melNextMs = millis() + m.notes[0].dur_ms;
}

// plays the melody consistently after it is started
void serviceMelody() {
  if (!s_melActive) return;

  unsigned long now = millis();
  if ((long)(now - s_melNextMs) >= 0) {
    const Melody& m = MELODIES[s_currentDir];
    s_melIndex++;
    if (s_melIndex >= m.len) s_melIndex = 0;

    const Note& n = m.notes[s_melIndex];
    playFreq(n.freq);
    s_melNextMs = now + n.dur_ms;
  }
}

// stops note
void speakerStop() {
  s_melActive = false;
  stopTone();
}
