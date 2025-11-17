// router table controller
// (C) Jef Collin
// 2021..2025


// reminder: when lvgl library is updated, edit the config file!!!


// axis type 1204 4mm/revolution
// stepper driver set to 800 steps/revolution
// one step 0.005mm


// todo


#include "FS.h"
#include <SPI.h>
#include <Preferences.h>
#include <lvgl.h>
#include "FastAccelStepper.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <LovyanGFX.hpp>

extern lv_font_t dseg_60;
extern lv_font_t dseg_175;


#define TAB_MAIN_REF        0
#define TAB_MAINSUB_REF     1
#define TAB_ZERO_REF        2
#define TAB_PRESETS_REF     3
#define TAB_SEQUENCER_REF   4
#define TAB_EDIT_REF        5
#define TAB_CALC_REF        6
#define TAB_WAITBUSY_REF    7
#define TAB_ZOOM_REF        8
#define TAB_DIRECTENTRY_REF 9


// lvgl
#define LV_USE_DEBUG = 0;
// 3.5" display
#define TFT_HOR_RES 480
#define TFT_VER_RES 320
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 5 * (LV_COLOR_DEPTH / 8))

// stepper driver lift DM556 or equivalent
// always enabled
#define LIFT_STEP_DIR 16
#define LIFT_STEP_PULSE 4

// future expansion
// stepper driver fence
#define FENCE_STEP_DIR 17
#define FENCE_STEP_PULSE 13

// range limit in mm
#define MAX_ROUTER_POS 300

// limit switch polarity
// normal closed true
// normal open false
#define LIMIT_NC false

Preferences preferences;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;

// default text for presets and sequences
#define PRESET_DEFAULT "Touch to edit"

// physical buttons/ pedals
#define buttonup_pin 22
#define buttondown_pin 21

// use alps or other decoder
#define Use_Alps_Encoder false

// rotary encode io pins
#define Encoder_1_Pin1 34
#define Encoder_1_Pin2 39
#define Encoder_2_Pin1 26
#define Encoder_2_Pin2 25
#define Encoder_1_Key 36
#define Encoder_2_Key 35

// end of travel switches
#define endswitch_up 33
#define endswitch_down 27

// connection to zero sensor
#define zero_sensor 32

// rotary encoder
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#if (Use_Alps_Encoder)
// Alps EC11 encoder requires half step tables, others need full step
#define R_START 0x0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
#define R_START 0x0

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

unsigned char Encoder_1_State = R_START;
unsigned char Encoder_2_State = R_START;

// track rotary encoder changes
int EncoderCounter1 = 0;
int EncoderCounter2 = 0;

long unsigned timer_encoderbutton1;
long unsigned timer_encoderbutton2;

boolean Encoder_Key1_Long_Press = false;
boolean Encoder_Key2_Long_Press = false;

// stepper commands for stepper task
typedef enum {
  CMD_NONE,
  CMD_SET_SPEED,
  CMD_SET_ACCEL,
  CMD_SET_CURRENTPOSITION,
  CMD_MOVE,
  CMD_MOVE_TO,
  CMD_STOP,
  CMD_EMERGENCY_STOP,
  CMD_SET_THIS_AS_CURRENT,
  CMD_ISRUNNING,
  CMD_GET_CURRENTPOSITION
} stepper_cmd_t;

typedef struct {
  stepper_cmd_t cmd;
  int32_t value;
} stepper_msg_t;

typedef struct {
  bool is_running;
} stepper_running_feedback_t;

typedef struct {
  int32_t current_position;
} stepper_position_feedback_t;

QueueHandle_t stepperQueue;
QueueHandle_t stepperRunningFeedbackQueue;
QueueHandle_t stepperPositionFeedbackQueue;

// state machine states sense zero
enum SenseZeroState {
  SENSE_ZERO_IDLE,
  SENSE_ZERO_INIT,
  SENSE_ZERO_CHECK_TOP_SWITCH,
  SENSE_ZERO_MOVE_DOWN,
  SENSE_ZERO_WAIT_DOWN,
  SENSE_ZERO_DEBOUNCE_DOWN,
  SENSE_ZERO_MOVE_UP,
  SENSE_ZERO_WAIT_UP,
  SENSE_ZERO_COMPLETE,
  SENSE_ZERO_ERROR
};

SenseZeroState senseZeroState = SENSE_ZERO_IDLE;
unsigned long senseZeroTimer = 0;

// state machine states for goto max top
enum GotoMaxTopState {
  GOTO_MAX_TOP_IDLE,
  GOTO_MAX_TOP_INIT,
  GOTO_MAX_TOP_CHECK_TOP_SWITCH,
  GOTO_MAX_TOP_MOVE_DOWN,
  GOTO_MAX_TOP_WAIT_DOWN,
  GOTO_MAX_TOP_DEBOUNCE_DOWN,
  GOTO_MAX_TOP_MOVE_UP,
  GOTO_MAX_TOP_WAIT_UP,
  GOTO_MAX_TOP_DEBOUNCE_UP,
  GOTO_MAX_TOP_MOVE_DOWN_FINAL,
  GOTO_MAX_TOP_WAIT_DOWN_FINAL,
  GOTO_MAX_TOP_DEBOUNCE_FINAL,
  GOTO_MAX_TOP_COMPLETE,
  GOTO_MAX_TOP_ERROR
};

GotoMaxTopState gotoMaxTopState = GOTO_MAX_TOP_IDLE;
unsigned long gotoMaxTopTimer = 0;

enum GotoMaxBottomState {
  GOTO_MAX_BOTTOM_IDLE,
  GOTO_MAX_BOTTOM_INIT,
  GOTO_MAX_BOTTOM_CHECK_BOTTOM_SWITCH,
  GOTO_MAX_BOTTOM_MOVE_UP,
  GOTO_MAX_BOTTOM_WAIT_UP,
  GOTO_MAX_BOTTOM_DEBOUNCE_UP,
  GOTO_MAX_BOTTOM_MOVE_DOWN,
  GOTO_MAX_BOTTOM_WAIT_DOWN,
  GOTO_MAX_BOTTOM_DEBOUNCE_DOWN,
  GOTO_MAX_BOTTOM_MOVE_UP_FINAL,
  GOTO_MAX_BOTTOM_WAIT_UP_FINAL,
  GOTO_MAX_BOTTOM_DEBOUNCE_FINAL,
  GOTO_MAX_BOTTOM_COMPLETE,
  GOTO_MAX_BOTTOM_ERROR
};

GotoMaxBottomState gotoMaxBottomState = GOTO_MAX_BOTTOM_IDLE;
unsigned long gotoMaxBottomTimer = 0;

enum MoveStepperState {
  MOVE_STEPPER_IDLE,
  MOVE_STEPPER_INIT,
  MOVE_STEPPER_START,
  MOVE_STEPPER_MOVING,
  MOVE_STEPPER_RECOVER_UP,
  MOVE_STEPPER_RECOVER_DOWN,
  MOVE_STEPPER_COMPLETE,
  MOVE_STEPPER_ERROR
};

MoveStepperState moveStepperState = MOVE_STEPPER_IDLE;
unsigned long moveStepperTimer = 0;
bool moveStepperDirectionUp = false;
float moveStepperTargetPosition = 0.0;
float moveStepperStepCount = 0.0;

String strbuf;
char printbuf[30];

// calculator
String calc_one = "0";
String calc_two = "0";
String calc_dec_one = "0";
String calc_dec_two = "0";
String calc_ops = "";
boolean calc_dec_active = false;

// direct entry
String direct_one = "0";
String direct_dec_one = "0";
boolean direct_dec_active = false;

// struct for preset and sequencer settings
struct SettingStructure {
  float setting_position;
  uint8_t setting_speed;
  String setting_info;
};

SettingStructure P_S_settings[50];

uint8_t presets_selected = 0;

// graphics lib configuration
class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9488     _panel_instance;
    lgfx::Touch_XPT2046     _touch_instance;
    lgfx::Bus_SPI           _bus_instance;
  public:
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 60000000;
        cfg.freq_read  = 16000000;
        cfg.spi_3wire  = true;
        cfg.use_lock   = true;
        cfg.dma_channel = SPI_DMA_CH_AUTO;
        cfg.pin_sclk = 18;
        cfg.pin_mosi = 23;
        cfg.pin_miso = 19;
        cfg.pin_dc   = 14;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
      }
      {
        auto cfg = _panel_instance.config();
        cfg.pin_cs           =    15;
        cfg.pin_rst          =    -1;
        cfg.pin_busy         =    -1;
        cfg.panel_width      =   320;
        cfg.panel_height     =   480;
        cfg.offset_x         =     0;
        cfg.offset_y         =     0;
        cfg.offset_rotation  =     0;
        cfg.dummy_read_pixel =     8;
        cfg.dummy_read_bits  =     1;
        cfg.readable         =  true;
        cfg.invert           = false;
        cfg.rgb_order        = false;
        cfg.dlen_16bit       = false;
        cfg.bus_shared       =  true;
        _panel_instance.config(cfg);
      }
      {
        // must be raw data point but not used if calibration is used
        auto cfg = _touch_instance.config();
        cfg.x_min      = 0;
        cfg.x_max      = 4000;
        cfg.y_min      = 0;
        cfg.y_max      = 4000;
        cfg.pin_int    = -1;
        cfg.bus_shared = true;
        cfg.offset_rotation = 0;
        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.freq = 1000000;
        cfg.pin_sclk = 18;
        cfg.pin_mosi = 23;
        cfg.pin_miso = 19;
        cfg.pin_cs   =  5;
        _touch_instance.config(cfg);
        _panel_instance.setTouch(&_touch_instance);
      }
      setPanel(&_panel_instance);
    }
};

LGFX TFT;

// callback routine for display
void my_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  lv_draw_sw_rgb565_swap(px_map, w * h);
  TFT.startWrite();
  TFT.pushImageDMA(area->x1, area->y1, w, h, (uint16_t*)px_map);
  TFT.endWrite();
  lv_disp_flush_ready(disp);
}

// two buffer system
void *draw_buf_1;
void *draw_buf_2;

static lv_display_t *disp;

uint16_t setting_calibration_data[8];
uint8_t calDataOK = 0;

String setting_sequencer_info[5];


unsigned long buttonUpPressTime = 0;
unsigned long buttonDownPressTime = 0;
bool buttonUpWasPressed = false;
bool buttonDownWasPressed = false;
bool continuousMoveActive = false;
int8_t continuousMoveDirection = 0;
unsigned long continuousMoveLastStep = 0;
unsigned long continuousMoveInterval = 50; // ms between steps for continuous movement
unsigned long lastContinuousStepTime = 0;


float router_position = 0.0;
float router_old_position = 0;
float newposition = 0;
boolean position_valid = false;
boolean router_cancel = false;
int8_t router_return_from_busy = -1;

// nr of steps per rev / distance per rev in mm
float stepfactor = 800 / 4;

// startup selections for step size and plunge rate
uint8_t router_stepsize = 2;
uint8_t router_speed = 2;

uint8_t error_number = 0;

uint8_t zoomreturnscreen = 0;

// timers
unsigned long LVGL_Timer;

// lvgl variables
static lv_obj_t *tabview;

static lv_obj_t *tabmain;
static lv_obj_t *tabmainsub;
static lv_obj_t *tabzero;
static lv_obj_t *tabpresets;
static lv_obj_t *tabsequencer;
static lv_obj_t *tabedit;
static lv_obj_t *tabcalc;
static lv_obj_t *tabwaitbusy;
static lv_obj_t *tabzoom;
static lv_obj_t *tabdirectentry;

static lv_obj_t * btn_pos_small;
static lv_obj_t * btn_pos_small_label;

static lv_obj_t * btn_pos_small2;
static lv_obj_t * btn_pos_small_label2;

static lv_obj_t * btn_pos_large;
static lv_obj_t * btn_pos_large_label;

static lv_obj_t * btn_gotozero;
static lv_obj_t * btn_setzero;
static lv_obj_t * btn_findzero;
static lv_obj_t * btn_nextmain;
static lv_obj_t * btn_presets;
static lv_obj_t * btn_sequencer;
static lv_obj_t * btn_gomaxup;
static lv_obj_t * btn_gomaxdown;
static lv_obj_t * btn_cancelmove;
static lv_obj_t * btn_startzero;
static lv_obj_t * btn_cancelzero;
static lv_obj_t * ddlist_stepsize;
static lv_obj_t * ddlist_speed;
static lv_obj_t * roller;

static lv_obj_t * btn_calculator;
static lv_obj_t * btn_gotocalc;

static lv_obj_t * btn_preset_next;
static lv_obj_t * btn_preset_previous;

static lv_obj_t * btn_preset_0;
static lv_obj_t * btn_preset_1;
static lv_obj_t * btn_preset_2;
static lv_obj_t * btn_preset_3;
static lv_obj_t * btn_preset_4;

static lv_obj_t * btn_sequencer_next;
static lv_obj_t * btn_sequencer_previous;

lv_style_t btn_style1_selected;

lv_obj_t * img_busy_main;
lv_obj_t * img_busy_mainsub;
lv_obj_t * img_busy_zoom;

static lv_obj_t * lbl_pre_title1;
static lv_obj_t * lbl_pre_title2;
static lv_obj_t * lbl_pre_title3;
static lv_obj_t * lbl_pre_title4;
static lv_obj_t * lbl_pre_title5;

static lv_obj_t * lbl_pre30;
static lv_obj_t * lbl_pre31;
static lv_obj_t * lbl_pre32;
static lv_obj_t * lbl_pre33;
static lv_obj_t * lbl_pre34;

static lv_obj_t * lbl_speed30;
static lv_obj_t * lbl_speed31;
static lv_obj_t * lbl_speed32;
static lv_obj_t * lbl_speed33;
static lv_obj_t * lbl_speed34;

static lv_obj_t * lbl_tag30;
static lv_obj_t * lbl_tag31;
static lv_obj_t * lbl_tag32;
static lv_obj_t * lbl_tag33;
static lv_obj_t * lbl_tag34;

static lv_obj_t * lbl_mark100;
static lv_obj_t * lbl_mark101;
static lv_obj_t * lbl_mark102;
static lv_obj_t * lbl_mark103;
static lv_obj_t * lbl_mark104;

static lv_obj_t * lbl_pre100;
static lv_obj_t * lbl_pre101;
static lv_obj_t * lbl_pre102;
static lv_obj_t * lbl_pre103;
static lv_obj_t * lbl_pre104;

static lv_obj_t * lbl_speed100;
static lv_obj_t * lbl_speed101;
static lv_obj_t * lbl_speed102;
static lv_obj_t * lbl_speed103;
static lv_obj_t * lbl_speed104;

static lv_obj_t * lbl_tag100;
static lv_obj_t * lbl_tag101;
static lv_obj_t * lbl_tag102;
static lv_obj_t * lbl_tag103;
static lv_obj_t * lbl_tag104;

static lv_obj_t * lbl_pre_number;
static lv_obj_t * lbl_seq_number;

static lv_obj_t * lbl_seqtitle100;

static lv_obj_t * btn_busy_cancel;

static lv_obj_t *txtentry;
static lv_obj_t *keyboard;

static lv_obj_t *calc_mtrx1 ;

static const char * btnm_calc[] = {"7", "8", "9", "/", "<", "\n",
                                   "4", "5", "6", "x", "C", "\n",
                                   "1", "2", "3", "-", "R+", "\n",
                                   "0", ".", "=", "+", "R-", ""
                                  };
String calc_disp1 = "0.0";
String calc_disp2 = "0.0";

static lv_obj_t *lbl_calc1;

static lv_obj_t *lbl_calc2;

static lv_obj_t *lbl_calcops;

static lv_obj_t *lbl_directpos;
static lv_obj_t *btn_directgoto;
static lv_obj_t *btn_directcancel;
static lv_obj_t *mtrx_directentry;

static const char * btnm_directentry[] = {"7", "8", "9", "\n",
                                          "4", "5", "6", "\n",
                                          "1", "2", "3", "\n",
                                          "0", ".", "C", ""
                                         };
String calc_directdisplay = "0.0";

// for keyboard routine
uint8_t kb_caller = 0;
uint8_t kb_returntab = 0;
String kb_text = "";

// screen selection
uint8_t preset_mode = 0;
uint8_t sequence_mode = 0;

// sequence step
uint8_t sequence_step = 0;

// touch screen callback
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
  uint16_t touchX, touchY;
  data->state = LV_INDEV_STATE_REL;
  if ( TFT.getTouch( &touchX, &touchY ) )
  {
    data->state = LV_INDEV_STATE_PR;
    if (touchX > 479 || touchY > 319)
    {
    }
    else
    {
      data->point.x = touchX;
      data->point.y = touchY;
    }
  }
}

// encoder interrupts
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][pinstate];
  unsigned char result = Encoder_1_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter1 < 10) {
      EncoderCounter1++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter1 > -10) {
      EncoderCounter1--;
    }
  }
  constrain(EncoderCounter1, -10, +10);
}

void IRAM_ATTR isr2() {
  unsigned char pinstate = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][pinstate];
  unsigned char result = Encoder_2_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter2 < 10) {
      EncoderCounter2++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter2 > -10) {
      EncoderCounter2--;
    }
  }
  constrain(EncoderCounter2, -10, +10);
}

// use Arduinos millis() as tick source
static uint32_t my_tick(void)
{
  return millis();
}

void setup() {
  Serial.begin(115200);

  // stepper motor drivers
  pinMode(LIFT_STEP_DIR, OUTPUT);
  pinMode(LIFT_STEP_PULSE, OUTPUT);

  pinMode(FENCE_STEP_DIR, OUTPUT);
  pinMode(FENCE_STEP_PULSE, OUTPUT);

  digitalWrite(LIFT_STEP_DIR, HIGH);
  digitalWrite(LIFT_STEP_PULSE, HIGH);

  digitalWrite(FENCE_STEP_DIR, HIGH);
  digitalWrite(FENCE_STEP_PULSE, HIGH);

  // buttons and switches
  pinMode(buttonup_pin, INPUT_PULLUP);
  pinMode(buttondown_pin, INPUT_PULLUP);

  pinMode(endswitch_up, INPUT_PULLUP);
  pinMode(endswitch_down, INPUT_PULLUP);
  pinMode(zero_sensor, INPUT_PULLUP);

  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  // get current state to start otherwise encoder might not react to first click
  unsigned char temppinstate1 = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][temppinstate1];

  unsigned char temppinstate2 = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][temppinstate2];

  attachInterrupt(Encoder_1_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_1_Pin2, isr1, CHANGE);
  attachInterrupt(Encoder_2_Pin1, isr2, CHANGE);
  attachInterrupt(Encoder_2_Pin2, isr2, CHANGE);

  TFT.init();
  TFT.setRotation(3);
  TFT.fillScreen(TFT_BLACK);

  lv_init();

  // set a tick source so that LVGL will know how much time elapsed
  lv_tick_set_cb(my_tick);

  // setup the two buffers
  draw_buf_1 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  draw_buf_2 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);

  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf_1, draw_buf_2, DRAW_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  clear_calibration();
  clear_settings();

  boolean forcecalibrate = false;
  // check if one of the buttons is pressed during boot
  if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 1) {
    // make sure its not a power on glitch due to capacitor charging on the encoder module
    delay(100);
    if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 1) {
      // call the calibration screen if one and only one of the encoder switches is pressed during power on
      forcecalibrate = true;
    }
  }
  if (!load_calibration() or forcecalibrate) {
    calibratescreen();
  } else {
    TFT.setTouchCalibrate(setting_calibration_data);
  }

  // factory defaults on both encoders clicked
  if (digitalRead(Encoder_1_Key) == 0 and digitalRead(Encoder_2_Key) == 0) {
    // make sure its not a power on glitch due to capacitor caharging on the encoder module
    delay(500);
    if (digitalRead(Encoder_1_Key) == 0 and digitalRead(Encoder_2_Key) == 0) {
      reset_settings();
    }
  }

  load_settings();

  Display_Splash_Screen();

  // setup stepper driver and tasks
  engine.init();
  stepper = engine.stepperConnectToPin(LIFT_STEP_PULSE);
  if (stepper) {
    stepper->setDirectionPin(LIFT_STEP_DIR);
  }
  stepperQueue = xQueueCreate(10, sizeof(stepper_msg_t));
  stepperRunningFeedbackQueue = xQueueCreate(10, sizeof(stepper_running_feedback_t));
  stepperPositionFeedbackQueue = xQueueCreate(10, sizeof(stepper_position_feedback_t));
  xTaskCreatePinnedToCore(stepperTask, "StepperTask", 4096, NULL, 1, NULL, 0);

  delay(3000);

  Setup_Screens();

  update_position();
  update_stepsize();
  update_speed();
  setspeed(2);
  set_busy_symbol(false);
  update_controls();
  send_command_to_task(CMD_EMERGENCY_STOP, 0);

  LVGL_Timer = millis() - 200;
}

void loop() {
  // check for pending error messages
  if (error_number > 0) {
    show_error_message();
  }

  if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
    check_actions_encoder1();
    check_actions_encoder2();
    check_actions_key1();
    check_actions_key2();
    check_buttons_and_move();
  }

  // sense zero state machine
  switch (senseZeroState) {
    case SENSE_ZERO_IDLE:
      break;

    case SENSE_ZERO_INIT:
      router_cancel = false;
      router_return_from_busy = -1;
      show_busy_screen();
      error_number = 0;
      position_valid = false;
      update_controls();
      // slowest speed
      setspeed(0);
      // reset current position
      send_command_to_task(CMD_EMERGENCY_STOP, 0);
      senseZeroState = SENSE_ZERO_CHECK_TOP_SWITCH;
      break;

    case SENSE_ZERO_CHECK_TOP_SWITCH:
      if (check_endswitch_up() or digitalRead(zero_sensor) == 0) {
        // at max up position or sensor already detected, need to move down 20mm first
        send_command_to_task(CMD_MOVE, -20 * stepfactor);
        senseZeroState = SENSE_ZERO_WAIT_DOWN;
      } else {
        // not at top, proceed to move up
        send_command_to_task(CMD_MOVE, MAX_ROUTER_POS * stepfactor);
        senseZeroState = SENSE_ZERO_WAIT_UP;
      }
      break;

    case SENSE_ZERO_WAIT_DOWN:
      if (router_cancel) {
        invalidate_position();
        senseZeroState = SENSE_ZERO_COMPLETE;
      } else if (!check_endswitch_up() and digitalRead(zero_sensor) != 0) {
        // switch and sensor cleared, move completed
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        senseZeroTimer = millis();
        senseZeroState = SENSE_ZERO_DEBOUNCE_DOWN;
      } else if (check_endswitch_down()) {
        // reached the bottom end switch
        invalidate_position();
        error_number = 2;
        senseZeroState = SENSE_ZERO_COMPLETE;
      } else if (!is_motor_running()) {
        // move is completed without clearing the end switch and/or sensor
        if (check_endswitch_up()) {
          error_number = 1;
        } else if (digitalRead(zero_sensor) == 0) {
          error_number = 8;
        }
        invalidate_position();
        senseZeroState = SENSE_ZERO_COMPLETE;
      }
      break;

    case SENSE_ZERO_DEBOUNCE_DOWN:
      // recheck sensors after timeout
      if (millis() - senseZeroTimer >= 100) {
        // restart
        senseZeroState = SENSE_ZERO_CHECK_TOP_SWITCH;
      }
      break;

    case SENSE_ZERO_WAIT_UP:
      if (router_cancel) {
        invalidate_position();
        senseZeroState = SENSE_ZERO_COMPLETE;
      } else if (digitalRead(zero_sensor) == 0) {
        // Zero sensor triggered
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        position_valid = true;
        senseZeroState = SENSE_ZERO_COMPLETE;
      } else if (check_endswitch_up()) {
        // reached top switch without finding sensor
        invalidate_position();
        error_number = 3;
        senseZeroState = SENSE_ZERO_COMPLETE;
      }
      break;

    case SENSE_ZERO_COMPLETE:
      router_position = 0;
      lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
      update_position();
      senseZeroState = SENSE_ZERO_IDLE;
      update_controls();
      break;

    case SENSE_ZERO_ERROR:
      // handle error state if needed
      senseZeroState = SENSE_ZERO_COMPLETE;
      break;
  }

  // state machine states for goto max top
  switch (gotoMaxTopState) {
    case GOTO_MAX_TOP_IDLE:
      break;

    case GOTO_MAX_TOP_INIT:
      router_cancel = false;
      router_return_from_busy = -1;
      show_busy_screen();
      error_number = 0;
      position_valid = false;
      update_controls();
      setspeed(2);
      send_command_to_task(CMD_EMERGENCY_STOP, 0);
      gotoMaxTopState = GOTO_MAX_TOP_CHECK_TOP_SWITCH;
      break;

    case GOTO_MAX_TOP_CHECK_TOP_SWITCH:
      if (check_endswitch_up()) {
        // at max up position, need to move down 20mm first
        send_command_to_task(CMD_MOVE, -20 * stepfactor);
        gotoMaxTopState = GOTO_MAX_TOP_WAIT_DOWN;
      } else {
        // not at top, proceed to move up
        send_command_to_task(CMD_MOVE, MAX_ROUTER_POS * stepfactor);
        gotoMaxTopState = GOTO_MAX_TOP_WAIT_UP;
      }
      break;

    case GOTO_MAX_TOP_WAIT_DOWN:
      if (router_cancel) {
        invalidate_position();
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      } else if (!check_endswitch_up()) {
        // switch released, move completed
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        gotoMaxTopTimer = millis();
        gotoMaxTopState = GOTO_MAX_TOP_DEBOUNCE_DOWN;
      } else if (check_endswitch_down()) {
        // reached the bottom end switch
        invalidate_position();
        error_number = 2;
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      } else if (!is_motor_running()) {
        // move is completed without clearing the end switch
        invalidate_position();
        error_number = 1;
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      }
      break;

    case GOTO_MAX_TOP_DEBOUNCE_DOWN:
      if (millis() - gotoMaxTopTimer >= 100) {
        // restart
        gotoMaxTopState = GOTO_MAX_TOP_CHECK_TOP_SWITCH;
      }
      break;

    case GOTO_MAX_TOP_WAIT_UP:
      if (router_cancel) {
        invalidate_position();
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      } else if (check_endswitch_up()) {
        // reached top switch
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        gotoMaxTopTimer = millis();
        gotoMaxTopState = GOTO_MAX_TOP_DEBOUNCE_UP;
      } else if (!is_motor_running()) {
        invalidate_position();
        error_number = 4;
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      }
      break;

    case GOTO_MAX_TOP_DEBOUNCE_UP:
      if (millis() - gotoMaxTopTimer >= 100) {
        if (!check_endswitch_up()) {
          invalidate_position();
          error_number = 4;
          gotoMaxTopState = GOTO_MAX_TOP_ERROR;
        } else {
          // Move down 20mm to clear top sensor
          send_command_to_task(CMD_MOVE, -20 * stepfactor);
          gotoMaxTopState = GOTO_MAX_TOP_WAIT_DOWN_FINAL;
        }
      }
      break;

    case GOTO_MAX_TOP_WAIT_DOWN_FINAL:
      if (router_cancel) {
        invalidate_position();
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      } else if (!check_endswitch_up()) {
        // switch released, move completed
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        gotoMaxTopTimer = millis();
        gotoMaxTopState = GOTO_MAX_TOP_DEBOUNCE_FINAL;
      } else if (check_endswitch_down()) {
        // reached the bottom end switch
        invalidate_position();
        error_number = 2;
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      } else if (!is_motor_running()) {
        invalidate_position();
        error_number = 1;
        gotoMaxTopState = GOTO_MAX_TOP_ERROR;
      }
      break;

    case GOTO_MAX_TOP_DEBOUNCE_FINAL:
      if (millis() - gotoMaxTopTimer >= 100) {
        if (check_endswitch_up()) {
          invalidate_position();
          error_number = 1;
          gotoMaxTopState = GOTO_MAX_TOP_ERROR;
        }
        else {
          gotoMaxTopState = GOTO_MAX_TOP_COMPLETE;
        }
      }
      break;

    case GOTO_MAX_TOP_COMPLETE:
      router_position = 0;
      lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
      update_position();
      gotoMaxTopState = GOTO_MAX_TOP_IDLE;
      update_controls();
      break;

    case GOTO_MAX_TOP_ERROR:
      // Handle error state if needed
      gotoMaxTopState = GOTO_MAX_TOP_COMPLETE;
      break;
  }

  // state machine states for goto max bottom
  switch (gotoMaxBottomState) {
    case GOTO_MAX_BOTTOM_IDLE:
      break;

    case GOTO_MAX_BOTTOM_INIT:
      router_cancel = false;
      router_return_from_busy = -1;
      show_busy_screen();
      error_number = 0;
      position_valid = false;
      update_controls();
      setspeed(2);
      send_command_to_task(CMD_EMERGENCY_STOP, 0);
      gotoMaxBottomState = GOTO_MAX_BOTTOM_CHECK_BOTTOM_SWITCH;
      break;

    case GOTO_MAX_BOTTOM_CHECK_BOTTOM_SWITCH:
      if (check_endswitch_down()) {
        // at max down position, need to move up 20mm first
        send_command_to_task(CMD_MOVE, 20 * stepfactor);
        gotoMaxBottomState = GOTO_MAX_BOTTOM_WAIT_UP;
      } else {
        // not at bottom, proceed to move down
        send_command_to_task(CMD_MOVE, -1 * MAX_ROUTER_POS * stepfactor);
        gotoMaxBottomState = GOTO_MAX_BOTTOM_WAIT_DOWN;
      }
      break;

    case GOTO_MAX_BOTTOM_WAIT_UP:
      if (router_cancel) {
        invalidate_position();
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      } else if (check_endswitch_down()) {
        // switch released, move completed
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        gotoMaxBottomTimer = millis();
        gotoMaxBottomState = GOTO_MAX_BOTTOM_DEBOUNCE_UP;
      } else if (check_endswitch_up()) {
        // reached the top end switch
        invalidate_position();
        error_number = 2;
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      } else if (!is_motor_running()) {
        invalidate_position();
        error_number = 5;
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      }
      break;

    case GOTO_MAX_BOTTOM_DEBOUNCE_UP:
      if (millis() - gotoMaxBottomTimer >= 100) {
        gotoMaxBottomState = GOTO_MAX_BOTTOM_CHECK_BOTTOM_SWITCH;
      }
      break;

    case GOTO_MAX_BOTTOM_WAIT_DOWN:
      if (router_cancel) {
        invalidate_position();
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      } else if (check_endswitch_down()) {
        // Reached bottom switch
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        gotoMaxBottomTimer = millis();
        gotoMaxBottomState = GOTO_MAX_BOTTOM_DEBOUNCE_DOWN;
      } else if (!is_motor_running()) {
        invalidate_position();
        error_number = 6;
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      }
      break;

    case GOTO_MAX_BOTTOM_DEBOUNCE_DOWN:
      if (millis() - gotoMaxBottomTimer >= 100) {
        if (!check_endswitch_down()) {
          invalidate_position();
          error_number = 6;
          gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
        } else {
          // Move up 20mm to clear bottom sensor
          send_command_to_task(CMD_MOVE, 20 * stepfactor);
          gotoMaxBottomState = GOTO_MAX_BOTTOM_WAIT_UP_FINAL;
        }
      }
      break;

    case GOTO_MAX_BOTTOM_WAIT_UP_FINAL:
      if (router_cancel) {
        invalidate_position();
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      } else if (!check_endswitch_down()) {
        // switch released, move completed
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        gotoMaxBottomTimer = millis();
        gotoMaxBottomState = GOTO_MAX_BOTTOM_DEBOUNCE_FINAL;
      } else if (check_endswitch_up()) {
        // reached the top end switch
        invalidate_position();
        error_number = 2;
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      } else if (!is_motor_running()) {
        invalidate_position();
        error_number = 5;
        gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
      }
      break;

    case GOTO_MAX_BOTTOM_DEBOUNCE_FINAL:
      if (millis() - gotoMaxBottomTimer >= 100) {
        if (check_endswitch_down()) {
          invalidate_position();
          error_number = 5;
          gotoMaxBottomState = GOTO_MAX_BOTTOM_ERROR;
        }
        else {
          gotoMaxBottomState = GOTO_MAX_BOTTOM_COMPLETE;
        }
      }
      break;

    case GOTO_MAX_BOTTOM_COMPLETE:
      router_position = 0;
      lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
      update_position();
      gotoMaxBottomState = GOTO_MAX_BOTTOM_IDLE;
      update_controls();
      break;

    case GOTO_MAX_BOTTOM_ERROR:
      // Handle error state if needed
      gotoMaxBottomState = GOTO_MAX_BOTTOM_COMPLETE;
      break;
  }

  // general move state machine
  switch (moveStepperState) {
    case MOVE_STEPPER_IDLE:
      break;

    case MOVE_STEPPER_INIT:
      router_cancel = false;
      set_busy_symbol(true);
      moveStepperTargetPosition = newposition;
      update_controls();
      moveStepperState = MOVE_STEPPER_START;
      break;

    case MOVE_STEPPER_START:
      // convert to step position
      moveStepperStepCount = (moveStepperTargetPosition * stepfactor);
      // determine direction
      moveStepperDirectionUp = (moveStepperTargetPosition > get_current_position(false));
      //moveStepperDirectionUp = (moveStepperStepCount > get_current_position());
      // start absolute movement
      send_command_to_task(CMD_MOVE_TO, moveStepperStepCount);
      moveStepperState = MOVE_STEPPER_MOVING;
      break;

    case MOVE_STEPPER_MOVING:
      if (router_cancel) {
        invalidate_position();
        moveStepperState = MOVE_STEPPER_ERROR;
      } else if (moveStepperDirectionUp and check_endswitch_up()) {
        // hit top switch while moving up
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        send_command_to_task(CMD_SET_THIS_AS_CURRENT, 0);
        // move down to clear
        send_command_to_task(CMD_MOVE, -1 * (20 * stepfactor));
        moveStepperState = MOVE_STEPPER_RECOVER_UP;
      } else if (!moveStepperDirectionUp and check_endswitch_down()) {
        // hit bottom switch while moving down
        send_command_to_task(CMD_EMERGENCY_STOP, 0);
        send_command_to_task(CMD_SET_THIS_AS_CURRENT, 0);
        // move up to clear
        send_command_to_task(CMD_MOVE, 20 * stepfactor);
        moveStepperState = MOVE_STEPPER_RECOVER_DOWN;
      } else if (!is_motor_running()) {
        // movement completed normally
        moveStepperState = MOVE_STEPPER_COMPLETE;
      }
      break;

    case MOVE_STEPPER_RECOVER_UP:
      if (router_cancel) {
        invalidate_position();
        moveStepperState = MOVE_STEPPER_ERROR;
      } else if (!check_endswitch_up()) {
        invalidate_position();
        error_number = 2;
        moveStepperState = MOVE_STEPPER_ERROR;
      } else if (!is_motor_running()) {
        invalidate_position();
        // limit switch not cleared
        error_number = 1;
        moveStepperState = MOVE_STEPPER_ERROR;
      }
      break;

    case MOVE_STEPPER_RECOVER_DOWN:
      if (router_cancel) {
        invalidate_position();
        moveStepperState = MOVE_STEPPER_ERROR;
      } else if (!check_endswitch_down()) {
        invalidate_position();
        error_number = 2;
        moveStepperState = MOVE_STEPPER_ERROR;
      } else if (!is_motor_running()) {
        invalidate_position();
        // limit switch not cleared
        error_number = 5;
        moveStepperState = MOVE_STEPPER_ERROR;
      }
      break;

    case MOVE_STEPPER_COMPLETE:
      set_busy_symbol(false);
      moveStepperState = MOVE_STEPPER_IDLE;
      update_controls();
      // return to caller screen if applicable
      if (router_return_from_busy > -1) {
        lv_tabview_set_act(tabview, router_return_from_busy, LV_ANIM_OFF);
        router_return_from_busy = -1;
      }
      break;

    case MOVE_STEPPER_ERROR:
      router_return_from_busy = 0;
      router_position = 0;
      position_valid = false;
      moveStepperState = MOVE_STEPPER_COMPLETE;
      break;
  }

  // update screen, let lvgl do its thing
  if (millis() >= LVGL_Timer + 5) {
    lv_timer_handler();
    LVGL_Timer = millis();
  }
}


void check_actions_encoder1(void) {
  // check rotary encoders
  if (EncoderCounter1 != 0) {
    int dir = (EncoderCounter1 > 0) ? 1 : -1;
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_MAIN_REF:
        // main screen step size
        router_stepsize = constrain(router_stepsize + dir, 0, 6);
        EncoderCounter1 -= dir;
        update_stepsize();
        break;

      case TAB_PRESETS_REF:
        // select preset
        if (presets_selected == 5 and dir == 1 and preset_mode < 4) {
          presets_selected = 1;
          preset_mode = constrain(preset_mode + 1, 0, 4);
          presets_set_buttons_state();
          update_presets();
        } else if (presets_selected == 1 and dir == -1 and preset_mode > 0) {
          presets_selected = 5;
          preset_mode = constrain(preset_mode - 1, 0, 4);
          presets_set_buttons_state();
          update_presets();
        } else {
          presets_selected = constrain(presets_selected + dir, 0, 5);
        }
        EncoderCounter1 -= dir;
        presets_set_selected_buttons_state();
        break;

      case TAB_SEQUENCER_REF:
        // sequence up/down
        sequence_mode = constrain(sequence_mode + dir, 0, 4);
        EncoderCounter1 -= dir;
        sequence_step = 0;
        update_seq_step();
        sequencer_set_buttons_state();
        update_presets();
        break;

      default:
        // clear if other screens
        EncoderCounter1 = 0;
        break;
    }
  }
}

void check_actions_encoder2(void) {
  if (EncoderCounter2 != 0) {
    int dir = (EncoderCounter2 > 0) ? 1 : -1;
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_MAIN_REF:
        // main screen plunge speed
        router_speed = constrain(router_speed + dir, 0, 4);
        EncoderCounter2 -= dir;
        update_speed();
        break;

      case TAB_PRESETS_REF:
        // preset up/down
        preset_mode = constrain(preset_mode + dir, 0, 4);
        EncoderCounter2 -= dir;
        presets_selected = 0;
        presets_set_buttons_state();
        update_presets();
        presets_set_selected_buttons_state();
        break;

      default:
        // clear if other screens
        EncoderCounter2 = 0;
        break;
    }
  }
}

void check_actions_key1(void) {
  if (digitalRead(Encoder_1_Key) == 0) {
    timer_encoderbutton1 = millis();
    Encoder_Key1_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_1_Key) == 0) {
      if (millis() - timer_encoderbutton1 > 1000) {
        Encoder_Key1_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key1_Long_Press) {
      // long press
      while (digitalRead(Encoder_1_Key) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press
      switch (lv_tabview_get_tab_act(tabview)) {
        case TAB_MAIN_REF:
          goto_presets();
          break;

        case TAB_PRESETS_REF:
          if (presets_selected != 0) {
            goto_preset(presets_selected - 1);
          }
          break;
      }
    }
  }
}

void check_actions_key2(void) {
  if (digitalRead(Encoder_2_Key) == 0) {
    timer_encoderbutton2 = millis();
    Encoder_Key2_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_2_Key) == 0) {
      if (millis() - timer_encoderbutton2 > 1000) {
        Encoder_Key2_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key2_Long_Press) {
      // long press
      while (digitalRead(Encoder_2_Key) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press
      switch (lv_tabview_get_tab_act(tabview)) {
        case TAB_MAIN_REF:
          break;
      }
    }
  }
}

void check_buttons_and_move() {
  bool buttonUpPressed = (digitalRead(buttonup_pin) == 0);
  bool buttonDownPressed = (digitalRead(buttondown_pin) == 0);
  // ignore if both buttons are pressed
  if (buttonUpPressed && buttonDownPressed) {
    if (continuousMoveActive) {
      stop_continuous_move();
    }
    buttonUpWasPressed = false;
    buttonDownWasPressed = false;
    return;
  }
  if (lv_tabview_get_tab_act(tabview) == TAB_SEQUENCER_REF) {
    if (buttonUpPressed) {
      delay(1);
      while (digitalRead(buttonup_pin) == 0) {}
      handle_single_step(true);
    }
    if (buttonDownPressed) {
      delay(1);
      while (digitalRead(buttondown_pin) == 0) {}
      handle_single_step(false);
    }
  }
  else {
    // handle UP button
    if (buttonUpPressed) {
      if (!buttonUpWasPressed) {
        // first press - start timing and do single step
        buttonUpPressTime = millis();
        buttonUpWasPressed = true;
        handle_single_step(true);
      } else {
        // button is being held - check for continuous movement
        unsigned long holdTime = millis() - buttonUpPressTime;
        if (holdTime > 300 && !continuousMoveActive) {
          start_continuous_move(true);
        }
      }
    } else {
      // button released
      if (buttonUpWasPressed) {
        buttonUpWasPressed = false;
        if (continuousMoveActive && continuousMoveDirection == 1) {
          stop_continuous_move();
        }
      }
    }
    // handle DOWN button
    if (buttonDownPressed) {
      if (!buttonDownWasPressed) {
        // first press - start timing and do single step
        buttonDownPressTime = millis();
        buttonDownWasPressed = true;
        handle_single_step(false);
      } else {
        // button is being held - check for continuous movement
        unsigned long holdTime = millis() - buttonDownPressTime;
        if (holdTime > 300 && !continuousMoveActive) {
          start_continuous_move(false);
        }
      }
    } else {
      // button released
      if (buttonDownWasPressed) {
        buttonDownWasPressed = false;
        if (continuousMoveActive && continuousMoveDirection == -1) {
          stop_continuous_move();
        }
      }
    }
    // handle continuous movement if active
    if (continuousMoveActive) {
      handle_continuous_move();
    }
  }
}

void handle_single_step(bool isUpButton) {
  if ((lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF ||
       lv_tabview_get_tab_act(tabview) == TAB_MAINSUB_REF ||
       lv_tabview_get_tab_act(tabview) == TAB_ZOOM_REF) && position_valid) {
    float stepSize = get_stepsize();
    float targetPosition;
    if (isUpButton) {
      targetPosition = router_position + stepSize;
      if (targetPosition > MAX_ROUTER_POS) targetPosition = MAX_ROUTER_POS;
    } else {
      targetPosition = router_position - stepSize;
      if (targetPosition < (-1 * MAX_ROUTER_POS)) targetPosition = -1 * MAX_ROUTER_POS;
    }
    // move to the exact step position
    move_to_position(targetPosition);
  } else if (lv_tabview_get_tab_act(tabview) == TAB_SEQUENCER_REF) {
    // sequencer handling
    if (isUpButton) {
      sequence_step = 0;
      update_seq_step();
    } else {
      if (sequence_step < 5) {
        sequence_step = sequence_step + 1;
        sequence_go();
        update_seq_step();
      }
    }
  }
}

void start_continuous_move(bool isUpButton) {
  continuousMoveActive = true;
  continuousMoveDirection = isUpButton ? 1 : -1;
  lastContinuousStepTime = millis();
}

void stop_continuous_move() {
  if (continuousMoveActive) {
    continuousMoveActive = false;
    continuousMoveDirection = 0;
  }
}

void handle_continuous_move() {
  if (!continuousMoveActive) return;
  unsigned long currentTime = millis();
  // only proceed if motor is not running (wait for previous step to complete)
  if (!is_motor_running()) {
    if (currentTime - continuousMoveLastStep >= continuousMoveInterval) {
      continuousMoveLastStep = currentTime;
      float stepSize = get_stepsize();
      float targetPosition = router_position + (continuousMoveDirection * stepSize);
      // apply boundaries
      if (targetPosition > MAX_ROUTER_POS) {
        targetPosition = MAX_ROUTER_POS;
        stop_continuous_move();
        return;
      } else if (targetPosition < (-1 * MAX_ROUTER_POS)) {
        targetPosition = -1 * MAX_ROUTER_POS;
        stop_continuous_move();
        return;
      }
      // check endstops
      if ((continuousMoveDirection > 0 && check_endswitch_up()) ||
          (continuousMoveDirection < 0 && check_endswitch_down())) {
        stop_continuous_move();
        return;
      }
      // move to the exact step position
      move_to_position(targetPosition);
    }
  }
}

void move_to_position(float targetPosition) {
  // ensure position is valid
  if (targetPosition > MAX_ROUTER_POS) targetPosition = MAX_ROUTER_POS;
  if (targetPosition < (-1 * MAX_ROUTER_POS)) targetPosition = -1 * MAX_ROUTER_POS;
  router_position = targetPosition;
  setspeed(router_speed);
  newposition = router_position;
  moveStepperState = MOVE_STEPPER_INIT;
  update_position();
}

boolean check_endswitch_up()   {
  return digitalRead(endswitch_up)   == (LIMIT_NC ? 1 : 0);
}
boolean check_endswitch_down() {
  return digitalRead(endswitch_down) == (LIMIT_NC ? 1 : 0);
}

void invalidate_position(void) {
  send_command_to_task(CMD_EMERGENCY_STOP, 0);
  position_valid = false;
  router_position = 0;
  update_position();
}

// splash screen
void Display_Splash_Screen(void) {
  lv_obj_set_style_bg_color (lv_scr_act(), lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  static lv_style_t label_style1;
  lv_style_set_text_color(&label_style1, lv_color_white());
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_40);
  lv_obj_t *labelintro1 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro1, &label_style1, LV_PART_MAIN);
  lv_label_set_text(labelintro1, "Router lift");
  lv_obj_align(labelintro1, LV_ALIGN_TOP_MID, 0, 0);
  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_color_white());
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_22);
  lv_obj_t *labelintro2 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro2, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro2, "At boot time press:");
  lv_obj_align(labelintro2, LV_ALIGN_BOTTOM_MID, 0, -190);
  lv_obj_t *labelintro3 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro3, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro3, "Up or Down button to calibrate LCD");
  lv_obj_align(labelintro3, LV_ALIGN_BOTTOM_MID, 0, -130);
  lv_obj_t *labelintro4 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro4, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro4, "Up + Down button to reset settings");
  lv_obj_align(labelintro4, LV_ALIGN_BOTTOM_MID, 0, -70);
  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_color_white());
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_28);
  lv_obj_t *labelintro10 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro10, &label_style3, LV_PART_MAIN);
  lv_label_set_text(labelintro10, "(C) Jef Collin 2021-2025");
  lv_obj_align(labelintro10, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_refr_now(NULL);
}

void Setup_Screens(void) {
  lv_obj_t * label;
  // tabs
  tabview = lv_tabview_create(lv_scr_act());
  lv_tabview_set_tab_bar_size(tabview, 0);

  tabmain = lv_tabview_add_tab(tabview, "");
  tabmainsub = lv_tabview_add_tab(tabview, "");
  tabzero = lv_tabview_add_tab(tabview, "");
  tabpresets = lv_tabview_add_tab(tabview, "");
  tabsequencer = lv_tabview_add_tab(tabview, "");
  tabedit = lv_tabview_add_tab(tabview, "");
  tabcalc = lv_tabview_add_tab(tabview, "");
  tabwaitbusy = lv_tabview_add_tab(tabview, "");
  tabzoom = lv_tabview_add_tab(tabview, "");
  tabdirectentry = lv_tabview_add_tab(tabview, "");

  lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_clear_flag(tabmain, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabmainsub, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabzero, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabpresets, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabsequencer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabedit, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabcalc, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabwaitbusy, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabzoom, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabdirectentry, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_style_bg_color (tabmain, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabmainsub, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabzero, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabpresets, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabsequencer, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabedit, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabcalc, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabwaitbusy, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabzoom, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabdirectentry, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  lv_obj_set_style_bg_opa(tabmain, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabmainsub, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabzero, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabpresets, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabsequencer, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabedit, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabcalc, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabwaitbusy, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabzoom, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabdirectentry, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);

  // remove 16 pix padding
  lv_obj_set_style_pad_top(tabmainsub, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabmainsub, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabmainsub, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabmainsub, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabmain, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabzero, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabzero, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabzero, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabzero, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabpresets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabpresets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabpresets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabpresets, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabsequencer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabsequencer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabsequencer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabsequencer, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabedit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabedit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabedit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabedit, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabcalc, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabcalc, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabcalc, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabcalc, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabwaitbusy, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabwaitbusy, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabwaitbusy, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabwaitbusy, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabzoom, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabzoom, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabzoom, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabzoom, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabdirectentry, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabdirectentry, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabdirectentry, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabdirectentry, 0, LV_PART_MAIN);

  // styles

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_color_white());
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_22);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_color_black());
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_24);

  static lv_style_t label_style4;
  lv_style_set_text_color(&label_style4,  lv_color_white());
  lv_style_set_text_font(&label_style4, &lv_font_montserrat_28);

  static lv_style_t label_style5;
  lv_style_set_text_color(&label_style5, lv_color_white());
  lv_style_set_text_font(&label_style5, &lv_font_montserrat_40);

  static lv_style_t label_style7;
  lv_style_set_text_color(&label_style7, lv_color_white());
  lv_style_set_text_font(&label_style7, &lv_font_montserrat_24);

  static lv_style_t label_style8;
  lv_style_set_text_color(&label_style8,  lv_color_black());
  lv_style_set_text_font(&label_style8, &lv_font_montserrat_28);

  static lv_style_t btn_style1;
  lv_style_set_radius(&btn_style1, 5);
  lv_style_set_bg_color(&btn_style1, lv_color_white());

  lv_style_init(&btn_style1_selected);
  lv_style_set_bg_color(&btn_style1_selected, lv_color_hex(0x007AFF)); // Blue highlight

  static lv_style_t btn_style3;
  lv_style_set_radius(&btn_style3, 5);
  lv_style_set_text_font(&btn_style3, &lv_font_montserrat_40);

  static lv_style_t btn_style5;
  lv_style_set_radius(&btn_style5, 0);
  lv_style_set_text_font(&btn_style5,  &dseg_60);
  lv_style_set_text_color(&btn_style5, lv_color_white());
  lv_style_set_bg_color(&btn_style5, lv_color_black());
  lv_style_set_outline_opa(&btn_style5, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style5, LV_OPA_TRANSP);

  static lv_style_t btn_style6;
  lv_style_set_bg_color(&btn_style6, lv_color_hex(0x2F4F4F));
  lv_style_set_border_width(&btn_style6, 0);

  static lv_style_t btn_style7;
  lv_style_set_radius(&btn_style7,  0);
  lv_style_set_bg_color(&btn_style7,  lv_color_hex(0x2F4F4F));
  lv_style_set_border_width(&btn_style7,  0);
  lv_style_set_pad_left(&btn_style7, 2);  // minimal left padding

  static lv_style_t btn_style8;
  lv_style_set_radius(&btn_style8, 0);
  lv_style_set_text_font(&btn_style8,  &dseg_175);
  lv_style_set_text_color(&btn_style8, lv_color_white());
  lv_style_set_bg_color(&btn_style8, lv_color_black());
  lv_style_set_outline_opa(&btn_style8, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style8, LV_OPA_TRANSP);

  static lv_style_t ddlist_style1;
  lv_style_set_text_color(&ddlist_style1, lv_color_black());
  lv_style_set_text_font(&ddlist_style1, &lv_font_montserrat_22);

  // main screen

  btn_pos_small = lv_btn_create(tabmain);
  lv_obj_add_event_cb(btn_pos_small, event_zoom, LV_EVENT_ALL, NULL);
  lv_obj_set_width(btn_pos_small, 240);
  lv_obj_set_height(btn_pos_small, 64);
  lv_obj_add_style(btn_pos_small, &btn_style5, LV_PART_MAIN);

  btn_pos_small_label = lv_label_create(btn_pos_small);
  lv_label_set_text(btn_pos_small_label, "0.0");
  lv_obj_set_align(btn_pos_small_label,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(btn_pos_small, 0, 0);
  lv_obj_set_style_pad_right(btn_pos_small, 0, 0);

  lv_obj_align(btn_pos_small, LV_ALIGN_TOP_LEFT, 0, 32);

  lv_obj_t *lbl_unit1 = lv_label_create(tabmain);
  lv_obj_add_style(lbl_unit1, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_unit1, LV_ALIGN_TOP_LEFT, 255, 73);
  lv_label_set_text(lbl_unit1, "mm");

  btn_cancelmove = lv_btn_create(tabmain);
  lv_obj_add_style(btn_cancelmove, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_cancelmove, event_cancelmove, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_cancelmove);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Cancel");
  lv_obj_set_width(btn_cancelmove, 140);
  lv_obj_set_height(btn_cancelmove, 140);
  lv_obj_center(label);
  lv_obj_align(btn_cancelmove, LV_ALIGN_TOP_RIGHT, -5, 5);

  btn_gotozero = lv_btn_create(tabmain);
  lv_obj_add_style(btn_gotozero, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_gotozero, event_gotozero, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_gotozero);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Goto 0");
  lv_obj_set_width(btn_gotozero, 140);
  lv_obj_set_height(btn_gotozero, 55);
  lv_obj_center(label);
  lv_obj_align(btn_gotozero, LV_ALIGN_TOP_RIGHT, -5, 175);

  btn_nextmain = lv_btn_create(tabmain);
  lv_obj_add_style(btn_nextmain, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_nextmain, event_mainsubscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_nextmain);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_RIGHT);
  lv_obj_set_width(btn_nextmain, 140);
  lv_obj_set_height(btn_nextmain, 55);
  lv_obj_center(label);
  lv_obj_align(btn_nextmain, LV_ALIGN_TOP_RIGHT, -5, 260);

  lv_obj_t *lbl_step = lv_label_create(tabmain);
  lv_obj_add_style(lbl_step, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_step, LV_ALIGN_TOP_LEFT, 5, 145);
  lv_label_set_text(lbl_step, "Step size");

  lv_obj_t *lbl_speed = lv_label_create(tabmain);
  lv_obj_add_style(lbl_speed, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed, LV_ALIGN_TOP_LEFT, 170, 145);
  lv_label_set_text(lbl_speed, "Plunge rate");

  ddlist_stepsize = lv_dropdown_create(tabmain);
  lv_dropdown_set_options(ddlist_stepsize, "0.1 mm\n"
                          "0.2 mm\n"
                          "0.5 mm\n"
                          "1 mm\n"
                          "2 mm\n"
                          "5 mm\n"
                          "10 mm");
  lv_obj_add_style(ddlist_stepsize, &ddlist_style1, LV_PART_MAIN);

  lv_obj_t * ddlist_stepsize_list = lv_dropdown_get_list(ddlist_stepsize); /* Get list */
  lv_obj_add_style(ddlist_stepsize_list, &ddlist_style1, LV_PART_MAIN);      /* Add styles to list */
  lv_obj_set_width(ddlist_stepsize, 140);
  lv_obj_set_height(ddlist_stepsize, 55);
  lv_obj_align(ddlist_stepsize, LV_ALIGN_TOP_LEFT, 5, 175);
  lv_obj_add_event_cb(ddlist_stepsize, event_stepsizechange, LV_EVENT_ALL, NULL);

  ddlist_speed = lv_dropdown_create(tabmain);

  //  lv_dropdown_set_options(ddlist_speed, "Slowest\n"
  //                          "Slow\n"
  //                          "Medium\n"
  //                          "Fast\n"
  //                          "Fastest");

  lv_dropdown_set_options(ddlist_speed, "30 mm\n"
                          "60 mm\n"
                          "150 mm\n"
                          "225 mm\n"
                          "300 mm");

  lv_obj_add_style(ddlist_speed, &ddlist_style1, LV_PART_MAIN);
  lv_obj_t * ddlist_speed_list = lv_dropdown_get_list(ddlist_speed); /* Get list */
  lv_obj_add_style(ddlist_speed_list, &ddlist_style1, LV_PART_MAIN);      /* Add styles to list */
  lv_obj_set_width(ddlist_speed, 140);
  lv_obj_set_height(ddlist_speed, 55);
  lv_obj_align(ddlist_speed, LV_ALIGN_TOP_LEFT, 170, 175);
  lv_obj_add_event_cb(ddlist_speed, event_speedchange, LV_EVENT_ALL, NULL);

  btn_presets = lv_btn_create(tabmain);
  lv_obj_add_style(btn_presets, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets, event_presets, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Presets");
  lv_obj_set_width(btn_presets, 140);
  lv_obj_set_height(btn_presets, 55);
  lv_obj_center(label);
  lv_obj_align(btn_presets, LV_ALIGN_TOP_LEFT, 5, 260);

  btn_sequencer = lv_btn_create(tabmain);
  lv_obj_add_style(btn_sequencer, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_sequencer, event_sequence, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_sequencer);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Sequencer");
  lv_obj_set_width(btn_sequencer, 140);
  lv_obj_set_height(btn_sequencer, 55);
  lv_obj_center(label);
  lv_obj_align(btn_sequencer, LV_ALIGN_TOP_LEFT, 170, 260);

  // busy image
  LV_IMG_DECLARE(hg);
  img_busy_main = lv_img_create(tabmain); /*Crate an image object*/
  lv_img_set_src(img_busy_main, &hg);  /*Set the created file as image */
  lv_obj_set_pos(img_busy_main, 261, 5);      /*Set the positions*/

  // main sub screen
  btn_pos_small2 = lv_btn_create(tabmainsub);
  lv_obj_add_event_cb(btn_pos_small2, event_zoom, LV_EVENT_ALL, NULL);
  lv_obj_set_width(btn_pos_small2, 240);
  lv_obj_set_height(btn_pos_small2, 64);
  lv_obj_add_style(btn_pos_small2, &btn_style5, LV_PART_MAIN);

  btn_pos_small_label2 = lv_label_create(btn_pos_small2);
  lv_label_set_text(btn_pos_small_label2, "0.0");
  lv_obj_set_align(btn_pos_small_label2,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(btn_pos_small2, 0, 0);
  lv_obj_set_style_pad_right(btn_pos_small2, 0, 0);

  lv_obj_align(btn_pos_small2, LV_ALIGN_TOP_LEFT, 0, 32);

  lv_obj_t *lbl_unit2 = lv_label_create(tabmainsub);
  lv_obj_add_style(lbl_unit2, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_unit2, LV_ALIGN_TOP_LEFT, 255, 73);
  lv_label_set_text(lbl_unit2, "mm");

  btn_gomaxup = lv_btn_create(tabmainsub);
  lv_obj_add_style(btn_gomaxup, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_gomaxup, event_gotomaxtop, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_gomaxup);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Max " LV_SYMBOL_UP);
  lv_obj_set_width(btn_gomaxup, 140);
  lv_obj_set_height(btn_gomaxup, 55);
  lv_obj_center(label);
  lv_obj_align(btn_gomaxup, LV_ALIGN_TOP_LEFT, 5, 175);

  btn_gomaxdown = lv_btn_create(tabmainsub);
  lv_obj_add_style(btn_gomaxdown, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_gomaxdown, event_gotomaxbottom, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_gomaxdown);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Max " LV_SYMBOL_DOWN);
  lv_obj_set_width(btn_gomaxdown, 140);
  lv_obj_set_height(btn_gomaxdown, 55);
  lv_obj_center(label);
  lv_obj_align(btn_gomaxdown, LV_ALIGN_TOP_LEFT, 5, 260);

  btn_setzero = lv_btn_create(tabmainsub);
  lv_obj_add_style(btn_setzero, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_setzero, event_setzero, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_setzero);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Set 0");
  lv_obj_set_width(btn_setzero, 140);
  lv_obj_set_height(btn_setzero, 55);
  lv_obj_center(label);
  lv_obj_align(btn_setzero, LV_ALIGN_TOP_LEFT, 170, 175);

  btn_findzero = lv_btn_create(tabmainsub);
  lv_obj_add_style(btn_findzero, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_findzero, event_zeroscreen, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_findzero);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Find 0");
  lv_obj_set_width(btn_findzero, 140);
  lv_obj_set_height(btn_findzero, 55);
  lv_obj_center(label);
  lv_obj_align(btn_findzero, LV_ALIGN_TOP_LEFT, 170, 260);

  btn_calculator = lv_btn_create(tabmainsub);
  lv_obj_add_style(btn_calculator, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_calculator, event_calculator, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_calculator);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Calculator");
  lv_obj_set_width(btn_calculator, 140);
  lv_obj_set_height(btn_calculator, 55);
  lv_obj_center(label);
  lv_obj_align(btn_calculator, LV_ALIGN_TOP_RIGHT, -5, 175);

  lv_obj_t * btn218 = lv_btn_create(tabmainsub);
  lv_obj_add_style(btn218, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn218, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn218);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_set_width(btn218, 140);
  lv_obj_set_height(btn218, 55);
  lv_obj_center(label);
  lv_obj_align(btn218, LV_ALIGN_TOP_RIGHT, -5, 260);

  img_busy_mainsub = lv_img_create(tabmainsub);
  lv_img_set_src(img_busy_mainsub, &hg);
  lv_obj_set_pos(img_busy_mainsub, 261, 5);

  // zero screen

  btn_startzero = lv_btn_create(tabzero);
  lv_obj_add_style(btn_startzero, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_startzero, event_sensezero, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_startzero);
  lv_obj_set_width(btn_startzero, 400);
  lv_obj_set_height(btn_startzero, 120);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_obj_center(label);
  lv_label_set_text(label, "Connect sensor and touch here");
  lv_obj_align(btn_startzero, LV_ALIGN_CENTER, 0, -50);

  btn_cancelzero = lv_btn_create(tabzero);
  lv_obj_add_style(btn_cancelzero, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_cancelzero, event_cancelzero, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_cancelzero);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Cancel");
  lv_obj_set_width(btn_cancelzero, 140);
  lv_obj_set_height(btn_cancelzero, 55);
  lv_obj_center(label);
  lv_obj_align(btn_cancelzero, LV_ALIGN_CENTER, 0, 100);

  // presets screen

  lv_obj_t *lbl_presets = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_presets, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_presets, LV_ALIGN_TOP_LEFT, 0, 16);
  lv_label_set_text(lbl_presets, "Presets");

  lbl_pre_number = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_pre_number, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_pre_number, LV_ALIGN_TOP_LEFT, 85, 16);
  lv_label_set_text(lbl_pre_number, "-");

  lv_obj_t * btn45 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn45, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn45, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn45);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_set_width(btn45, 42);
  lv_obj_set_height(btn45, 42);
  lv_obj_align(btn45, LV_ALIGN_TOP_LEFT, 429, 9);

  btn_preset_previous = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_previous, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_previous, event_preset_previous, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_preset_previous);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_LEFT);
  lv_obj_center(label);
  lv_obj_set_width(btn_preset_previous, 42);
  lv_obj_set_height(btn_preset_previous, 42);
  lv_obj_align(btn_preset_previous, LV_ALIGN_TOP_LEFT, 329, 9);

  btn_preset_next = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_next, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_next, event_preset_next, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_preset_next);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_RIGHT);
  lv_obj_center(label);
  lv_obj_set_width(btn_preset_next, 42);
  lv_obj_set_height(btn_preset_next, 42);
  lv_obj_align(btn_preset_next, LV_ALIGN_TOP_LEFT, 379, 9);

  // preset number buttons
  btn_preset_0 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_0, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_0, event_presetgo1, LV_EVENT_CLICKED, NULL);
  lbl_pre_title1 = lv_label_create(btn_preset_0);
  lv_obj_add_style(lbl_pre_title1, &label_style8, LV_PART_MAIN);
  lv_label_set_text(lbl_pre_title1, "-");
  lv_obj_center(lbl_pre_title1);
  lv_obj_set_width(btn_preset_0, 78);
  lv_obj_set_height(btn_preset_0, 42);
  lv_obj_align(btn_preset_0, LV_ALIGN_TOP_LEFT, 0, 60);

  btn_preset_1 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_1, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_1, event_presetgo2, LV_EVENT_CLICKED, NULL);
  lbl_pre_title2 = lv_label_create(btn_preset_1);
  lv_obj_add_style(lbl_pre_title2, &label_style8, LV_PART_MAIN);
  lv_label_set_text(lbl_pre_title2, "-");
  lv_obj_center(lbl_pre_title2);
  lv_obj_set_width(btn_preset_1, 78);
  lv_obj_set_height(btn_preset_1, 42);
  lv_obj_align(btn_preset_1, LV_ALIGN_TOP_LEFT, 0, 111);

  btn_preset_2 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_2, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_2, event_presetgo3, LV_EVENT_CLICKED, NULL);
  lbl_pre_title3 = lv_label_create(btn_preset_2);
  lv_obj_add_style(lbl_pre_title3, &label_style8, LV_PART_MAIN);
  lv_label_set_text(lbl_pre_title3, "-");
  lv_obj_center(lbl_pre_title3);
  lv_obj_set_width(btn_preset_2, 78);
  lv_obj_set_height(btn_preset_2, 42);
  lv_obj_align(btn_preset_2, LV_ALIGN_TOP_LEFT, 0, 162);

  btn_preset_3 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_3, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_3, event_presetgo4, LV_EVENT_CLICKED, NULL);
  lbl_pre_title4 = lv_label_create(btn_preset_3);
  lv_obj_add_style(lbl_pre_title4, &label_style8, LV_PART_MAIN);
  lv_label_set_text(lbl_pre_title4, "-");
  lv_obj_center(lbl_pre_title4);
  lv_obj_set_width(btn_preset_3, 78);
  lv_obj_set_height(btn_preset_3, 42);
  lv_obj_align(btn_preset_3, LV_ALIGN_TOP_LEFT, 0, 213);

  btn_preset_4 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_preset_4, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_preset_4, event_presetgo5, LV_EVENT_CLICKED, NULL);
  lbl_pre_title5 = lv_label_create(btn_preset_4);
  lv_obj_add_style(lbl_pre_title5, &label_style8, LV_PART_MAIN);
  lv_label_set_text(lbl_pre_title5, "-");
  lv_obj_center(lbl_pre_title5);
  lv_obj_set_width(btn_preset_4, 78);
  lv_obj_set_height(btn_preset_4, 42);
  lv_obj_align(btn_preset_4, LV_ALIGN_TOP_LEFT, 0, 264);

  // position labels
  lbl_pre30 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_pre30, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre30, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre30, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre30, 100);
  lv_obj_align(lbl_pre30, LV_ALIGN_TOP_LEFT, 55, 65);

  lbl_pre31 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_pre31, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre31, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre31, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre31, 100);
  lv_obj_align(lbl_pre31, LV_ALIGN_TOP_LEFT, 55, 116);

  lbl_pre32 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_pre32, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre32, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre32, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre32, 100);
  lv_obj_align(lbl_pre32, LV_ALIGN_TOP_LEFT, 55, 167);

  lbl_pre33 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_pre33, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre33, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre33, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre33, 100);
  lv_obj_align(lbl_pre33, LV_ALIGN_TOP_LEFT, 55, 218);

  lbl_pre34 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_pre34, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre34, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre34, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre34, 100);
  lv_obj_align(lbl_pre34, LV_ALIGN_TOP_LEFT, 55, 269);

  // speed labels
  lbl_speed30 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_speed30, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed30, LV_ALIGN_TOP_LEFT, 164, 69);

  lbl_speed31 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_speed31, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed31, LV_ALIGN_TOP_LEFT, 164, 120);

  lbl_speed32 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_speed32, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed32, LV_ALIGN_TOP_LEFT, 164, 171);

  lbl_speed33 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_speed33, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed33, LV_ALIGN_TOP_LEFT, 164, 222);

  lbl_speed34 = lv_label_create(tabpresets);
  lv_obj_add_style(lbl_speed34, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed34, LV_ALIGN_TOP_LEFT, 164, 273);

  // text and edit buttons
  lv_obj_t * btn50 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn50, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn50, event_presetedit1, LV_EVENT_CLICKED, NULL);

  lbl_tag30 = lv_label_create(btn50);
  lv_obj_add_style(lbl_tag30, &label_style2, LV_PART_MAIN);

  lv_obj_set_width(btn50, 171);
  lv_obj_set_height(btn50, 40);
  lv_obj_align(btn50, LV_ALIGN_TOP_LEFT, 200, 61);

  lv_obj_t * btn51 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn51, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn51, event_presetedit2, LV_EVENT_CLICKED, NULL);

  lbl_tag31 = lv_label_create(btn51);
  lv_obj_add_style(lbl_tag31, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn51, 171);
  lv_obj_set_height(btn51, 40);
  lv_obj_align(btn51, LV_ALIGN_TOP_LEFT, 200, 112);

  lv_obj_t * btn52 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn52, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn52, event_presetedit3, LV_EVENT_CLICKED, NULL);

  lbl_tag32 = lv_label_create(btn52);
  lv_obj_add_style(lbl_tag32, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn52, 171);
  lv_obj_set_height(btn52, 40);
  lv_obj_align(btn52, LV_ALIGN_TOP_LEFT, 200, 163);

  lv_obj_t * btn53 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn53, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn53, event_presetedit4, LV_EVENT_CLICKED, NULL);

  lbl_tag33 = lv_label_create(btn53);
  lv_obj_add_style(lbl_tag33, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn53, 171);
  lv_obj_set_height(btn53, 40);
  lv_obj_align(btn53, LV_ALIGN_TOP_LEFT, 200, 214);

  lv_obj_t * btn54 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn54, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn54, event_presetedit5, LV_EVENT_CLICKED, NULL);

  lbl_tag34 = lv_label_create(btn54);
  lv_obj_add_style(lbl_tag34, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn54, 171);
  lv_obj_set_height(btn54, 40);
  lv_obj_align(btn54, LV_ALIGN_TOP_LEFT, 200, 265);

  // delete buttons
  lv_obj_t * btn31 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn31, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn31, event_presetdel1, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn31);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn31, 42);
  lv_obj_set_height(btn31, 42);
  lv_obj_align(btn31, LV_ALIGN_TOP_LEFT, 379, 60);

  lv_obj_t * btn33 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn33, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn33, event_presetdel2, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn33);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn33, 42);
  lv_obj_set_height(btn33, 42);
  lv_obj_align(btn33, LV_ALIGN_TOP_LEFT, 379, 111);

  lv_obj_t * btn35 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn35, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn35, event_presetdel3, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn35);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn35, 42);
  lv_obj_set_height(btn35, 42);
  lv_obj_align(btn35, LV_ALIGN_TOP_LEFT, 379, 162);

  lv_obj_t * btn37 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn37, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn37, event_presetdel4, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn37);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn37, 42);
  lv_obj_set_height(btn37, 42);
  lv_obj_align(btn37, LV_ALIGN_TOP_LEFT, 379, 213);

  lv_obj_t * btn39 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn39, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn39, event_presetdel5, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn39);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn39, 42);
  lv_obj_set_height(btn39, 42);
  lv_obj_align(btn39, LV_ALIGN_TOP_LEFT, 379, 264);

  // save buttons
  lv_obj_t * btn30 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn30, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn30, event_presetsave1, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn30);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn30, 42);
  lv_obj_set_height(btn30, 42);
  lv_obj_align(btn30, LV_ALIGN_TOP_LEFT, 429, 60);

  lv_obj_t * btn32 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn32, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn32, event_presetsave2, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn32);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn32, 42);
  lv_obj_set_height(btn32, 42);
  lv_obj_align(btn32, LV_ALIGN_TOP_LEFT, 429, 111);

  lv_obj_t * btn34 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn34, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn34, event_presetsave3, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn34);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn34, 42);
  lv_obj_set_height(btn34, 42);
  lv_obj_align(btn34, LV_ALIGN_TOP_LEFT, 429, 162);

  lv_obj_t * btn36 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn36, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn36, event_presetsave4, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn36);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn36, 42);
  lv_obj_set_height(btn36, 42);
  lv_obj_align(btn36, LV_ALIGN_TOP_LEFT, 429, 213);

  lv_obj_t * btn38 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn38, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn38, event_presetsave5, LV_EVENT_CLICKED, NULL);

  label = lv_label_create(btn38);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn38, 42);
  lv_obj_set_height(btn38, 42);
  lv_obj_align(btn38, LV_ALIGN_TOP_LEFT, 429, 264);

  // sequencer screen 1

  lv_obj_t *lbl_seq1 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_seq1, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_seq1, LV_ALIGN_TOP_LEFT, 0, 16);
  lv_label_set_text(lbl_seq1, "Sequence");

  lbl_seq_number = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_seq_number, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_seq_number, LV_ALIGN_TOP_LEFT, 117, 16);
  lv_label_set_text(lbl_seq_number, "-");

  lv_obj_t * btn160 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn160, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn160, event_seqedit_title, LV_EVENT_CLICKED, NULL);
  lbl_seqtitle100 = lv_label_create(btn160);
  lv_obj_add_style(lbl_seqtitle100, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn160, 175);
  lv_obj_set_height(btn160, 40);
  lv_obj_align(btn160, LV_ALIGN_TOP_LEFT, 146, 10);

  lv_obj_t * btn145 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn145, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn145, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn145);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_set_width(btn145, 42);
  lv_obj_set_height(btn145, 42);
  lv_obj_align(btn145, LV_ALIGN_TOP_LEFT, 429, 9);

  btn_sequencer_previous = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn_sequencer_previous, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_sequencer_previous, event_seq_previous, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_sequencer_previous);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_LEFT);
  lv_obj_center(label);
  lv_obj_set_width(btn_sequencer_previous, 42);
  lv_obj_set_height(btn_sequencer_previous, 42);
  lv_obj_align(btn_sequencer_previous, LV_ALIGN_TOP_LEFT, 329, 9);

  btn_sequencer_next = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn_sequencer_next, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_sequencer_next, event_seq_next, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_sequencer_next);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_RIGHT);
  lv_obj_center(label);
  lv_obj_set_width(btn_sequencer_next, 42);
  lv_obj_set_height(btn_sequencer_next, 42);
  lv_obj_align(btn_sequencer_next, LV_ALIGN_TOP_LEFT, 379, 9);

  // play or checkmark icons
  lbl_mark100 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_mark100, &label_style4, LV_PART_MAIN);
  lv_obj_align(lbl_mark100, LV_ALIGN_TOP_LEFT, 10, 61);

  lbl_mark101 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_mark101, &label_style4, LV_PART_MAIN);
  lv_obj_align(lbl_mark101, LV_ALIGN_TOP_LEFT, 10, 112);

  lbl_mark102 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_mark102, &label_style4, LV_PART_MAIN);
  lv_obj_align(lbl_mark102, LV_ALIGN_TOP_LEFT, 10, 163);

  lbl_mark103 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_mark103, &label_style4, LV_PART_MAIN);
  lv_obj_align(lbl_mark103, LV_ALIGN_TOP_LEFT, 10, 214);

  lbl_mark104 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_mark104, &label_style4, LV_PART_MAIN);
  lv_obj_align(lbl_mark104, LV_ALIGN_TOP_LEFT, 10, 265);

  // position labels
  lbl_pre100 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_pre100, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre100, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre100, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre100, 100);
  lv_obj_align(lbl_pre100, LV_ALIGN_TOP_LEFT, 30, 65);

  lbl_pre101 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_pre101, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre101, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre101, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre101, 100);
  lv_obj_align(lbl_pre101, LV_ALIGN_TOP_LEFT, 30, 116);

  lbl_pre102 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_pre102, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre102, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre102, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre102, 100);
  lv_obj_align(lbl_pre102, LV_ALIGN_TOP_LEFT, 30, 167);

  lbl_pre103 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_pre103, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre103, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre103, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre103, 100);
  lv_obj_align(lbl_pre103, LV_ALIGN_TOP_LEFT, 30, 218);

  lbl_pre104 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_pre104, &label_style7, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_pre104, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(lbl_pre104, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_pre104, 100);
  lv_obj_align(lbl_pre104, LV_ALIGN_TOP_LEFT, 30, 269);

  // speed labels
  lbl_speed100 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_speed100, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed100, LV_ALIGN_TOP_LEFT, 139, 69);

  lbl_speed101 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_speed101, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed101, LV_ALIGN_TOP_LEFT, 139, 120);

  lbl_speed102 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_speed102, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed102, LV_ALIGN_TOP_LEFT, 139, 171);

  lbl_speed103 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_speed103, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed103, LV_ALIGN_TOP_LEFT, 139, 222);

  lbl_speed104 = lv_label_create(tabsequencer);
  lv_obj_add_style(lbl_speed104, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_speed104, LV_ALIGN_TOP_LEFT, 139, 273);

  // text and edit buttons
  lv_obj_t * btn150 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn150, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn150, event_seqedit1, LV_EVENT_CLICKED, NULL);
  lbl_tag100 = lv_label_create(btn150);
  lv_obj_add_style(lbl_tag100, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn150, 196);
  lv_obj_set_height(btn150, 40);
  lv_obj_align(btn150, LV_ALIGN_TOP_LEFT, 175, 61);

  lv_obj_t * btn151 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn151, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn151, event_seqedit2, LV_EVENT_CLICKED, NULL);
  lbl_tag101 = lv_label_create(btn151);
  lv_obj_add_style(lbl_tag101, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn151, 196);
  lv_obj_set_height(btn151, 40);
  lv_obj_align(btn151, LV_ALIGN_TOP_LEFT, 175, 112);

  lv_obj_t * btn152 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn152, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn152, event_seqedit3, LV_EVENT_CLICKED, NULL);
  lbl_tag102 = lv_label_create(btn152);
  lv_obj_add_style(lbl_tag102, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn152, 196);
  lv_obj_set_height(btn152, 40);
  lv_obj_align(btn152, LV_ALIGN_TOP_LEFT, 175, 163);

  lv_obj_t * btn153 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn153, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn153, event_seqedit4, LV_EVENT_CLICKED, NULL);
  lbl_tag103 = lv_label_create(btn153);
  lv_obj_add_style(lbl_tag103, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn153, 196);
  lv_obj_set_height(btn153, 40);
  lv_obj_align(btn153, LV_ALIGN_TOP_LEFT, 175, 214);

  lv_obj_t * btn154 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn154, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn154, event_seqedit5, LV_EVENT_CLICKED, NULL);
  lbl_tag104 = lv_label_create(btn154);
  lv_obj_add_style(lbl_tag104, &label_style2, LV_PART_MAIN);
  lv_obj_set_width(btn154, 196);
  lv_obj_set_height(btn154, 40);
  lv_obj_align(btn154, LV_ALIGN_TOP_LEFT, 175, 265);

  // delete buttons
  lv_obj_t * btn131 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn131, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn131, event_seqdel1, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn131);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn131, 42);
  lv_obj_set_height(btn131, 42);
  lv_obj_align(btn131, LV_ALIGN_TOP_LEFT, 379, 60);

  lv_obj_t * btn133 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn133, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn133,  event_seqdel2, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn133);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn133, 42);
  lv_obj_set_height(btn133, 42);
  lv_obj_align(btn133, LV_ALIGN_TOP_LEFT, 379, 111);

  lv_obj_t * btn135 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn135, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn135,  event_seqdel3, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn135);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn135, 42);
  lv_obj_set_height(btn135, 42);
  lv_obj_align(btn135, LV_ALIGN_TOP_LEFT, 379, 162);

  lv_obj_t * btn137 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn137, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn137,  event_seqdel4, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn137);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn137, 42);
  lv_obj_set_height(btn137, 42);
  lv_obj_align(btn137, LV_ALIGN_TOP_LEFT, 379, 213);

  lv_obj_t * btn139 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn139, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn139,  event_seqdel5, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn139);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_set_width(btn139, 42);
  lv_obj_set_height(btn139, 42);
  lv_obj_align(btn139, LV_ALIGN_TOP_LEFT, 379, 264);

  lv_obj_t * btn130 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn130, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn130,  event_seqsave1, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn130);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn130, 42);
  lv_obj_set_height(btn130, 42);
  lv_obj_align(btn130, LV_ALIGN_TOP_LEFT, 429, 60);

  lv_obj_t * btn132 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn132, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn132,  event_seqsave2, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn132);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn132, 42);
  lv_obj_set_height(btn132, 42);
  lv_obj_align(btn132, LV_ALIGN_TOP_LEFT, 429, 111);

  lv_obj_t * btn134 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn134, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn134,  event_seqsave3, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn134);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn134, 42);
  lv_obj_set_height(btn134, 42);
  lv_obj_align(btn134, LV_ALIGN_TOP_LEFT, 429, 162);

  lv_obj_t * btn136 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn136, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn136,  event_seqsave4, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn136);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn136, 42);
  lv_obj_set_height(btn136, 42);
  lv_obj_align(btn136, LV_ALIGN_TOP_LEFT, 429, 213);

  lv_obj_t * btn138 = lv_btn_create(tabsequencer);
  lv_obj_add_style(btn138, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn138,  event_seqsave5, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn138);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_set_width(btn138, 42);
  lv_obj_set_height(btn138, 42);
  lv_obj_align(btn138, LV_ALIGN_TOP_LEFT, 429, 264);

  // edit screen

  txtentry  = lv_textarea_create(tabedit);
  lv_obj_add_style(txtentry, &label_style3, LV_PART_MAIN);
  lv_textarea_set_align(txtentry, LV_TEXT_ALIGN_LEFT);
  lv_textarea_set_max_length(txtentry, 15);
  lv_textarea_set_one_line(txtentry, true);
  lv_obj_set_width(txtentry, 300);
  lv_obj_set_height(txtentry, 45);
  lv_obj_align(txtentry, LV_ALIGN_TOP_MID, 00, 10);
  keyboard = lv_keyboard_create(tabedit);
  lv_keyboard_set_textarea(keyboard, txtentry);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 00, 100);
  lv_obj_add_event_cb(keyboard,  event_keyboardhandler, LV_EVENT_ALL, NULL);

  // calculator screen

  lbl_calc1 = lv_label_create(tabcalc);
  lv_obj_add_style(lbl_calc1, &label_style5, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_calc1, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_calc1, 300);
  lv_obj_set_style_text_align(lbl_calc1 , LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(lbl_calc1, LV_ALIGN_TOP_LEFT, 80, 5);

  lbl_calc2 = lv_label_create(tabcalc);
  lv_obj_add_style(lbl_calc2, &label_style2, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_calc2, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_calc2, 300);
  lv_obj_set_style_text_align(lbl_calc2 , LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(lbl_calc2, LV_ALIGN_TOP_LEFT, 60, 48);

  lbl_calcops = lv_label_create(tabcalc);
  lv_obj_add_style(lbl_calcops, &label_style2, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_calcops, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_calcops, 300);
  lv_obj_set_style_text_align(lbl_calcops , LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(lbl_calcops, LV_ALIGN_TOP_LEFT, 80, 48);

  calc_mtrx1 = lv_btnmatrix_create(tabcalc);
  lv_btnmatrix_set_map(calc_mtrx1, btnm_calc);
  lv_obj_add_style(calc_mtrx1, &btn_style3, LV_PART_ITEMS);
  lv_obj_add_style(calc_mtrx1, &btn_style6, LV_PART_MAIN);
  lv_obj_set_height(calc_mtrx1, 230);
  lv_obj_set_width(calc_mtrx1, 460);
  lv_obj_align(calc_mtrx1, LV_ALIGN_CENTER, 0, 35);
  lv_obj_add_event_cb(calc_mtrx1, event_calckey, LV_EVENT_ALL, NULL);

  lv_obj_t * btn600 = lv_btn_create(tabcalc);
  lv_obj_add_style(btn600, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn600, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn600);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_set_width(btn600, 42);
  lv_obj_set_height(btn600, 42);
  lv_obj_center(label);
  lv_obj_align(btn600, LV_ALIGN_TOP_LEFT, 429, 9);

  btn_gotocalc = lv_btn_create(tabcalc);
  lv_obj_add_style(btn_gotocalc, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_gotocalc, event_gotocalc, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_gotocalc);
  lv_obj_add_style(label, &label_style8, LV_PART_MAIN);
  lv_label_set_text(label, "Goto");
  lv_obj_set_width(btn_gotocalc, 84);
  lv_obj_set_height(btn_gotocalc, 42);
  lv_obj_center(label);
  lv_obj_align(btn_gotocalc, LV_ALIGN_TOP_LEFT, 9, 9);

  // wait screen

  lv_obj_t * img_busy_wait = lv_img_create(tabwaitbusy); /*Crate an image object*/
  lv_img_set_src(img_busy_wait, &hg);  /*Set the created file as image */
  lv_obj_set_pos(img_busy_wait, 219, 10);      /*Set the positions*/

  btn_busy_cancel = lv_btn_create(tabwaitbusy);
  lv_obj_add_style(btn_busy_cancel, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_busy_cancel, event_cancel, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_busy_cancel);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Cancel");
  lv_obj_set_width(btn_busy_cancel, 180);
  lv_obj_set_height(btn_busy_cancel, 180);
  lv_obj_center(label);
  lv_obj_align(btn_busy_cancel, LV_ALIGN_TOP_MID, 0, 120);

  // zoom screen

  btn_pos_large = lv_btn_create(tabzoom);
  lv_obj_add_event_cb(btn_pos_large, event_zoomreturn, LV_EVENT_CLICKED, NULL);
  lv_obj_set_width(btn_pos_large, 480);
  lv_obj_set_height(btn_pos_large, 320);
  lv_obj_add_style(btn_pos_large, &btn_style8, LV_PART_MAIN);

  btn_pos_large_label = lv_label_create(btn_pos_large);
  lv_label_set_text(btn_pos_large_label, "0.0");
  lv_obj_set_align(btn_pos_large_label,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(btn_pos_large_label, 0, 0);
  lv_obj_set_style_pad_right(btn_pos_large_label, 0, 0);

  lv_obj_align(btn_pos_large, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *lbl_unitlarge = lv_label_create(tabzoom);
  lv_obj_add_style(lbl_unitlarge, &label_style4, LV_PART_MAIN);
  lv_obj_align(lbl_unitlarge, LV_ALIGN_TOP_RIGHT, -30, 250);
  lv_label_set_text(lbl_unitlarge, "mm");

  img_busy_zoom = lv_img_create(tabzoom); /*Crate an image object*/
  lv_img_set_src(img_busy_zoom, &hg);  /*Set the created file as image */
  lv_obj_set_pos(img_busy_zoom, 218, 5);      /*Set the positions*/

  // direct entry

  lbl_directpos = lv_label_create(tabdirectentry);
  lv_obj_add_style(lbl_directpos, &label_style5, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_directpos, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_directpos, 150);
  lv_obj_set_style_text_align(lbl_directpos , LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(lbl_directpos, LV_ALIGN_TOP_LEFT, 140, 5);

  mtrx_directentry = lv_btnmatrix_create(tabdirectentry);
  lv_btnmatrix_set_map(mtrx_directentry, btnm_directentry);
  lv_obj_add_style(mtrx_directentry, &btn_style3, LV_PART_ITEMS);
  lv_obj_add_style(mtrx_directentry, &btn_style6, LV_PART_MAIN);
  lv_obj_set_height(mtrx_directentry, 250);
  lv_obj_set_width(mtrx_directentry, 300);
  lv_obj_align(mtrx_directentry, LV_ALIGN_TOP_LEFT, 5, 68);
  lv_obj_add_event_cb(mtrx_directentry, event_directkey, LV_EVENT_ALL, NULL);

  btn_directgoto = lv_btn_create(tabdirectentry);
  lv_obj_add_style(btn_directgoto, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_directgoto, event_gotodirect, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_directgoto);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Goto");
  lv_obj_set_width(btn_directgoto, 140);
  lv_obj_set_height(btn_directgoto, 55);
  lv_obj_center(label);
  lv_obj_align(btn_directgoto, LV_ALIGN_TOP_RIGHT, -5, 175);

  btn_directcancel = lv_btn_create(tabdirectentry);
  lv_obj_add_style(btn_directcancel, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_directcancel, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_directcancel);
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_label_set_text(label, "Cancel");
  lv_obj_set_width(btn_directcancel, 140);
  lv_obj_set_height(btn_directcancel, 55);
  lv_obj_center(label);
  lv_obj_align(btn_directcancel, LV_ALIGN_TOP_RIGHT, -5, 260);

}


static void event_mainscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
    update_position();
  }
}

static void event_mainsubscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      // goto main sub screen 1
      lv_tabview_set_act(tabview, TAB_MAINSUB_REF, LV_ANIM_OFF);
      set_busy_symbol(false);
      update_position();
    }
  }
}

static void event_presets(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    goto_presets();
  }
}

static void event_sequence(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      // goto seq screen 1
      lv_tabview_set_act(tabview, TAB_SEQUENCER_REF, LV_ANIM_OFF);
      sequence_mode = 0;
      sequence_step = 0;
      update_seq_step();
      sequencer_set_buttons_state();
      update_presets();
    }
  }
}

static void event_gotozero(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (position_valid == true) {
      if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
        router_position = 0;
        setspeed(2);
        newposition = router_position;
        moveStepperState = MOVE_STEPPER_INIT;
        //  move_stepper(router_position);
        update_position();
        router_old_position = 0;
      }
    }
  }
}

static void event_gotomaxtop(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      gotoMaxTopState = GOTO_MAX_TOP_INIT;
    }
  }
}

static void event_gotomaxbottom(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      gotoMaxBottomState = GOTO_MAX_BOTTOM_INIT;
    }
  }
}

static void event_setzero(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      router_position = 0;
      send_command_to_task(CMD_EMERGENCY_STOP, 0);
      router_old_position = 0;
      position_valid = true;
      update_position();
      update_controls();

    }
  }
}

static void event_zeroscreen(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      lv_tabview_set_act(tabview, TAB_ZERO_REF, LV_ANIM_OFF);
    }
  }
}

void event_sensezero(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    senseZeroState = SENSE_ZERO_INIT;
  }
}

static void event_zoom(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  static bool long_press_handled = false;
  // Reset flag at the start of a new press sequence
  if (event == LV_EVENT_PRESSED) {
    long_press_handled = false;
  }
  if (event == LV_EVENT_LONG_PRESSED) {
    long_press_handled = true;
    if (position_valid) {
      direct_start();
    }
  } else if (event == LV_EVENT_CLICKED) {
    if (long_press_handled) {
      return;
    }
    zoomreturnscreen = lv_tabview_get_tab_act(tabview);
    lv_tabview_set_act(tabview, TAB_ZOOM_REF, LV_ANIM_OFF);
    set_busy_symbol(false);
    update_position();
  }
}

static void event_zoomreturn(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, zoomreturnscreen, LV_ANIM_OFF);
    update_position();
  }
}

static void event_cancel(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    router_cancel = true;
  }
}

static void event_cancelzero(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  }
}

static void event_cancelmove(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    router_cancel = true;
  }
}

static void event_preset_next(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    preset_mode = preset_mode + 1;
    if (preset_mode > 4) {
      preset_mode = 4;
    }
    presets_set_buttons_state();
    update_presets();
  }
}

static void event_preset_previous(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (preset_mode > 0) {
      preset_mode = preset_mode - 1;
    }
    presets_set_buttons_state();
    update_presets();
  }
}

// enable or disable next/prev buttons
void presets_set_buttons_state(void) {
  // disable if last screen
  if (preset_mode == 4) {
    lv_obj_add_state(btn_preset_next, LV_STATE_DISABLED);
  }
  else {
    lv_obj_remove_state(btn_preset_next, LV_STATE_DISABLED);
  }
  // disable if last screen
  if (preset_mode == 0) {
    lv_obj_add_state(btn_preset_previous, LV_STATE_DISABLED);
  }
  else {
    lv_obj_remove_state(btn_preset_previous, LV_STATE_DISABLED);
  }
}

static void event_presetedit1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(1, 3,  lbl_tag30);
  }
}

static void event_presetedit2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(2, 3,  lbl_tag31);
  }
}

static void event_presetedit3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(3, 3,  lbl_tag32);
  }
}

static void event_presetedit4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(4, 3,  lbl_tag33);
  }
}

static void event_presetedit5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(5, 3,  lbl_tag34);
  }
}

void preset_delete(uint8_t line_no) {
  // calculate index, preset_mode = screen number
  uint8_t array_index = (preset_mode * 5) + line_no;
  P_S_settings[array_index].setting_position = 0;
  P_S_settings[array_index].setting_speed = 0;
  P_S_settings[array_index].setting_info = PRESET_DEFAULT;
  save_update_presets();
}

static void event_presetdel1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    preset_delete(0);
  }
}

static void event_presetdel2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    preset_delete(1);
  }
}

static void event_presetdel3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    preset_delete(2);
  }
}

static void event_presetdel4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    preset_delete(3);
  }
}

static void event_presetdel5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    preset_delete(4);
  }
}

void save_preset(uint8_t line_no) {
  // calculate index, preset_mode = screen number
  uint8_t array_index = (preset_mode * 5) + line_no;
  P_S_settings[array_index].setting_position = router_position;
  P_S_settings[array_index].setting_speed = router_speed;
  save_update_presets();
}

static void event_presetsave1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_preset(0);
  }
}

static void event_presetsave2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_preset(1);
  }
}

static void event_presetsave3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_preset(2);
  }
}

static void event_presetsave4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_preset(3);
  }
}

static void event_presetsave5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_preset(4);
  }
}

static void event_presetgo1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    goto_preset(0);
  }
}

static void event_presetgo2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    goto_preset(1);
  }
}

static void event_presetgo3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    goto_preset(2);
  }
}

static void event_presetgo4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    goto_preset(3);
  }
}

static void event_presetgo5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    goto_preset(4);
  }
}

static void goto_preset(uint8_t line_no) {
  // calculate index, preset_mode = screen number
  uint8_t array_index = (preset_mode * 5) + line_no;
  setspeed(P_S_settings[array_index].setting_speed);
  router_position = P_S_settings[array_index].setting_position;
  newposition = router_position;
  moveStepperState = MOVE_STEPPER_INIT;
  // move_stepper(router_position);
  lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  update_position();
  update_speed();
}

static void save_update_presets(void) {
  save_settings();
  update_presets();
}

void goto_presets(void) {
  if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
    // goto preset screen 1
    lv_tabview_set_act(tabview, TAB_PRESETS_REF, LV_ANIM_OFF);
    preset_mode = 0;
    presets_set_buttons_state();
    update_presets();
    presets_selected = 0;
    presets_set_selected_buttons_state();
  }
}

void update_presets(void) {
  switch (lv_tabview_get_tab_act(tabview)) {
    case TAB_PRESETS_REF:
      switch (preset_mode) {
        case 0:
          convert_position(P_S_settings[0].setting_position);
          lv_label_set_text(lbl_pre30, printbuf);
          convert_position(P_S_settings[1].setting_position);
          lv_label_set_text(lbl_pre31, printbuf);
          convert_position(P_S_settings[2].setting_position);
          lv_label_set_text(lbl_pre32, printbuf);
          convert_position(P_S_settings[3].setting_position);
          lv_label_set_text(lbl_pre33, printbuf);
          convert_position(P_S_settings[4].setting_position);
          lv_label_set_text(lbl_pre34, printbuf);

          convert_speed(P_S_settings[0].setting_speed);
          lv_label_set_text(lbl_speed30, printbuf);
          convert_speed(P_S_settings[1].setting_speed);
          lv_label_set_text(lbl_speed31, printbuf);
          convert_speed(P_S_settings[2].setting_speed);
          lv_label_set_text(lbl_speed32, printbuf);
          convert_speed(P_S_settings[3].setting_speed);
          lv_label_set_text(lbl_speed33, printbuf);
          convert_speed(P_S_settings[4].setting_speed);
          lv_label_set_text(lbl_speed34, printbuf);

          P_S_settings[0].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag30, printbuf);
          P_S_settings[1].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag31, printbuf);
          P_S_settings[2].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag32, printbuf);
          P_S_settings[3].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag33, printbuf);
          P_S_settings[4].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag34, printbuf);

          lv_label_set_text(lbl_pre_title1, "1");
          lv_label_set_text(lbl_pre_title2, "2");
          lv_label_set_text(lbl_pre_title3, "3");
          lv_label_set_text(lbl_pre_title4, "4");
          lv_label_set_text(lbl_pre_title5, "5");
          lv_label_set_text(lbl_pre_number, "1-5");
          break;

        case 1:
          convert_position(P_S_settings[5].setting_position);
          lv_label_set_text(lbl_pre30, printbuf);
          convert_position(P_S_settings[6].setting_position);
          lv_label_set_text(lbl_pre31, printbuf);
          convert_position(P_S_settings[7].setting_position);
          lv_label_set_text(lbl_pre32, printbuf);
          convert_position(P_S_settings[8].setting_position);
          lv_label_set_text(lbl_pre33, printbuf);
          convert_position(P_S_settings[9].setting_position);
          lv_label_set_text(lbl_pre34, printbuf);

          convert_speed(P_S_settings[5].setting_speed);
          lv_label_set_text(lbl_speed30, printbuf);
          convert_speed(P_S_settings[6].setting_speed);
          lv_label_set_text(lbl_speed31, printbuf);
          convert_speed(P_S_settings[7].setting_speed);
          lv_label_set_text(lbl_speed32, printbuf);
          convert_speed(P_S_settings[8].setting_speed);
          lv_label_set_text(lbl_speed33, printbuf);
          convert_speed(P_S_settings[9].setting_speed);
          lv_label_set_text(lbl_speed34, printbuf);

          P_S_settings[5].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag30, printbuf);
          P_S_settings[6].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag31, printbuf);
          P_S_settings[7].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag32, printbuf);
          P_S_settings[8].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag33, printbuf);
          P_S_settings[9].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag34, printbuf);

          lv_label_set_text(lbl_pre_title1, "6");
          lv_label_set_text(lbl_pre_title2, "7");
          lv_label_set_text(lbl_pre_title3, "8");
          lv_label_set_text(lbl_pre_title4, "9");
          lv_label_set_text(lbl_pre_title5, "10");
          lv_label_set_text(lbl_pre_number, "6-10");
          break;

        case 2:
          convert_position(P_S_settings[10].setting_position);
          lv_label_set_text(lbl_pre30, printbuf);
          convert_position(P_S_settings[11].setting_position);
          lv_label_set_text(lbl_pre31, printbuf);
          convert_position(P_S_settings[12].setting_position);
          lv_label_set_text(lbl_pre32, printbuf);
          convert_position(P_S_settings[13].setting_position);
          lv_label_set_text(lbl_pre33, printbuf);
          convert_position(P_S_settings[14].setting_position);
          lv_label_set_text(lbl_pre34, printbuf);

          convert_speed(P_S_settings[10].setting_speed);
          lv_label_set_text(lbl_speed30, printbuf);
          convert_speed(P_S_settings[11].setting_speed);
          lv_label_set_text(lbl_speed31, printbuf);
          convert_speed(P_S_settings[12].setting_speed);
          lv_label_set_text(lbl_speed32, printbuf);
          convert_speed(P_S_settings[13].setting_speed);
          lv_label_set_text(lbl_speed33, printbuf);
          convert_speed(P_S_settings[14].setting_speed);
          lv_label_set_text(lbl_speed34, printbuf);

          P_S_settings[10].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag30, printbuf);
          P_S_settings[11].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag31, printbuf);
          P_S_settings[12].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag32, printbuf);
          P_S_settings[13].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag33, printbuf);
          P_S_settings[14].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag34, printbuf);

          lv_label_set_text(lbl_pre_title1, "11");
          lv_label_set_text(lbl_pre_title2, "12");
          lv_label_set_text(lbl_pre_title3, "13");
          lv_label_set_text(lbl_pre_title4, "14");
          lv_label_set_text(lbl_pre_title5, "15");
          lv_label_set_text(lbl_pre_number, "11-15");
          break;

        case 3:
          convert_position(P_S_settings[15].setting_position);
          lv_label_set_text(lbl_pre30, printbuf);
          convert_position(P_S_settings[16].setting_position);
          lv_label_set_text(lbl_pre31, printbuf);
          convert_position(P_S_settings[17].setting_position);
          lv_label_set_text(lbl_pre32, printbuf);
          convert_position(P_S_settings[18].setting_position);
          lv_label_set_text(lbl_pre33, printbuf);
          convert_position(P_S_settings[19].setting_position);
          lv_label_set_text(lbl_pre34, printbuf);

          convert_speed(P_S_settings[15].setting_speed);
          lv_label_set_text(lbl_speed30, printbuf);
          convert_speed(P_S_settings[16].setting_speed);
          lv_label_set_text(lbl_speed31, printbuf);
          convert_speed(P_S_settings[17].setting_speed);
          lv_label_set_text(lbl_speed32, printbuf);
          convert_speed(P_S_settings[18].setting_speed);
          lv_label_set_text(lbl_speed33, printbuf);
          convert_speed(P_S_settings[19].setting_speed);
          lv_label_set_text(lbl_speed34, printbuf);

          P_S_settings[15].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag30, printbuf);
          P_S_settings[16].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag31, printbuf);
          P_S_settings[17].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag32, printbuf);
          P_S_settings[18].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag33, printbuf);
          P_S_settings[19].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag34, printbuf);

          lv_label_set_text(lbl_pre_title1, "16");
          lv_label_set_text(lbl_pre_title2, "17");
          lv_label_set_text(lbl_pre_title3, "18");
          lv_label_set_text(lbl_pre_title4, "19");
          lv_label_set_text(lbl_pre_title5, "20");
          lv_label_set_text(lbl_pre_number, "16-20");
          break;

        case 4:
          convert_position(P_S_settings[20].setting_position);
          lv_label_set_text(lbl_pre30, printbuf);
          convert_position(P_S_settings[21].setting_position);
          lv_label_set_text(lbl_pre31, printbuf);
          convert_position(P_S_settings[22].setting_position);
          lv_label_set_text(lbl_pre32, printbuf);
          convert_position(P_S_settings[23].setting_position);
          lv_label_set_text(lbl_pre33, printbuf);
          convert_position(P_S_settings[24].setting_position);
          lv_label_set_text(lbl_pre34, printbuf);

          convert_speed(P_S_settings[20].setting_speed);
          lv_label_set_text(lbl_speed30, printbuf);
          convert_speed(P_S_settings[21].setting_speed);
          lv_label_set_text(lbl_speed31, printbuf);
          convert_speed(P_S_settings[22].setting_speed);
          lv_label_set_text(lbl_speed32, printbuf);
          convert_speed(P_S_settings[23].setting_speed);
          lv_label_set_text(lbl_speed33, printbuf);
          convert_speed(P_S_settings[24].setting_speed);
          lv_label_set_text(lbl_speed34, printbuf);

          P_S_settings[20].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag30, printbuf);
          P_S_settings[21].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag31, printbuf);
          P_S_settings[22].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag32, printbuf);
          P_S_settings[23].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag33, printbuf);
          P_S_settings[24].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag34, printbuf);
          lv_label_set_text(lbl_pre_title1, "21");
          lv_label_set_text(lbl_pre_title2, "22");
          lv_label_set_text(lbl_pre_title3, "23");
          lv_label_set_text(lbl_pre_title4, "24");
          lv_label_set_text(lbl_pre_title5, "25");
          lv_label_set_text(lbl_pre_number, "21-25");
          break;

      }
      break;

    case TAB_SEQUENCER_REF:
      strbuf = "  " + String(sequence_mode + 1);
      strbuf =  strbuf.substring(strbuf.length() - 1);
      strbuf.toCharArray(printbuf, 2);
      lv_label_set_text(lbl_seq_number, printbuf);
      switch (sequence_mode) {
        case 0:
          convert_position(P_S_settings[25].setting_position);
          lv_label_set_text(lbl_pre100, printbuf);
          convert_position(P_S_settings[26].setting_position);
          lv_label_set_text(lbl_pre101, printbuf);
          convert_position(P_S_settings[27].setting_position);
          lv_label_set_text(lbl_pre102, printbuf);
          convert_position(P_S_settings[28].setting_position);
          lv_label_set_text(lbl_pre103, printbuf);
          convert_position(P_S_settings[29].setting_position);
          lv_label_set_text(lbl_pre104, printbuf);

          convert_speed(P_S_settings[25].setting_speed);
          lv_label_set_text(lbl_speed100, printbuf);
          convert_speed(P_S_settings[26].setting_speed);
          lv_label_set_text(lbl_speed101, printbuf);
          convert_speed(P_S_settings[27].setting_speed);
          lv_label_set_text(lbl_speed102, printbuf);
          convert_speed(P_S_settings[28].setting_speed);
          lv_label_set_text(lbl_speed103, printbuf);
          convert_speed(P_S_settings[29].setting_speed);
          lv_label_set_text(lbl_speed104, printbuf);

          P_S_settings[25].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag100, printbuf);
          P_S_settings[26].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag101, printbuf);
          P_S_settings[27].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag102, printbuf);
          P_S_settings[28].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag103, printbuf);
          P_S_settings[29].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag104, printbuf);
          setting_sequencer_info[0].toCharArray(printbuf, 20);
          lv_label_set_text(lbl_seqtitle100, printbuf);

          break;

        case 1:
          convert_position(P_S_settings[30].setting_position);
          lv_label_set_text(lbl_pre100, printbuf);
          convert_position(P_S_settings[31].setting_position);
          lv_label_set_text(lbl_pre101, printbuf);
          convert_position(P_S_settings[32].setting_position);
          lv_label_set_text(lbl_pre102, printbuf);
          convert_position(P_S_settings[33].setting_position);
          lv_label_set_text(lbl_pre103, printbuf);
          convert_position(P_S_settings[34].setting_position);
          lv_label_set_text(lbl_pre104, printbuf);

          convert_speed(P_S_settings[30].setting_speed);
          lv_label_set_text(lbl_speed100, printbuf);
          convert_speed(P_S_settings[31].setting_speed);
          lv_label_set_text(lbl_speed101, printbuf);
          convert_speed(P_S_settings[32].setting_speed);
          lv_label_set_text(lbl_speed102, printbuf);
          convert_speed(P_S_settings[33].setting_speed);
          lv_label_set_text(lbl_speed103, printbuf);
          convert_speed(P_S_settings[34].setting_speed);
          lv_label_set_text(lbl_speed104, printbuf);

          P_S_settings[30].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag100, printbuf);
          P_S_settings[31].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag101, printbuf);
          P_S_settings[32].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag102, printbuf);
          P_S_settings[33].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag103, printbuf);
          P_S_settings[34].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag104, printbuf);
          setting_sequencer_info[1].toCharArray(printbuf, 20);
          lv_label_set_text(lbl_seqtitle100, printbuf);

          break;

        case 2:
          convert_position(P_S_settings[35].setting_position);
          lv_label_set_text(lbl_pre100, printbuf);
          convert_position(P_S_settings[36].setting_position);
          lv_label_set_text(lbl_pre101, printbuf);
          convert_position(P_S_settings[37].setting_position);
          lv_label_set_text(lbl_pre102, printbuf);
          convert_position(P_S_settings[38].setting_position);
          lv_label_set_text(lbl_pre103, printbuf);
          convert_position(P_S_settings[39].setting_position);
          lv_label_set_text(lbl_pre104, printbuf);

          convert_speed(P_S_settings[35].setting_speed);
          lv_label_set_text(lbl_speed100, printbuf);
          convert_speed(P_S_settings[36].setting_speed);
          lv_label_set_text(lbl_speed101, printbuf);
          convert_speed(P_S_settings[37].setting_speed);
          lv_label_set_text(lbl_speed102, printbuf);
          convert_speed(P_S_settings[38].setting_speed);
          lv_label_set_text(lbl_speed103, printbuf);
          convert_speed(P_S_settings[39].setting_speed);
          lv_label_set_text(lbl_speed104, printbuf);

          P_S_settings[35].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag100, printbuf);
          P_S_settings[36].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag101, printbuf);
          P_S_settings[37].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag102, printbuf);
          P_S_settings[38].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag103, printbuf);
          P_S_settings[39].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag104, printbuf);
          setting_sequencer_info[2].toCharArray(printbuf, 20);
          lv_label_set_text(lbl_seqtitle100, printbuf);

          break;

        case 3:
          convert_position(P_S_settings[40].setting_position);
          lv_label_set_text(lbl_pre100, printbuf);
          convert_position(P_S_settings[41].setting_position);
          lv_label_set_text(lbl_pre101, printbuf);
          convert_position(P_S_settings[42].setting_position);
          lv_label_set_text(lbl_pre102, printbuf);
          convert_position(P_S_settings[43].setting_position);
          lv_label_set_text(lbl_pre103, printbuf);
          convert_position(P_S_settings[44].setting_position);
          lv_label_set_text(lbl_pre104, printbuf);

          convert_speed(P_S_settings[40].setting_speed);
          lv_label_set_text(lbl_speed100, printbuf);
          convert_speed(P_S_settings[41].setting_speed);
          lv_label_set_text(lbl_speed101, printbuf);
          convert_speed(P_S_settings[42].setting_speed);
          lv_label_set_text(lbl_speed102, printbuf);
          convert_speed(P_S_settings[43].setting_speed);
          lv_label_set_text(lbl_speed103, printbuf);
          convert_speed(P_S_settings[44].setting_speed);
          lv_label_set_text(lbl_speed104, printbuf);

          P_S_settings[40].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag100, printbuf);
          P_S_settings[41].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag101, printbuf);
          P_S_settings[42].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag102, printbuf);
          P_S_settings[43].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag103, printbuf);
          P_S_settings[44].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag104, printbuf);
          setting_sequencer_info[3].toCharArray(printbuf, 20);
          lv_label_set_text(lbl_seqtitle100, printbuf);

          break;

        case 4:
          convert_position(P_S_settings[45].setting_position);
          lv_label_set_text(lbl_pre100, printbuf);
          convert_position(P_S_settings[46].setting_position);
          lv_label_set_text(lbl_pre101, printbuf);
          convert_position(P_S_settings[47].setting_position);
          lv_label_set_text(lbl_pre102, printbuf);
          convert_position(P_S_settings[48].setting_position);
          lv_label_set_text(lbl_pre103, printbuf);
          convert_position(P_S_settings[49].setting_position);
          lv_label_set_text(lbl_pre104, printbuf);

          convert_speed(P_S_settings[45].setting_speed);
          lv_label_set_text(lbl_speed100, printbuf);
          convert_speed(P_S_settings[46].setting_speed);
          lv_label_set_text(lbl_speed101, printbuf);
          convert_speed(P_S_settings[47].setting_speed);
          lv_label_set_text(lbl_speed102, printbuf);
          convert_speed(P_S_settings[48].setting_speed);
          lv_label_set_text(lbl_speed103, printbuf);
          convert_speed(P_S_settings[49].setting_speed);
          lv_label_set_text(lbl_speed104, printbuf);

          P_S_settings[45].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag100, printbuf);
          P_S_settings[46].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag101, printbuf);
          P_S_settings[47].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag102, printbuf);
          P_S_settings[48].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag103, printbuf);
          P_S_settings[49].setting_info.toCharArray(printbuf, 20);
          lv_label_set_text(lbl_tag104, printbuf);
          setting_sequencer_info[4].toCharArray(printbuf, 20);
          lv_label_set_text(lbl_seqtitle100, printbuf);
          break;
      }
      break;
  }
}

void presets_set_selected_buttons_state(void) {
  if (presets_selected == 1) {
    lv_obj_add_style(btn_preset_0, &btn_style1_selected, LV_PART_MAIN);
  } else {
    lv_obj_remove_style(btn_preset_0, &btn_style1_selected, LV_PART_MAIN);
  }
  if (presets_selected == 2) {
    lv_obj_add_style(btn_preset_1, &btn_style1_selected, LV_PART_MAIN);
  } else {
    lv_obj_remove_style(btn_preset_1, &btn_style1_selected, LV_PART_MAIN);
  }
  if (presets_selected == 3) {
    lv_obj_add_style(btn_preset_2, &btn_style1_selected, LV_PART_MAIN);
  } else {
    lv_obj_remove_style(btn_preset_2, &btn_style1_selected, LV_PART_MAIN);
  }
  if (presets_selected == 4) {
    lv_obj_add_style(btn_preset_3, &btn_style1_selected, LV_PART_MAIN);
  } else {
    lv_obj_remove_style(btn_preset_3, &btn_style1_selected, LV_PART_MAIN);
  }
  if (presets_selected == 5) {
    lv_obj_add_style(btn_preset_4, &btn_style1_selected, LV_PART_MAIN);
  } else {
    lv_obj_remove_style(btn_preset_4, &btn_style1_selected, LV_PART_MAIN);
  }
}

static void event_seq_next(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    sequence_mode = sequence_mode + 1;
    if (sequence_mode > 4) {
      sequence_mode = 4;
    }
    presets_selected = 0;
    sequencer_set_buttons_state();
    sequence_step = 0;
    update_seq_step();
    update_presets();
  }
}

static void event_seq_previous(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (sequence_mode > 0) {
      sequence_mode = sequence_mode - 1;
    }
    presets_selected = 0;
    sequencer_set_buttons_state();
    sequence_step = 0;
    update_seq_step();
    update_presets();
  }
}

// enable or disable next/prev buttons
void sequencer_set_buttons_state(void) {
  // disable if last screen
  if (sequence_mode == 4) {
    lv_obj_add_state(btn_sequencer_next, LV_STATE_DISABLED);
  }
  else {
    lv_obj_remove_state(btn_sequencer_next, LV_STATE_DISABLED);
  }
  // disable if last screen
  if (sequence_mode == 0) {
    lv_obj_add_state(btn_sequencer_previous, LV_STATE_DISABLED);
  }
  else {
    lv_obj_remove_state(btn_sequencer_previous, LV_STATE_DISABLED);
  }
}

static void event_seqedit1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(6, 4,  lbl_tag100);
  }
}

static void event_seqedit2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(7, 4,  lbl_tag101);
  }
}

static void event_seqedit3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(8, 4,  lbl_tag102);
  }
}

static void event_seqedit4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(9, 4,  lbl_tag103);
  }
}

static void event_seqedit5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(10, 4,  lbl_tag104);
  }
}

static void event_seqedit_title(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    keyboard_start(11, 4,  lbl_seqtitle100);
  }
}

void sequence_delete(uint8_t line_no) {
  // calculate index, sequence_mode = screen number
  uint8_t array_index = 25 + (sequence_mode * 5) + line_no;
  P_S_settings[array_index].setting_position = 0;
  P_S_settings[array_index].setting_speed = 0;
  P_S_settings[array_index].setting_info = PRESET_DEFAULT;
  save_update_presets();
}

static void event_seqdel1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    sequence_delete(0);
  }
}

static void event_seqdel2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    sequence_delete(1);
  }
}

static void event_seqdel3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    sequence_delete(2);
  }
}

static void event_seqdel4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    sequence_delete(3);
  }
}

static void event_seqdel5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    sequence_delete(4);
  }
}

void save_sequence(uint8_t line_no) {
  // calculate index, sequence_mode = screen number
  uint8_t array_index = 25 + (sequence_mode * 5) + line_no;
  P_S_settings[array_index].setting_position = router_position;
  P_S_settings[array_index].setting_speed = router_speed;
  save_update_presets();
}

static void event_seqsave1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_sequence(0);
  }
}

static void event_seqsave2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_sequence(1);
  }
}

static void event_seqsave3(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_sequence(2);
  }
}

static void event_seqsave4(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_sequence(3);
  }
}

static void event_seqsave5(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and position_valid) {
    save_sequence(4);
  }
}

static void sequence_go(void) {
  if (position_valid) {
    // when cancelled return to main screen since position is invalid
    router_return_from_busy = TAB_SEQUENCER_REF;
    show_busy_screen();
    uint8_t array_index = 25 + (sequence_mode * 5) + (sequence_step - 1);
    setspeed(P_S_settings[array_index].setting_speed);
    router_position = P_S_settings[array_index].setting_position;
    newposition = router_position;
    moveStepperState = MOVE_STEPPER_INIT;
  }
}

void update_seq_step(void) {
  switch (sequence_step) {
    case 0:
      lv_label_set_text(lbl_mark100, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark101, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark102, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark103, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark104, LV_SYMBOL_PLAY);
      break;
    case 1:
      lv_label_set_text(lbl_mark100, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark101, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark102, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark103, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark104, LV_SYMBOL_PLAY);
      break;
    case 2:
      lv_label_set_text(lbl_mark100, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark101, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark102, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark103, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark104, LV_SYMBOL_PLAY);
      break;
    case 3:
      lv_label_set_text(lbl_mark100, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark101, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark102, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark103, LV_SYMBOL_PLAY);
      lv_label_set_text(lbl_mark104, LV_SYMBOL_PLAY);
      break;
    case 4:
      lv_label_set_text(lbl_mark100, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark101, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark102, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark103, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark104, LV_SYMBOL_PLAY);
      break;
    case 5:
      lv_label_set_text(lbl_mark100, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark101, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark102, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark103, LV_SYMBOL_OK);
      lv_label_set_text(lbl_mark104, LV_SYMBOL_OK);
      break;
  }
}

static void event_stepsizechange(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    router_stepsize = lv_dropdown_get_selected(obj);
  }
}

static void update_stepsize(void) {
  lv_dropdown_set_selected(ddlist_stepsize, router_stepsize);
}

float get_stepsize(void)
{
  float stepsize = 0;
  switch (router_stepsize) {
    case 0:
      stepsize = 0.1;
      break;
    case 1:
      stepsize = 0.2;
      break;
    case 2:
      stepsize = 0.5;
      break;
    case 3:
      stepsize = 1;
      break;
    case 4:
      stepsize = 2;
      break;
    case 5:
      stepsize = 5;
      break;
    case 6:
      stepsize = 10;
  }
  return stepsize;
}

static void event_speedchange(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    router_speed = lv_dropdown_get_selected(obj);
  }
}

static void update_speed(void) {
  lv_dropdown_set_selected(ddlist_speed, router_speed);
}

static void setspeed(uint8_t newspeed) {
  int32_t speedmHz, accelHzPerS;
  switch (newspeed) {
    case 0: speedmHz = 100000;  accelHzPerS = 200;  break;
    case 1: speedmHz = 200000;  accelHzPerS = 400;  break;
    case 2: speedmHz = 500000;  accelHzPerS = 1000; break;
    case 3: speedmHz = 750000;  accelHzPerS = 1500; break;
    case 4: speedmHz = 1000000; accelHzPerS = 2000; break;
    default: speedmHz = 500000; accelHzPerS = 1000; break;
  }
  send_command_to_task(CMD_SET_SPEED, speedmHz);
  send_command_to_task(CMD_SET_ACCEL, accelHzPerS);
}

void convert_speed(uint8_t spd) {
  strbuf = "??";
  switch (spd) {
    case 0:
      strbuf = "VS";
      break;
    case 1:
      strbuf = "SL";
      break;
    case 2:
      strbuf = "ME";
      break;
    case 3:
      strbuf = "FA";
      break;
    case 4:
      strbuf = "VF";
      break;
  }
  strbuf.toCharArray(printbuf, 3);
}

// update lift position
static void update_position() {
  // change color if position is not validated
  lv_color_t newcolor = lv_color_white();
  if (!position_valid) {
    router_position = 0;
    newcolor = lv_palette_main(LV_PALETTE_RED);
  }
  // format the position to a string with 1 decimal place
  strbuf = String(router_position, 1);
  // Ensure the decimal separator is a dot
  strbuf.replace(',', '.');
  // Convert the string to a char array
  strbuf.toCharArray(printbuf, 30);
  lv_obj_set_style_text_color(btn_pos_small_label, newcolor, 0);
  lv_label_set_text(btn_pos_small_label, printbuf);
  lv_obj_set_style_text_color(btn_pos_small_label2, newcolor, 0);
  lv_label_set_text(btn_pos_small_label2, printbuf);
  lv_obj_set_style_text_color(btn_pos_large_label, newcolor, 0);
  lv_label_set_text(btn_pos_large_label, printbuf);
}

void convert_position(float pos) {
  // Format the position to a string with 1 decimal place
  strbuf = String(pos, 1);
  // Ensure the decimal separator is a dot
  strbuf.replace(',', '.');
  // Convert the string to a char array
  strbuf.toCharArray(printbuf, 30);
}

// show or hide hourglass busy symbol
void set_busy_symbol(boolean isbusy) {
  if (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF) {
    if (isbusy) {
      lv_obj_remove_flag(img_busy_main, LV_OBJ_FLAG_HIDDEN);
    }
    else {
      lv_obj_add_flag(img_busy_main, LV_OBJ_FLAG_HIDDEN);
    }
    lv_refr_now(NULL);
  }
  if (lv_tabview_get_tab_act(tabview) == TAB_MAINSUB_REF) {
    if (isbusy) {
      lv_obj_remove_flag(img_busy_mainsub, LV_OBJ_FLAG_HIDDEN);
    }
    else {
      lv_obj_add_flag(img_busy_mainsub, LV_OBJ_FLAG_HIDDEN);
    }
    lv_refr_now(NULL);
  }
  if (lv_tabview_get_tab_act(tabview) == TAB_ZOOM_REF) {
    if (isbusy) {
      lv_obj_remove_flag(img_busy_zoom, LV_OBJ_FLAG_HIDDEN);
    }
    else {
      lv_obj_add_flag(img_busy_zoom, LV_OBJ_FLAG_HIDDEN);
    }
    lv_refr_now(NULL);
  }
}

// activate busy screen
void show_busy_screen(void) {
  lv_tabview_set_act(tabview, TAB_WAITBUSY_REF, LV_ANIM_OFF);
  lv_refr_now(NULL);
}

void show_error_message(void) {
  if (error_number > 0) {
    const char *error_message;
    switch (error_number) {
      case 1:
        error_message = "Failed to clear top switch";
        break;

      case 2:
        error_message = "Fail: end switch reached";
        break;

      case 3:
        error_message = "Failed to detect zero sensor";
        break;

      case 4:
        error_message = "Failed to detect top switch";
        break;

      case 5:
        error_message = "Failed to clear bottom switch";
        break;

      case 6:
        error_message = "Failed to detect bottom switch";
        break;

      case 7:
        error_message = "New position outside limits";
        break;

      case 8:
        error_message = "Failed to clear zero sensor";
        break;

      default:
        error_message = "Unknown error";
        break;
    }

    // create a message box
    lv_obj_t *mbox1 = lv_msgbox_create(lv_scr_act());
    lv_obj_set_size(mbox1, 460, 230);
    lv_obj_center(mbox1);

    // styles
    static lv_style_t style_text;
    lv_style_init(&style_text);
    lv_style_set_text_font(&style_text, &lv_font_montserrat_26);

    static lv_style_t style_warning;
    lv_style_init(&style_warning);
    lv_style_set_text_color(&style_warning, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_text_font(&style_warning, &lv_font_montserrat_40);

    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    lv_style_set_text_font(&style_btn, &lv_font_montserrat_24);

    // content (centered vertically + horizontally)
    lv_obj_t *content = lv_msgbox_get_content(mbox1);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content,
                          LV_FLEX_ALIGN_CENTER, // horizontal
                          LV_FLEX_ALIGN_CENTER, // vertical
                          LV_FLEX_ALIGN_CENTER);// track align

    // warning symbol
    lv_obj_t *text1 = lv_msgbox_add_text(mbox1, LV_SYMBOL_WARNING);
    lv_obj_add_style(text1, &style_warning, LV_PART_MAIN);
    lv_obj_set_width(text1, LV_PCT(100));
    lv_obj_set_style_text_align(text1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_bottom(text1, 30, 0);

    // error message
    lv_obj_t *text = lv_msgbox_add_text(mbox1, error_message);
    lv_obj_add_style(text, &style_text, LV_PART_MAIN);
    lv_obj_set_width(text, LV_PCT(100));
    lv_obj_set_style_text_align(text, LV_TEXT_ALIGN_CENTER, 0);

    // footer / button
    lv_obj_t *btn = lv_msgbox_add_footer_button(mbox1, "OK");
    lv_obj_add_style(btn, &style_btn, LV_PART_MAIN);
    lv_obj_set_size(btn, 140, 55);

    // center button inside footer
    lv_obj_t *footer = lv_msgbox_get_footer(mbox1);
    lv_obj_set_height(footer, 80);
    lv_obj_set_flex_align(footer,
                          LV_FLEX_ALIGN_CENTER, // horizontal
                          LV_FLEX_ALIGN_CENTER, // vertical
                          LV_FLEX_ALIGN_CENTER);// track align

    lv_obj_add_event_cb(btn, event_messagebox_close, LV_EVENT_CLICKED, NULL);

    // clear error after it is shown
    error_number = 0;
  }
}

// called by messagebox
static void event_messagebox_close(lv_event_t * e)
{
  lv_obj_t * btn = (lv_obj_t*) lv_event_get_target(e); // Get the button that triggered the event
  // traverse up to find the message box
  lv_obj_t * mbox = btn;
  while (mbox && !lv_obj_check_type(mbox, &lv_msgbox_class)) {
    mbox = lv_obj_get_parent(mbox);
  }
  if (mbox) {
    lv_msgbox_close(mbox);
  }
}

void update_controls(void) {
  if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
    if (position_valid) {
      // enable all buttons
      lv_obj_clear_state(btn_gotozero, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_setzero, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_findzero, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_nextmain, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_presets, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_sequencer, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_gomaxup, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_gomaxdown, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_calculator, LV_STATE_DISABLED);
      lv_obj_clear_state(ddlist_stepsize, LV_STATE_DISABLED);
      lv_obj_clear_state(ddlist_speed, LV_STATE_DISABLED);
    }
    else {
      // enable only valid buttons when position is invalid
      lv_obj_clear_state(btn_setzero, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_findzero, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_nextmain, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_gomaxup, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_gomaxdown, LV_STATE_DISABLED);
      lv_obj_clear_state(ddlist_stepsize, LV_STATE_DISABLED);
      lv_obj_clear_state(ddlist_speed, LV_STATE_DISABLED);
      lv_obj_add_state(btn_gotozero, LV_STATE_DISABLED);
      lv_obj_add_state(btn_presets, LV_STATE_DISABLED);
      lv_obj_add_state(btn_sequencer, LV_STATE_DISABLED);
      lv_obj_add_state(btn_calculator, LV_STATE_DISABLED);
    }
    lv_obj_add_state(btn_cancelmove, LV_STATE_DISABLED);
  }
  else {
    // we are moving so disable buttons
    lv_obj_add_state(btn_gotozero, LV_STATE_DISABLED);
    lv_obj_add_state(btn_setzero, LV_STATE_DISABLED);
    lv_obj_add_state(btn_findzero, LV_STATE_DISABLED);
    lv_obj_add_state(btn_nextmain, LV_STATE_DISABLED);
    lv_obj_add_state(btn_presets, LV_STATE_DISABLED);
    lv_obj_add_state(btn_sequencer, LV_STATE_DISABLED);
    lv_obj_add_state(btn_gomaxup, LV_STATE_DISABLED);
    lv_obj_add_state(btn_gomaxdown, LV_STATE_DISABLED);
    lv_obj_add_state(btn_calculator, LV_STATE_DISABLED);
    lv_obj_add_state(ddlist_stepsize, LV_STATE_DISABLED);
    lv_obj_add_state(ddlist_speed, LV_STATE_DISABLED);
    // enable cancel
    lv_obj_clear_state(btn_cancelmove, LV_STATE_DISABLED);
  }
}

static void event_calculator(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (!is_motor_running() and moveStepperState == MOVE_STEPPER_IDLE and senseZeroState == SENSE_ZERO_IDLE and gotoMaxBottomState == GOTO_MAX_BOTTOM_IDLE and gotoMaxTopState == GOTO_MAX_TOP_IDLE) {
      lv_tabview_set_act(tabview, TAB_CALC_REF, LV_ANIM_OFF);
      // disable if no valid position
      if (position_valid) {
        lv_obj_remove_state(btn_gotocalc, LV_STATE_DISABLED);
      }
      else {
        lv_obj_add_state(btn_gotocalc, LV_STATE_DISABLED);
      }
      calc_dec_active = false;
      calc_one = "0";
      calc_two = "";
      calc_dec_one = "";
      calc_dec_two = "";
      calc_ops = "";
      update_calc_1();
    }
  }
}

static void event_calckey(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    uint32_t btn_id = lv_buttonmatrix_get_selected_button(obj);
    const char * btn_value = lv_buttonmatrix_get_button_text(obj, btn_id);
    char newkey = btn_value[0];
    if (isDigit(newkey)) {
      if (calc_dec_active) {
        if (calc_dec_one.length() < 2) {
          calc_dec_one.concat(newkey);
        }
      }
      else {
        if (  calc_one == "0" and calc_one.length() == 1) {
          calc_one = newkey;
        }
        else {
          if (calc_one.length() < 5) {
            calc_one.concat(newkey);
          }
        }
      }
    }
    else {
      // not a digit
      switch (newkey) {
        case '.':
          if (!calc_dec_active) {
            calc_dec_active = true;
          }
          break;

        case '<':
          if (calc_dec_active) {
            if (calc_dec_one.length() > 0) {
              calc_dec_one.remove(calc_dec_one.length() - 1, 1);
            }
            else {
              calc_dec_active = false;
            }
          }
          else {
            if (calc_one.length() == 1 and calc_one == "0" ) {
              if (calc_two.length() > 0 ) {
                calc_ops = "";
                calc_one = calc_two;
                calc_two = "";
                calc_dec_one = calc_dec_two;
                calc_dec_two = "";
                calc_dec_active = true;
              }
            }
            else {
              if (calc_one.length() == 1) {
                calc_one = "0";
              }
              else {
                if (calc_one.length() > 0) {
                  calc_one.remove(calc_one.length() - 1, 1);
                }
              }
            }
          }
          break;

        case '+':
          if (calc_one == "0" and calc_one.length() == 1) {
          }
          else {
            if (calc_ops == "") {
              calc_ops = "+";
              calc_two = calc_one;
              calc_one = "0";
              calc_dec_two = calc_dec_one;
              calc_dec_one = "";
              calc_dec_active = false;
              if (calc_dec_two.length() < 2) {
                calc_dec_two.concat("00");
                calc_dec_two = calc_dec_two.substring(0, 1);
              }
            }
          }
          break;

        case '-':
          if (calc_one == "0" and calc_one.length() == 1) {
          }
          else {
            if (calc_ops == "") {
              calc_ops = "-";
              calc_two = calc_one;
              calc_one = "0";
              calc_dec_two = calc_dec_one;
              calc_dec_one = "";
              calc_dec_active = false;
              if (calc_dec_two.length() < 2) {
                calc_dec_two.concat("00");
                calc_dec_two = calc_dec_two.substring(0, 1);
              }
            }
          }
          break;

        case 'x':
          if (calc_one == "0" and calc_one.length() == 1) {
          }
          else {
            if (calc_ops == "") {
              calc_ops = "x";
              calc_two = calc_one;
              calc_one = "0";
              calc_dec_two = calc_dec_one;
              calc_dec_one = "";
              calc_dec_active = false;
              if (calc_dec_two.length() < 2) {
                calc_dec_two.concat("00");
                calc_dec_two = calc_dec_two.substring(0, 1);
              }
            }
          }
          break;

        case '/':
          if (calc_one == "0" and calc_one.length() == 1) {
          }
          else {
            if (calc_ops == "") {
              calc_ops = "/";
              calc_two = calc_one;
              calc_one = "0";
              calc_dec_two = calc_dec_one;
              calc_dec_one = "";
              calc_dec_active = false;
              if (calc_dec_two.length() < 2) {
                calc_dec_two.concat("00");
                calc_dec_two = calc_dec_two.substring(0, 1);
              }
            }
          }
          break;

        case 'C':
          calc_dec_active = false;
          calc_one = "0";
          calc_two = "";
          calc_dec_one = "";
          calc_dec_two = "";
          calc_ops = "";
          break;

        case '=':
          if (calc_ops != "") {
            String numberone = calc_two + "." + calc_dec_two;
            String numbertwo = calc_one + "." + calc_dec_one;
            float value1 = numberone.toFloat();
            float value2 = numbertwo.toFloat();
            float calc_result = 0;
            String out;
            if (calc_ops == "+") {
              calc_result = value1 + value2;
            }
            else {
              if (calc_ops == "-") {
                calc_result = value1 - value2;
              }
              else {

                if (calc_ops == "x") {
                  calc_result = value1 * value2;
                }
                else {

                  if (calc_ops == "/") {
                    calc_result = value1 / value2;
                  }
                }
              }
            }

            if (calc_result > 99999.99) {
              calc_result = 99999.99;
            }
            out = String(calc_result, 2);
            calc_dec_one = out.substring(out.length() - 2, out.length());
            calc_one = out.substring(0, out.length() - 3);
            if (calc_dec_one == "00") {
              calc_dec_active = false;
              calc_dec_one = "";
            }
            else {
              calc_dec_active = true;
            }
            calc_two = "";
            calc_dec_two = "";
            calc_ops = "";
          }
          break;

        case 'R':
          char newkey2 = btn_value[1];
          switch (newkey2) {
            case '+':
              if (calc_dec_active) {
                if (calc_dec_one.length() == 2) {
                  String dec = calc_dec_one.substring(calc_dec_one.length() - 1, calc_dec_one.length());
                  int decvalue = dec.toInt();
                  if (decvalue < 5 and decvalue != 0) {
                    dec = calc_dec_one.substring(0, 1);
                    dec.concat('5');
                    calc_dec_one = dec;
                  }
                  else {
                    if (decvalue > 5 and decvalue != 0 ) {
                      dec = calc_dec_one.substring(0, 1);
                      decvalue = dec.toInt() + 1;
                      dec = String(decvalue);
                      dec.concat('0');
                      calc_dec_one = dec;
                    }
                  }
                }
              }
              break;

            case '-':
              if (calc_dec_active) {
                if (calc_dec_one.length() == 2) {
                  String dec = calc_dec_one.substring(calc_dec_one.length() - 1, calc_dec_one.length());
                  int decvalue = dec.toInt();
                  if (decvalue < 5 and decvalue != 0) {
                    dec = calc_dec_one.substring(0, 1);
                    dec.concat('0');
                    calc_dec_one = dec;
                  }
                  else {
                    if (decvalue > 5 and decvalue != 0 ) {
                      dec = calc_dec_one.substring(0, 1);
                      dec.concat('5');
                      calc_dec_one = dec;
                    }
                  }
                }
              }
              break;
          }
          break;
      }
    }
    update_calc_1();
  }
}

void update_calc_1(void) {
  String result1 = calc_one;
  String result2 = calc_two;
  if (calc_dec_active) {
    result1.concat(".");
    if (calc_dec_one.length() > 0) {
      result1.concat(calc_dec_one);
    }
  }
  result1.toCharArray(printbuf, 30);
  lv_label_set_text(lbl_calc1, printbuf);
  if (calc_ops.length() == 1) {
    calc_ops.toCharArray(printbuf, 30);
    lv_label_set_text(lbl_calcops, printbuf);
    result2.concat(".");
    result2.concat(calc_dec_two);
    result2.toCharArray(printbuf, 30);
    lv_label_set_text(lbl_calc2, printbuf);
  }
  else {
    lv_label_set_text(lbl_calc2, " ");
    lv_label_set_text(lbl_calcops, " ");
  }
}

void event_gotocalc(lv_event_t * e) {
  // goto calculated position
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (calc_ops == "" and position_valid) {
      // no operation in progress and valid position
      String calc_pos = calc_one + "." + calc_dec_one;
      float calc_value = (calc_pos.toFloat()) * 10;
      calc_value  = round(calc_value) / 10;
      if (abs(calc_value) <= MAX_ROUTER_POS) {
        // limits ok
        router_return_from_busy = -1;
        show_busy_screen();
        setspeed(router_speed);
        router_position = calc_value;
        newposition = router_position;
        moveStepperState = MOVE_STEPPER_INIT;
        // move_stepper(router_position);
        lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
        update_position();
      }
      else {
        invalidate_position();
        error_number = 7;
      }
    }
  }
}

void direct_start(void) {
  lv_tabview_set_act(tabview, TAB_DIRECTENTRY_REF, LV_ANIM_OFF);
  direct_dec_active = false;
  direct_one = "0";
  direct_dec_one = "";
  update_direct_pos();
}

static void event_directkey(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    uint32_t btn_id = lv_buttonmatrix_get_selected_button(obj);
    const char * btn_value = lv_buttonmatrix_get_button_text(obj, btn_id);
    char newkey = btn_value[0];
    if (isDigit(newkey)) {
      if (direct_dec_active) {
        if (direct_dec_one.length() < 1) {
          direct_dec_one.concat(newkey);
        }
      }
      else {
        if (  direct_one == "0" and direct_one.length() == 1) {
          direct_one = newkey;
        }
        else {
          if (direct_one.length() < 3) {
            direct_one.concat(newkey);
          }
        }
      }
    }
    else {
      // not a digit
      switch (newkey) {
        case '.':
          if (!direct_dec_active) {
            direct_dec_active = true;
          }
          break;

        case 'C':
          direct_dec_active = false;
          direct_one = "0";
          direct_dec_one = "";
          break;
      }
    }
    update_direct_pos();
  }
}

void update_direct_pos(void) {
  String result1 = direct_one;
  if (direct_dec_active) {
    result1.concat(".");
    if (direct_dec_one.length() > 0) {
      result1.concat(direct_dec_one);
    }
  }
  result1.toCharArray(printbuf, 30);
  lv_label_set_text(lbl_directpos, printbuf);
}

void event_gotodirect(lv_event_t * e) {
  // goto entered position
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    String direct_pos = direct_one + "." + direct_dec_one;
    float direct_value = (direct_pos.toFloat()) * 10;
    direct_value  = round(direct_value) / 10;
    if (abs(direct_value) <= MAX_ROUTER_POS) {
      // limits ok
      router_return_from_busy = -1;
      show_busy_screen();
      setspeed(router_speed);
      router_position = direct_value;
      newposition = router_position;
      moveStepperState = MOVE_STEPPER_INIT;
      lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
      update_position();
    }
    else {
      lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
      error_number = 7;
    }
  }
}

void keyboard_start(uint8_t caller, uint8_t tab, lv_obj_t * obj ) {
  kb_caller = caller;
  kb_returntab = tab;
  kb_text = lv_label_get_text(obj);
  // if default, empty string
  if (kb_text == PRESET_DEFAULT) {
    kb_text = "";
  }
  // char printbuf[30];
  kb_text.toCharArray(printbuf, 20);
  lv_textarea_set_text(txtentry, printbuf);
  lv_tabview_set_act(tabview, TAB_EDIT_REF, LV_ANIM_OFF);
}

static void event_keyboardhandler(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_READY) {
    String kbtxt = lv_textarea_get_text(txtentry);
    if (kbtxt.length() == 0) {
      kbtxt = PRESET_DEFAULT;
    }
    uint8_t array_index = 0;
    if (kb_caller >= 1 and kb_caller <= 5) {
      array_index = (preset_mode * 5) + (kb_caller - 1);
      P_S_settings[array_index].setting_info = kbtxt;
    }
    if (kb_caller >= 6 and kb_caller <= 10) {
      array_index = 25 + (sequence_mode * 5) + (kb_caller - 6);
      P_S_settings[array_index].setting_info = kbtxt;
    }
    if (kb_caller == 11) {
      setting_sequencer_info[sequence_mode] = kbtxt;
    }
    lv_tabview_set_act(tabview, kb_returntab, LV_ANIM_OFF);
    save_settings();
    update_presets();
  }
  if (event == LV_EVENT_CANCEL) {
    lv_tabview_set_act(tabview, kb_returntab, LV_ANIM_OFF);
  }
}

// save all preset and sequencer settings
void save_settings() {
  preferences.begin("settings", false);
  for (int i = 0; i < 50; i++) {
    preferences.putFloat(("pos" + String(i)).c_str(), P_S_settings[i].setting_position);
    preferences.putUChar(("spd" + String(i)).c_str(), P_S_settings[i].setting_speed);
    preferences.putString(("inf" + String(i)).c_str(), P_S_settings[i].setting_info);
  }
  for (int i = 0; i < 5; i++) {
    preferences.putString(("seqinf" + String(i)).c_str(), setting_sequencer_info[i]);
  }
  preferences.end();
}

// load all preset and sequencer settings
void load_settings() {
  preferences.begin("settings", true);
  // Load main settings
  for (int i = 0; i < 50; i++) {
    P_S_settings[i].setting_position = preferences.getFloat(("pos" + String(i)).c_str(), 0.0);
    P_S_settings[i].setting_speed = preferences.getUChar(("spd" + String(i)).c_str(), 0);
    P_S_settings[i].setting_info = preferences.getString(("inf" + String(i)).c_str(), PRESET_DEFAULT);
  }
  // Load extra strings
  for (int i = 0; i < 5; i++) {
    setting_sequencer_info[i] = preferences.getString(("seqinf" + String(i)).c_str(), PRESET_DEFAULT);
  }
  preferences.end();
}

// reset all preset and sequencer settings and save
void reset_settings() {
  preferences.begin("settings", false); // open Preferences in read-write mode
  preferences.clear(); // clear all stored data
  clear_settings();
  save_settings(); // save defaults
  preferences.end(); // close Preferences
}

// reset all preset and sequencer settings to default
void clear_settings() {
  for (int i = 0; i < 50; i++) {
    P_S_settings[i].setting_position = 0.0;
    P_S_settings[i].setting_speed = 0;
    P_S_settings[i].setting_info = PRESET_DEFAULT;
  }
  for (int i = 0; i < 5; i++) {
    setting_sequencer_info[i] = PRESET_DEFAULT;
  }
}

static void event_calibrate(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    calibratescreen();
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  }
}


void calibratescreen() {
  TFT.fillScreen(TFT_BLACK);
  // clear this namespace
  preferences.begin("calibration", false);
  preferences.clear();
  preferences.end();

  TFT.calibrateTouch(setting_calibration_data, TFT_WHITE, TFT_RED, 14);

  //  Serial.println("New calibration data:");
  //  for (int i = 0; i < 8; i++) {
  //    Serial.printf("  %d: %d\n", i, setting_calibration_data[i]);
  //  }

  save_calibration();
  TFT.setTouchCalibrate(setting_calibration_data);
}

void save_calibration() {
  preferences.begin("calibration", false);
  preferences.putBytes("calib", setting_calibration_data, sizeof(setting_calibration_data));
  preferences.end();
}

bool load_calibration() {
  preferences.begin("calibration", true);
  size_t len = preferences.getBytes("calib", setting_calibration_data, sizeof(setting_calibration_data));
  preferences.end();
  if (len != sizeof(setting_calibration_data)) {
    // Serial.println("No valid calibration data found.");
    return false;
  }
  // check if all 0
  bool allzero = true;
  for (int i = 0; i < 8; i++) {
    if (setting_calibration_data[i] != 0) {
      allzero = false;
      break;
    }
  }
  if (allzero) {
    // Serial.println("Calibration data empty.");
    return false;
  }
  //  Serial.println("Calibration data loaded:");
  //  for (int i = 0; i < 8; i++) {
  //    Serial.printf("  %d: %d\n", i, setting_calibration_data[i]);
  //  }
  return true;
}

// reset all calibration settings and save
void reset_calibration() {
  preferences.begin("calibration", false);
  preferences.clear();
  clear_calibration();
  save_calibration();
  preferences.end();
}

// reset all calibration settings to default
void clear_calibration() {
  for (int i = 0; i < 8; i++) {
    setting_calibration_data[i] = 0;
  }
}

// all stepper activity is handled by this task
void stepperTask(void *pvParameters) {
  stepper_msg_t cmd;
  for (;;) {
    // check for new commands
    if (!xQueueIsQueueEmptyFromISR(stepperQueue)) {
      if (xQueueReceive(stepperQueue, &cmd, pdMS_TO_TICKS(1)) == pdTRUE) {
        switch (cmd.cmd) {
          case CMD_NONE:
            break;

          case CMD_SET_SPEED:
            if (stepper->getSpeedInMilliHz() != cmd.value) {
              stepper->setSpeedInMilliHz(cmd.value);
            }
            break;

          case CMD_SET_ACCEL:
            if (stepper->getAcceleration() != cmd.value) {
              stepper->setAcceleration(cmd.value);
            }
            break;

          case CMD_SET_CURRENTPOSITION:
            stepper->setCurrentPosition(cmd.value);
            break;

          case CMD_MOVE:
            stepper->move(cmd.value);
            break;

          case CMD_MOVE_TO:
            stepper->moveTo(cmd.value);
            break;

          case CMD_STOP:
            stepper->stopMove();
            break;

          case CMD_EMERGENCY_STOP:
            // this generates E (25451) rmt: rmt_tx_disable(842): channel can't be disabled in state 0
            // but can be ignored
            if (stepper->isRunning()) {
              //Serial.println("running");
              stepper->forceStopAndNewPosition(0);
            }
            else {
              //Serial.println("not running");
              stepper->setCurrentPosition(0);
            }
            break;

          case CMD_SET_THIS_AS_CURRENT:
            stepper->stopMove();
            stepper->setCurrentPosition(stepper->getCurrentPosition());

            break;

          case CMD_ISRUNNING:
            stepper_running_feedback_t fb1;
            fb1.is_running = stepper->isRunning();
            xQueueSend(stepperRunningFeedbackQueue, &fb1, 0);
            break;

          case CMD_GET_CURRENTPOSITION:
            stepper_position_feedback_t fb2;
            fb2.current_position = stepper->getCurrentPosition();
            xQueueSend(stepperPositionFeedbackQueue, &fb2, 0);
            break;
        }

        //        Serial.print("Command: ");
        //        switch (cmd.cmd) {
        //          case CMD_NONE: Serial.print("NONE"); break;
        //          case CMD_SET_SPEED: Serial.print("SET_SPEED"); break;
        //          case CMD_SET_ACCEL: Serial.print("SET_ACCEL"); break;
        //          case CMD_SET_CURRENTPOSITION: Serial.print("SET_CURRENTPOSITION"); break;
        //          case CMD_MOVE: Serial.print("MOVE"); break;
        //          case CMD_MOVE_TO: Serial.print("MOVE_TO"); break;
        //          case CMD_STOP: Serial.print("STOP"); break;
        //          case CMD_EMERGENCY_STOP: Serial.print("EMERGENCY_STOP"); break;
        //          case CMD_SET_THIS_AS_CURRENT: Serial.print("SET_THIS_AS_CURRENT"); break;
        //          case CMD_ISRUNNING: Serial.print("ISRUNNING"); break;
        //          case CMD_GET_CURRENTPOSITION: Serial.print("GET_CURRENTPOSITION"); break;
        //
        //
        //          default: Serial.print("UNKNOWN"); break;
        //        }
        //
        //        Serial.print(" | Value: ");
        //        Serial.print(cmd.value);
        //
        //        if (cmd.cmd == CMD_SET_CURRENTPOSITION or cmd.cmd == CMD_MOVE or cmd.cmd == CMD_MOVE_TO) {
        //          Serial.print(" | Actual pos: ");
        //          Serial.println(cmd.value * 0.005);
        //        }
        //        else {
        //          Serial.println("");
        //        }
        //        // one step 0.005mm
      }
    }
    // to prevent watchdog error triggers
    vTaskDelay(1);
  }
}

// send command and parameter to task queue
void send_command_to_task(stepper_cmd_t command, int32_t value) {
  stepper_msg_t msg;
  msg.cmd = command;
  msg.value = value;
  xQueueSend(stepperQueue, &msg, portMAX_DELAY);
}

// get motor running status
boolean is_motor_running(void) {
  stepper_msg_t msg;
  msg.cmd = CMD_ISRUNNING;
  msg.value = 0;
  xQueueSend(stepperQueue, &msg, portMAX_DELAY);
  stepper_running_feedback_t fb;
  if (xQueueReceive(stepperRunningFeedbackQueue, &fb, pdMS_TO_TICKS(50))) {
    return fb.is_running;
  }
  return false;
}

// get current position in raw steps or mm
float get_current_position(boolean return_raw) {
  stepper_msg_t msg;
  msg.cmd = CMD_GET_CURRENTPOSITION;
  msg.value = 0;
  xQueueSend(stepperQueue, &msg, portMAX_DELAY);
  stepper_position_feedback_t fb;
  if (xQueueReceive(stepperPositionFeedbackQueue, &fb, pdMS_TO_TICKS(50))) {
    if (return_raw) {
      // return steps
      return fb.current_position;
    }
    else {
      // returns actual position in mm
      return (float)(fb.current_position / stepfactor);
    }
  }
  return 0;
}

// debug routine in case of problems
// alternative enable monitoring in lvgl
void print_memory_info_to_serial() {
  // Gather memory info
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  size_t stack_free = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

  // Print memory information to the serial monitor
  Serial.println("=== Memory Info ===");
  Serial.printf("Heap: %u / %u bytes free\n", (unsigned int)free_heap, (unsigned int)total_heap);

  // If PSRAM is available, display its info
  if (total_psram > 0) {
    Serial.printf("PSRAM: %u / %u bytes free\n", (unsigned int)free_psram, (unsigned int)total_psram);
  } else {
    Serial.println("PSRAM: Not available");
  }

  // Display stack memory info
  Serial.printf("Stack: %u bytes free\n", (unsigned int)stack_free);
  Serial.println("====================");
}

void get_event_name(lv_event_code_t event) {
  switch (event) {
    case LV_EVENT_ALL:
      Serial.println("LV_EVENT_ALL");
      break;
    case LV_EVENT_PRESSED:
      Serial.println("LV_EVENT_PRESSED");
      break;
    case LV_EVENT_PRESSING:
      Serial.println("LV_EVENT_PRESSING");
      break;
    case LV_EVENT_PRESS_LOST:
      Serial.println("LV_EVENT_PRESS_LOST");
      break;
    case LV_EVENT_SHORT_CLICKED:
      Serial.println("LV_EVENT_SHORT_CLICKED");
      break;
    case LV_EVENT_LONG_PRESSED:
      Serial.println("LV_EVENT_LONG_PRESSED");
      break;
    case LV_EVENT_LONG_PRESSED_REPEAT:
      Serial.println("LV_EVENT_LONG_PRESSED_REPEAT");
      break;
    case LV_EVENT_CLICKED:
      Serial.println("LV_EVENT_CLICKED");
      break;
    case LV_EVENT_DOUBLE_CLICKED:
      Serial.println("LV_EVENT_DOUBLE_CLICKED");
      break;
    case LV_EVENT_RELEASED:
      Serial.println("LV_EVENT_RELEASED");
      break;
    case LV_EVENT_SCROLL_BEGIN:
      Serial.println("LV_EVENT_SCROLL_BEGIN");
      break;
    case LV_EVENT_SCROLL_THROW_BEGIN:
      Serial.println("LV_EVENT_SCROLL_THROW_BEGIN");
      break;
    case LV_EVENT_SCROLL_END:
      Serial.println("LV_EVENT_SCROLL_END");
      break;
    case LV_EVENT_SCROLL:
      Serial.println("LV_EVENT_SCROLL");
      break;
    case LV_EVENT_GESTURE:
      Serial.println("LV_EVENT_GESTURE");
      break;
    case LV_EVENT_KEY:
      Serial.println("LV_EVENT_KEY");
      break;
    case LV_EVENT_ROTARY:
      Serial.println("LV_EVENT_ROTARY");
      break;
    case LV_EVENT_FOCUSED:
      Serial.println("LV_EVENT_FOCUSED");
      break;
    case LV_EVENT_DEFOCUSED:
      Serial.println("LV_EVENT_DEFOCUSED");
      break;
    case LV_EVENT_LEAVE:
      Serial.println("LV_EVENT_LEAVE");
      break;
    case LV_EVENT_HIT_TEST:
      Serial.println("LV_EVENT_HIT_TEST");
      break;
    case LV_EVENT_INDEV_RESET:
      Serial.println("LV_EVENT_INDEV_RESET");
      break;
    case LV_EVENT_HOVER_OVER:
      Serial.println("LV_EVENT_HOVER_OVER");
      break;
    case LV_EVENT_HOVER_LEAVE:
      Serial.println("LV_EVENT_HOVER_LEAVE");
      break;
    case LV_EVENT_COVER_CHECK:
      Serial.println("LV_EVENT_COVER_CHECK");
      break;
    case LV_EVENT_REFR_EXT_DRAW_SIZE:
      Serial.println("LV_EVENT_REFR_EXT_DRAW_SIZE");
      break;
    case LV_EVENT_DRAW_MAIN_BEGIN:
      Serial.println("LV_EVENT_DRAW_MAIN_BEGIN");
      break;
    case LV_EVENT_DRAW_MAIN:
      Serial.println("LV_EVENT_DRAW_MAIN");
      break;
    case LV_EVENT_DRAW_MAIN_END:
      Serial.println("LV_EVENT_DRAW_MAIN_END");
      break;
    case LV_EVENT_DRAW_POST_BEGIN:
      Serial.println("LV_EVENT_DRAW_POST_BEGIN");
      break;
    case LV_EVENT_DRAW_POST:
      Serial.println("LV_EVENT_DRAW_POST");
      break;
    case LV_EVENT_DRAW_POST_END:
      Serial.println("LV_EVENT_DRAW_POST_END");
      break;
    case LV_EVENT_DRAW_TASK_ADDED:
      Serial.println("LV_EVENT_DRAW_TASK_ADDED");
      break;
    case LV_EVENT_VALUE_CHANGED:
      Serial.println("LV_EVENT_VALUE_CHANGED");
      break;
    case LV_EVENT_INSERT:
      Serial.println("LV_EVENT_INSERT");
      break;
    case LV_EVENT_REFRESH:
      Serial.println("LV_EVENT_REFRESH");
      break;
    case LV_EVENT_READY:
      Serial.println("LV_EVENT_READY");
      break;
    case LV_EVENT_CANCEL:
      Serial.println("LV_EVENT_CANCEL");
      break;
    case LV_EVENT_CREATE:
      Serial.println("LV_EVENT_CREATE");
      break;
    case LV_EVENT_DELETE:
      Serial.println("LV_EVENT_DELETE");
      break;
    case LV_EVENT_CHILD_CHANGED:
      Serial.println("LV_EVENT_CHILD_CHANGED");
      break;
    case LV_EVENT_CHILD_CREATED:
      Serial.println("LV_EVENT_CHILD_CREATED");
      break;
    case LV_EVENT_CHILD_DELETED:
      Serial.println("LV_EVENT_CHILD_DELETED");
      break;
    case LV_EVENT_SCREEN_UNLOAD_START:
      Serial.println("LV_EVENT_SCREEN_UNLOAD_START");
      break;
    case LV_EVENT_SCREEN_LOAD_START:
      Serial.println("LV_EVENT_SCREEN_LOAD_START");
      break;
    case LV_EVENT_SCREEN_LOADED:
      Serial.println("LV_EVENT_SCREEN_LOADED");
      break;
    case LV_EVENT_SCREEN_UNLOADED:
      Serial.println("LV_EVENT_SCREEN_UNLOADED");
      break;
    case LV_EVENT_SIZE_CHANGED:
      Serial.println("LV_EVENT_SIZE_CHANGED");
      break;
    case LV_EVENT_STYLE_CHANGED:
      Serial.println("LV_EVENT_STYLE_CHANGED");
      break;
    case LV_EVENT_LAYOUT_CHANGED:
      Serial.println("LV_EVENT_LAYOUT_CHANGED");
      break;
    case LV_EVENT_GET_SELF_SIZE:
      Serial.println("LV_EVENT_GET_SELF_SIZE");
      break;
    case LV_EVENT_INVALIDATE_AREA:
      Serial.println("LV_EVENT_INVALIDATE_AREA");
      break;
    case LV_EVENT_RESOLUTION_CHANGED:
      Serial.println("LV_EVENT_RESOLUTION_CHANGED");
      break;
    case LV_EVENT_COLOR_FORMAT_CHANGED:
      Serial.println("LV_EVENT_COLOR_FORMAT_CHANGED");
      break;
    case LV_EVENT_VSYNC:
      Serial.println("LV_EVENT_VSYNC");
      break;


    // Check if these events exist in your LVGL version by looking at lv_event.h
    // Uncomment them if they are available
    /*
      case LV_EVENT_DRAG_BEGIN:
      Serial.println("LV_EVENT_DRAG_BEGIN");
      break;
      case LV_EVENT_DRAG_END:
      Serial.println("LV_EVENT_DRAG_END");
      break;
      case LV_EVENT_DRAG_THROW_BEGIN:
      Serial.println("LV_EVENT_DRAG_THROW_BEGIN");
      break;
      case LV_EVENT_PREPROCESS:
      Serial.println("LV_EVENT_PREPROCESS");
      break;
    */
    default:
      Serial.print("UNKNOWN_EVENT: ");
      Serial.println(event);
      break;
  }
}
