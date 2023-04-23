#include <Arduino.h>
#include <SPI.h>
#include <lvgl.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "ui.h"
#include <WiFi.h>
#include "main.h"

static const char *TAG = "esp32s2";
const char *ssid = "Neurotoxin2";
const char *password = "Mxbb2Col";
const char *udpAddress = "10.175.1.1";
const int udpPort = 44444;
WiFiUDP udp;

TimerHandle_t wifiReconnectTimer;
hw_timer_t *lv_tick_timer = NULL;
lv_group_t *main_group = NULL;

TaskHandle_t heartbeat;
TaskHandle_t joystic;
TaskHandle_t con_strength;
TaskHandle_t con_info;

int valueX = 0; // to store the X-axis value
int valueY = 0; // to store the Y-axis value
int command = COMMAND_NO;

static lv_indev_t *indev_keypad;

void WiFiEvent(WiFiEvent_t event)
{
  // Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection, reconnect...");
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void connectToWifi()
{
  // Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

static void init_keypad()
{
  pinMode(GPIO_UP_PIN, INPUT_PULLUP);
  // pinMode(GPIO_RT_PIN, INPUT_PULLDOWN);
  pinMode(GPIO_DN_PIN, INPUT_PULLUP);
  pinMode(GPIO_LT_PIN, INPUT_PULLUP);
  pinMode(GPIO_CR_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
}

static void indev_keypad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  // Read input from the GPIO keys
  int up_state = digitalRead(GPIO_UP_PIN);
  int dn_state = digitalRead(GPIO_DN_PIN);
  int lt_state = digitalRead(GPIO_LT_PIN);
  // int rt_state = digitalRead(GPIO_RT_PIN);
  int cr_state = digitalRead(GPIO_CR_PIN);

  // Create an LVGL input data structure
  data->state = LV_INDEV_STATE_REL;
  if (dn_state == LOW)
  {
    data->key = LV_KEY_PREV;
    data->state = LV_INDEV_STATE_PR;
    Serial.println("DOWN");
  }
  else if (up_state == LOW)
  {
    data->key = LV_KEY_NEXT;
    data->state = LV_INDEV_STATE_PR;
    Serial.println("UP");
  }
  else if (cr_state == LOW)
  {
    data->key = LV_KEY_ENTER;
    data->state = LV_INDEV_STATE_PR;
    Serial.println("ENTER");
  }
  else if (lt_state == LOW)
  {
    data->key = LV_KEY_ESC;
    data->state = LV_INDEV_STATE_PR;
    Serial.println("LEFT");
  } // else if(rt_state == HIGH) {
    // data->key = LV_KEY_RIGHT;
    // data->state = LV_INDEV_STATE_PR;
    // Serial.println("RIGHT");
  //}
}

/*Change to your screen resolution*/
static const uint16_t screenWidth = 160;
static const uint16_t screenHeight = 80;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

void IRAM_ATTR onTick()
{
  lv_tick_inc(5);
}

int getStrength()
{
  long rssi = WiFi.RSSI();
  long averageRSSI = 0;

  averageRSSI = 2 * (rssi + 100);
  return averageRSSI;
}

void heartbeat_Task(void *parameter)
{
    for (;;)
    {
        digitalWrite(LED, LOW);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        digitalWrite(LED, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void joystic_Task(void *parameter)
{

  for (;;)
  {
    // read X and Y analog values
    valueX = analogRead(VRX_PIN);
    valueY = analogRead(VRY_PIN);
    // converts the analog value to commands
    // reset commands
    command = COMMAND_NO;

    // check left/right commands
    if (valueX < LEFT_THRESHOLD)
      command = command | COMMAND_LEFT;
    else if (valueX > RIGHT_THRESHOLD)
      command = command | COMMAND_RIGHT;

    // check up/down commands
    if (valueY < UP_THRESHOLD)
      command = command | COMMAND_UP;
    else if (valueY > DOWN_THRESHOLD)
      command = command | COMMAND_DOWN;

    // NOTE: AT A TIME, THERE MAY BE NO COMMAND, ONE COMMAND OR TWO COMMANDS
    lv_img_set_src(ui_JoyImage, &ui_img_joy_normal_png);
    // print command to serial and process command
    if (command & COMMAND_LEFT)
    {
      lv_img_set_src(ui_JoyImage, &ui_img_joy_lf_png);
      Serial.println("J_LEFT");
      uint8_t cmd_buffer[50] = "J_LEFT";
      udp.beginPacket(udpAddress, udpPort);
      udp.write(cmd_buffer, 11);
      udp.endPacket();
    } 

    if (command & COMMAND_RIGHT)
    {
      lv_img_set_src(ui_JoyImage, &ui_img_joy_rt_png);
      Serial.println("J_RIGHT");
      uint8_t cmd_buffer[50] = "J_RIGHT";
      udp.beginPacket(udpAddress, udpPort);
      udp.write(cmd_buffer, 11);
      udp.endPacket();
    } 

    if (command & COMMAND_UP)
    {
      lv_img_set_src(ui_JoyImage, &ui_img_joy_up_png);
      Serial.println("J_UP");
      uint8_t cmd_buffer[50] = "J_UP";
      udp.beginPacket(udpAddress, udpPort);
      udp.write(cmd_buffer, 11);
      udp.endPacket();
    } 

    if (command & COMMAND_DOWN)
    {
      lv_img_set_src(ui_JoyImage, &ui_img_joy_dn_png);
      Serial.println("J_DOWN");
      uint8_t cmd_buffer[50] = "J_DOWN";
      udp.beginPacket(udpAddress, udpPort);
      udp.write(cmd_buffer, 11);
      udp.endPacket();
    } 
    vTaskDelay(10);
  }
}

void conStrength_Task(void *parameter)
{
  char buffer[100];
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      lv_bar_set_value(ui_SignalStrength, getStrength(), LV_ANIM_OFF);
    }
    vTaskDelay(250);
  }
}

void conInfo_Task(void *parameter)
{
  char buffer[100];
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      sprintf(buffer, "%s RSSI: %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      lv_label_set_text(ui_StatusBar, buffer);
    }
    else
    {
      lv_label_set_text(ui_StatusBar, "Disconnected");
    }
    vTaskDelay(1000);
  }
}

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */

  init_keypad();
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  connectToWifi();

  lv_init();

  lv_tick_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(lv_tick_timer, &onTick, true);
  timerAlarmWrite(lv_tick_timer, 5000, true);
  timerAlarmEnable(lv_tick_timer);

  tft.begin();        /* TFT init */
  tft.setRotation(3); /* Landscape orientation, flipped */

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Create the custom indev driver
  static lv_indev_drv_t indev_drv_keypad;
  lv_indev_drv_init(&indev_drv_keypad);
  indev_drv_keypad.type = LV_INDEV_TYPE_KEYPAD;
  indev_drv_keypad.read_cb = indev_keypad_read;

  // Register the custom indev driver
  indev_keypad = lv_indev_drv_register(&indev_drv_keypad);

  ui_init();
  main_group = lv_group_create();
  lv_group_add_obj(main_group, ui_MainScreen);
  lv_indev_set_group(indev_keypad, main_group);
  lv_group_set_default(main_group);
  lv_indev_enable(indev_keypad, true);

  xTaskCreatePinnedToCore(heartbeat_Task, "SYS_HEARTBEAT", 2000, NULL, 1, &heartbeat, 0);
  xTaskCreatePinnedToCore(joystic_Task, "Joystic", 8192, NULL, 1, &joystic, 1);
  xTaskCreatePinnedToCore(conStrength_Task, "ConnectionStrength", 8192, NULL, 1, &con_strength, 0);
  xTaskCreatePinnedToCore(conInfo_Task, "ConnectionStrength", 8192, NULL, 1, &con_info, 0);
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */

  // Serial.println(getStrength());
}
