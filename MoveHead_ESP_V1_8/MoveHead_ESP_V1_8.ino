// MoveHead_ESP_V1_8
/*
 Board AI Thinker ESP-CAM
 This module requires ESP32 ver 1.0.4 to run
 .h files for this is located in C:\Users\HP\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4

 This works identifying the face position without the need of server
 connected to the Adafruit-PWM-Servo-Driver-Library
 requires:
 'Adafruit PWM Servo Driver Library' ver 3.0.2
 'Adafruit GFX Library' Ver 1.11.9
 'Adafruit SSD1306' by Adafruit Library Ver 2.5.10

**/

// camera libraries
#include <ArduinoWebsockets.h> // C:\Users\HP\Documents\Arduino\libraries\ArduinoWebsockets\src
#include "esp_camera.h" //C:\Users\HP\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\esp32-camera
#include "fb_gfx.h"     //C:\Users\HP\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\fb_gfx
#include "fd_forward.h" //C:\Users\HP\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\esp-face

//  PCA9685 16-Channel 12-Bit PWM Servo Driver , https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Adafruit_PWMServoDriver.h>  // this needs Adafruit PWM Servo Driver Library ver 3.0.1
#define SERVOMIN 125 // this is the minimum pulse length count out of 4096
#define SERVOMAX 575 // this is the maximum pulse length count out of 4096
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
uint8_t servonum = 0;
uint8_t numberOfServos = 6;

//#include <Wire.h>
#include <Adafruit_GFX.h> // Adafruit GFX Library Ver 1.11.9
#include <Adafruit_SSD1306.h> // Adafruit SS1306 by Adafruit Library Ver 2.5.9

 // ESP32-CAM doesn't have dedicated i2c pins, so we define our own. Let's choose 15 and 14
#define I2C_SDA 15
#define I2C_SCL 14
TwoWire I2Cbus = TwoWire(0);

// Display defines
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

// define parameters
int pos;                //temporary servo position
int speed = 10;              //Servo speed
int i;                  // index i for loops
int NewPositionX=0;     // New servo Pan position 
int NewPositionY=0;     // New servo Tilt position 
int NewWaistPosition=0; // New servo Waist position 
int MinTilt = 140;      // 0 Degree Minimum Tilt pulse width
int ZeroTilt = 200;     //20 Degree Zero Tilt pulse width
int MaxTilt = 250;      //55 Degree Maximum Tilt pulse width -- 110 steps
int MinPan = 190;       //30 Degree Minimum Pan pulse width
int ZeroPan = 350;      //90 Degree Zero Pan pulse width
int MaxPan = 510;       //150 Degree Maximum Pan pulse width -- 320 steps
int ScreenPosX = 0;     // variable to hold face tracked X position
int ScreenPosY=0;       // variable to hold face tracked Y position
int ScreenSizeX = 320;  // Screen Frame size X
int ScreenSizeY=240;    // Screen Frame size Y

int OldPositionX = ZeroPan;     // Variable to hold previous Pan Pulse width
int OldPositionY = ZeroTilt;    // Variable to hold previous Tilt Pulse width
int OldWaistPosition=ZeroPan;         // Variable to hold previous Waist Pulse width

int NoActivity = 0;     // variable to hold number of loops wutg no face detected


// Define camera_pins for CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22 
 
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();
 
 
void setup()
{
  Serial.begin(115200);
  Serial.println("Loadong...");
  Serial.println();

  Wire.begin(15, 14); // SDA on GPIO15, SCL on GPIO14


  // Initialize I2C with our defined pins
  I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);
  Serial.println("Initialize display");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.printf("SSD1306 OLED display failed to initalize.\nCheck that display SDA is connected to pin %d and SCL connected to pin %d\n", I2C_SDA, I2C_SCL);
    while (true);
  }


  pwm.begin();
  pwm.setPWMFreq(60); 
  delay(10);

  Serial.println("Show 'Initialising....' on display");
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.print("Initialise");
  display.display();

// reset to zero position
//  pwm.setPWM(0, 0, MinPan); // Move Pan Servo from Zero
//  delay(50);
//  pwm.setPWM(1, 0, MinTilt); // Move Tilt Servo from Zero
//  delay(500); 
  MoveHead(ScreenSizeX/2, ScreenSizeY/2, 10);
  MoveWaist (ScreenSizeX/2, 10);
  delay(1000); 

//talt head back
  MoveHead(ScreenSizeX/2, 0, 10);
  delay(100);

//talt head forward
  MoveHead(ScreenSizeX/2, ScreenSizeY, 10);
  delay(100);

//talt head Zero
  MoveHead(ScreenSizeX/2, ScreenSizeY/2, 10);
  delay(100);

//Pan Waist Right
  MoveWaist(0, 10);
  delay(100);

//Pan Waist Left
  MoveWaist(ScreenSizeX, 10);
  delay(100);

//Pan Waist Zero
  MoveWaist(ScreenSizeX/2, 10);
  delay(100);

//Pan head Right
  MoveHead(0, ScreenSizeY/2, 5);
  delay(100);

//Pan head Left
  MoveHead(ScreenSizeX, ScreenSizeY/2, 5);
  delay(100);

//Pan head Zero
  MoveHead(ScreenSizeX/2, ScreenSizeY/2, 5);
  delay(500); 

  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Head V1.8 ");
  display.display();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
 
 /* Apart from QVGA the rest don't work
  FRAMESIZE_UXGA (1600 x 1200),   FRAMESIZE_QVGA (320 x 240),   FRAMESIZE_CIF (352 x 288),   FRAMESIZE_VGA (640 x 480)
  FRAMESIZE_SVGA (800 x 600),   FRAMESIZE_XGA (1024 x 768),   FRAMESIZE_SXGA (1280 x 1024)
*/
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);


 }
 
 

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes)
{
  int x, y, w, h, i, half_width, half_height;
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {
 
    // Convoluted way of finding face centre...
    x = ((int)boxes->box[i].box_p[0]);
    w = (int)boxes->box[i].box_p[2] - x + 1;
    half_width = w / 2;
    int face_center_pan = x + half_width; // image frame face centre x co-ordinate
 
    y = (int)boxes->box[i].box_p[1];
    h = (int)boxes->box[i].box_p[3] - y + 1;
    half_height = h / 2;
    int face_center_tilt = y + half_height;  // image frame face centre y co-ordinate
 
    //    assume QVGA 320x240
    //    int sensor_width = 320;
    //    int sensor_height = 240;
    //    int lens_fov = 45
  
    Serial.print("Face:");
    Serial.print(face_center_pan); 
    Serial.print(" X ");
    Serial.print(face_center_tilt);
  
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Detected  ");
  display.display();
  
  MoveHead(ScreenSizeX-face_center_pan, ScreenSizeY-face_center_tilt, 10);

//-----------------------------------------------------------------------------
  }
}
 



 void MoveHead(int ScreenPosX, int ScreenPosY, int speed){
  
// Map the screen positions to Servo positions
NewPositionX = map(ScreenPosX, 0, ScreenSizeX, MinPan, MaxPan);
NewPositionY = map(ScreenPosY, 0, ScreenSizeY, MinTilt, MaxTilt);

// Display X,Y screen coordinate, speed, Pan and Tilt

    display.setTextSize(1);
    display.setCursor(0, 18);
    display.print("x:      Y:            ");
    display.setCursor(15, 18);
    display.print(ScreenPosX); 
    display.setCursor(60, 18);
    display.print(ScreenPosY);
    display.setTextColor(WHITE, BLACK);

    display.setTextSize(2);
    display.setCursor(0, 30);
    display.print("Pan:                ");
    display.setCursor(65, 30);
    display.print(NewPositionY); //pan_center

    display.setCursor(0, 50);
    display.print("Tilt:                   ");
    display.setCursor(65, 50);
    display.print(NewPositionX); //tilt_center
    display.display();

// New Routine to move diagnal
if (NewPositionX > OldPositionX){
 for(pos = OldPositionX; pos < NewPositionX; pos++) // in steps of 1 degree
  {  
    pwm.setPWM(0,0,pos);
    if (NewPositionY > OldPositionY){
      pwm.setPWM(1,0,OldPositionY+1);   // stepped position
      OldPositionY = OldPositionY+1;}
    if (NewPositionY < OldPositionY){
      pwm.setPWM(1,0,OldPositionY-1);   // stepped position
      OldPositionY = OldPositionY-1;}
    delay(speed);             // mini delay for each step in milli sec
  }
    delay(100);
}

if (NewPositionX < OldPositionX){
 for(pos = OldPositionX ; pos > NewPositionX; pos --) // in steps of 1 degree
  {  
    pwm.setPWM(0,0,pos);   // stepped position 
      if (NewPositionY > OldPositionY){
      pwm.setPWM(1,0,OldPositionY+1);   // stepped position
      OldPositionY = OldPositionY+1;}
    if (NewPositionY < OldPositionY){
      pwm.setPWM(1,0,OldPositionY-1);   // stepped position
      OldPositionY = OldPositionY-1;}
    delay(speed);             // mini delay for each step in milli sec
  }
  delay(100);
}
 OldPositionX = NewPositionX;

// ensure that the Y position is complete
 if (NewPositionY > OldPositionY){
 for(pos = OldPositionY; pos < NewPositionY; pos ++) // in steps of 1 degree
  {  
    pwm.setPWM(1,0,pos);   // stepped position 
    delay(speed);             // mini delay for each step in milli sec
  }
}

if (NewPositionY < OldPositionY){
 for(pos = OldPositionY ; pos > NewPositionY; pos --) // in steps of 1 degree
  {  
    pwm.setPWM(1,0,pos);   // stepped position 
    delay(speed);             // mini delay for each step in milli sec
  }
}
 OldPositionY = NewPositionY;
delay (100);


}

 void MoveWaist(int ScreenPosX, int speed){
   
// Map the screen positions to Servo positions
NewWaistPosition = map(ScreenPosX, 0, ScreenSizeX, MinPan, MaxPan);;


// ensure that the Y position is complete
 if (NewWaistPosition > OldWaistPosition){
 for(pos = OldWaistPosition; pos < NewWaistPosition; pos ++) // in steps of 1 degree
  {  
    pwm.setPWM(2,0,pos);   // stepped position 
    delay(speed);             // mini delay for each step in milli sec
  }
}

if (NewWaistPosition < OldWaistPosition){
 for(pos = OldWaistPosition ; pos > NewWaistPosition; pos --) // in steps of 1 degree
  {  
    pwm.setPWM(2,0,pos);   // stepped position 
    delay(speed);             // mini delay for each step in milli sec
  }
}
 OldWaistPosition = NewWaistPosition;
delay (100);


}


void loop()
{

 // auto client = socket_server.accept();
  camera_fb_t * fb = NULL;
  dl_matrix3du_t *image_matrix = NULL;
 
  while (true) {
    fb = esp_camera_fb_get();
    image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
 
    fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
 
    box_array_t *net_boxes = NULL;
    net_boxes = face_detect(image_matrix, &mtmn_config);
 
    if (net_boxes) {
      draw_face_boxes(image_matrix, net_boxes);
      free(net_boxes->score);
      free(net_boxes->box);
      free(net_boxes->landmark);
      free(net_boxes);
      NoActivity = 0;
    }
 
 // move randomly if there is a period of inactivity
  if (NoActivity > 100) {
      MoveHead(random(50, ScreenSizeX), random(50, ScreenSizeY), 40);
      MoveWaist(random(50, ScreenSizeY), 40);
      NoActivity = 0;
    }

  if (NoActivity > 10) {
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.print("Searching");
      display.display();
    }


    NoActivity = NoActivity +1;

    esp_camera_fb_return(fb);
    fb = NULL;
    dl_matrix3du_free(image_matrix);
  }
  

}