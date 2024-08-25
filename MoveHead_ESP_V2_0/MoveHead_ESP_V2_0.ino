
/*------------------------------------------------------------------------------------------------------------------ 
  MoveHead_ESP_V2_0 With ADS1115 Servo feedback                                                                   
------------------------------------------------------------------------------------------------------------------ 

 Board AI Thinker ESP-CAM
 This module requires ESP32 ver 1.0.4 to run
 .h files for this is located in C:\Users\'user account name'\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4
  The screen size used is 320X240 (ScreenSizeX = 320; ScreenSizeY=240;)

 This works identifying the face position without the need of server
 It uses Adafruit-PWM-Servo-Driver-Library to control servos
 Servos are modified to access the middle pot pin
 This pin is connected to an analog to digital I2C module to read the voltage as feedback Servo position
 
 In addition to ESP32 (ver 1.0.4) This software requires the following libraries:
  'Adafruit PWM Servo Driver Library' ver 3.0.2
  'Adafruit GFX Library' Ver 1.11.9
  'Adafruit SSD1306' by Adafruit Library Ver 2.5.10
  'ADS115-Driver' Library Ver 1.0.2 by Wh11eRabbitHU

Three servos are used
  Servo 0 = X, Pan 
  Servo 1 = Y, Tilt, nod
  Servo 2 = Waist Control

As ESP32-CAM doesn't have dedicated i2c pins, TwoWire (TwoWire I2Cbus = TwoWire(0);) function is used on 15 (SDA) and 14 (SCL)
  OLED is on I2C bus address 0x3D (normally this is 0x3C)
  PWM servo controller is connected to I2C bus address 0x40
  ADS1115 Analogue to Digital converter used to get voltage feedback from modified servos is on I2C address 0x48

*/


/*------------------------------------------------------------------------------------------------------------------ 
  ESP32-CAM specific libraries and variables used                                                                   
------------------------------------------------------------------------------------------------------------------ */

// ESP32-CAM camera function
#include <ArduinoWebsockets.h> //\Arduino\libraries\ArduinoWebsockets\src
#include "esp_camera.h" //C:\Users\'user account name'\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\esp32-camera
#include "fb_gfx.h"     //C:\Users\'user account name'\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\fb_gfx
#include "fd_forward.h" //C:\Users\'user account name'\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\esp-face

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


/*------------------------------------------------------------------------------------------------------------------ 
  Camera Function                                                                   
------------------------------------------------------------------------------------------------------------------ */
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

/*------------------------------------------------------------------------------------------------------------------ 
  Other libraries and variables used                                                                   
------------------------------------------------------------------------------------------------------------------ */

// PCA9685 16-Channel 12-Bit PWM Servo Driver , https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Adafruit_PWMServoDriver.h>  // this needs Adafruit PWM Servo Driver Library ver 3.0.1
#define SERVOMIN 125 // this is the minimum pulse length count out of 4096
#define SERVOMAX 575 // this is the maximum pulse length count out of 4096
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);   //Servo board is at I2C address 0x40
uint8_t servonum = 0;
uint8_t numberOfServos = 6;

// Instead of '#include <Wire.h>' TwoWire is used
// ESP32-CAM doesn't have dedicated i2c pins, so we define our own. Let's choose 15 and 14
#define I2C_SDA 15
#define I2C_SCL 14
TwoWire I2Cbus = TwoWire(0);

// OLED Display parameters
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3D

#include <Adafruit_GFX.h> // Adafruit GFX Library Ver 1.11.9
#include <Adafruit_SSD1306.h> // Adafruit SS1306 by Adafruit Library Ver 2.5.9
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

// Define variables and parameters
int pos;                      //temporary servo position
int speed = 10;               //Servo speed
int i;                        // index i for loops
int NewPositionX=0;           // New servo Pan position 
int NewPositionY=0;           // New servo Tilt position 
int NewWaistPosition=0;       // New servo Waist position 
int MinTilt = 140;            // 0 Degree Minimum Tilt pulse width
int ZeroTilt = 200;           //20 Degree Zero Tilt pulse width
int MaxTilt = 250;            //55 Degree Maximum Tilt pulse width -- 110 steps
int MinPan = 190;             //30 Degree Minimum Pan pulse width
int ZeroPan = 350;            //90 Degree Zero Pan pulse width
int MaxPan = 510;             //150 Degree Maximum Pan pulse width -- 320 steps
int ScreenPosX = 0;           // variable to hold face tracked X position
int ScreenPosY=0;             // variable to hold face tracked Y position
int ScreenSizeX = 320;        // Screen Frame size X
int ScreenSizeY=240;          // Screen Frame size Y
int OldPositionX = ZeroPan;   // Variable to hold previous Pan Pulse width
int OldPositionY = ZeroTilt;  // Variable to hold previous Tilt Pulse width
int OldWaistPosition=ZeroPan; // Variable to hold previous Waist Pulse width
int NoActivity = 0;           // variable to hold number of loops wutg no face detected


// initialising the ADS1115
#include "ADS1115-Driver.h"
ADS1115 ads1115 = ADS1115(0x48);  //ADS115 is at I2C address 0x48
  int MinV=0;
  int MaxV = 3332;
uint16_t readValue(uint8_t input) 
  {
	  ads1115.setMultiplexer(input);
	  ads1115.startSingleConvertion();
	  delayMicroseconds(25); // The ADS1115 needs to wake up from sleep mode and usually it takes 25 uS to do that
	  while (ads1115.getOperationalStatus() == 0);
	  return ads1115.readConvertedValue();
  }




/*------------------------------------------------------------------------------------------------------------------ 
  Setup                                                                   
------------------------------------------------------------------------------------------------------------------ */
void setup()
{
  Serial.begin(115200);
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

// Setting I2C servo
  pwm.begin();
  pwm.setPWMFreq(60); 
  delay(10);

// Settingup the ADS1115
	ads1115.reset();
	ads1115.setDeviceMode(ADS1115_MODE_SINGLE);
	ads1115.setDataRate(ADS1115_DR_250_SPS);
	ads1115.setPga(ADS1115_PGA_4_096);

  Serial.println("Show 'Initialising....' on display");
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
  OLED_Heading("Head V2.0 ");

  OldPositionX = getPWM(0);       // Variable to hold previous Pan Pulse width
  OldPositionY = getPWM(1);       // Variable to hold previous Tilt Pulse width
  OldWaistPosition = getPWM(2);   // Variable to hold previous Waist Pulse width
  OLED_Servo();
  delay(2000);

  InitialiseHead();  //initialise head to zero and move to max min on all servos

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
  if (psramFound()) 
    {
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
    } 
  else 
    {
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
  if (err != ESP_OK) 
    {
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
 

/*------------------------------------------------------------------------------------------------------------------ 
  Main loop                                                                   
------------------------------------------------------------------------------------------------------------------ */
void loop()
{

 // auto client = socket_server.accept();
  camera_fb_t * fb = NULL;
  dl_matrix3du_t *image_matrix = NULL;
 
  while (true) 
  {
    fb = esp_camera_fb_get();
    image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
 
    fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
 
    box_array_t *net_boxes = NULL;
    net_boxes = face_detect(image_matrix, &mtmn_config);
 
    if (net_boxes) 
      {
        draw_face_boxes(image_matrix, net_boxes);
        free(net_boxes->score);
        free(net_boxes->box);
        free(net_boxes->landmark);
        free(net_boxes);
        NoActivity = 0;
      }
 
    // move randomly if there is a period of inactivity
    if (NoActivity > 100) 
      {
        MoveHead(random(50, ScreenSizeX), random(50, ScreenSizeY), 40);
        MoveWaist(random(50, ScreenSizeY), 40);
        NoActivity = 0;
      }
    if (NoActivity > 10) 
      {
        OLED_Heading("Searching ");
        // Clear Camera position display
        display.setTextSize(2);
        display.setCursor(0, 50);
        display.print("           ");
        display.setTextColor(WHITE, BLACK);
        display.display();
      }

    NoActivity = NoActivity +1;

    esp_camera_fb_return(fb);
    fb = NULL;
    dl_matrix3du_free(image_matrix);
  }
}



/*------------------------------------------------------------------------------------------------------------------ 
  Camera Functions                                                                   
------------------------------------------------------------------------------------------------------------------ */
static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes)
{
  int x, y, w, h, i, half_width, half_height;
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) 
    {
      // Convoluted way of finding face centre...
      // 0,0 is the top right corner of the screen
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
  
      OLED_Heading(" Detected ");
      OLED_XY(320-face_center_pan, 240-face_center_tilt);
      MoveHead(ScreenSizeX-face_center_pan, ScreenSizeY-face_center_tilt, 10);

    }
  }
 
/*------------------------------------------------------------------------------------------------------------------ 
  Initialise the Head servos                                                                   
------------------------------------------------------------------------------------------------------------------ */
 void InitialiseHead()
{
  // reset to zero position
  // Get the current position of the servos from the feedback and set it as the old position
  // Without this part, the software is not in control of the speed of servos
  DisplayServoData();  // see what the servo voltages are for calibration of  MinV=0; and MaxV = 3332;
  OLED_Heading("Initialise");
  
  MoveHead(ScreenSizeX/2, ScreenSizeY/2, 10);
  MoveWaist (ScreenSizeX/2, 10);
  OLED_Servo();

  Serial.print("Centre --- ");
  DisplayServoData();  // see what the servo voltages are for calibration of  MinV=0; and MaxV = 3332;
  delay(100);

  //talt head back
  MoveHead(ScreenSizeX/2, 0, 10);

  //talt head forward
  MoveHead(ScreenSizeX/2, ScreenSizeY, 10);

  //talt head Zero
  MoveHead(ScreenSizeX/2, ScreenSizeY/2, 10);

  //Pan Waist Right
  MoveWaist(0, 10);

  //Pan Waist Left
  MoveWaist(ScreenSizeX, 10);

  //Pan Waist Zero
  MoveWaist(ScreenSizeX/2, 10);

  //Pan head Right
  MoveHead(0, ScreenSizeY/2, 5);

  //Pan head Left
  MoveHead(ScreenSizeX, ScreenSizeY/2, 5);

  //Pan head Zero
  MoveHead(ScreenSizeX/2, ScreenSizeY/2, 5);

  Serial.print("Centre Position - ");
  DisplayServoData();

}

/*------------------------------------------------------------------------------------------------------------------ 
  OLED display Functions                                                                   
------------------------------------------------------------------------------------------------------------------ */
void OLED_Heading(String Heading)
{
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(Heading);
  display.display();
}

// second line showing the feedback voltages of the three servos
void OLED_Servo()
{
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("P:     T:     W:    ");
    display.setCursor(10, 20);
    display.print(int(getMilliVolt(0))); 
    display.setCursor(52, 20);
    display.print(int(getMilliVolt(1))); 
    display.setCursor(94, 20);
    display.print(int(getMilliVolt(2)));
    display.setTextColor(WHITE, BLACK);
    display.display();
}

// Third line showing the commanded / actual servo positions
void OLED_CurrentPos()
{
    display.setTextSize(1);
    display.setCursor(0, 31);
    display.print("                    ");
    display.setCursor(10, 31);
    display.print(NewPositionX); 
    display.setCursor(52, 31);
    display.print(NewPositionY); 
    display.setCursor(94, 31);
    display.print(NewWaistPosition);
    display.setTextColor(WHITE, BLACK);
    display.display();
}

// last line showing the camera Detected X Y position
void OLED_XY(int CameraX, int CameraY)
{
    display.setTextSize(2);
    display.setCursor(0, 50);
    display.print("X         ");
    display.setCursor(10, 50);
    display.print(":");
    display.setCursor(65, 50);
    display.print("Y");
    display.setCursor(75, 50);
    display.print(":");
    display.setCursor(19, 50);
    display.print(CameraX); 
    display.setCursor(85, 50);
    display.print(CameraY); 
    display.setTextColor(WHITE, BLACK);
    display.display();
}

/*------------------------------------------------------------------------------------------------------------------ 
  Move head based on screen position                                                                   
------------------------------------------------------------------------------------------------------------------ */
 void MoveHead(int ScreenPosX, int ScreenPosY, int speed)
 {
  
  // Map the screen positions to Servo positions
  NewPositionX = map(ScreenPosX, 0, ScreenSizeX, MinPan, MaxPan);
  NewPositionY = map(ScreenPosY, 0, ScreenSizeY, MinTilt, MaxTilt);

  // Display X,Y screen coordinate, speed, Pan and Tilt
  OLED_CurrentPos();
  OLED_Servo();

  // New Routine to move diagnal
  if (NewPositionX > OldPositionX)
  {
    for(pos = OldPositionX; pos < NewPositionX; pos++) // in steps of 1 degree
      {  
        pwm.setPWM(0,0,pos);
        if (NewPositionY > OldPositionY)
          {
            pwm.setPWM(1,0,OldPositionY+1);   // stepped position
            OldPositionY = OldPositionY+1;
          }
        if (NewPositionY < OldPositionY)
          {
            pwm.setPWM(1,0,OldPositionY-1);   // stepped position
            OldPositionY = OldPositionY-1;
          }
        delay(speed);             // mini delay for each step in milli sec
      }
    delay(100);
  }

  if (NewPositionX < OldPositionX)
    {
      for(pos = OldPositionX ; pos > NewPositionX; pos --) // in steps of 1 degree
        {  
          pwm.setPWM(0,0,pos);   // stepped position 
          if (NewPositionY > OldPositionY)
            {
              pwm.setPWM(1,0,OldPositionY+1);   // stepped position
              OldPositionY = OldPositionY+1;
            }
          if (NewPositionY < OldPositionY)
            {
              pwm.setPWM(1,0,OldPositionY-1);   // stepped position
              OldPositionY = OldPositionY-1;
            }
          delay(speed);             // mini delay for each step in milli sec
        }
        delay(100);
    }
  OldPositionX = NewPositionX;

// ensure that the Y position is complete
  if (NewPositionY > OldPositionY)
    {
      for(pos = OldPositionY; pos < NewPositionY; pos ++) // in steps of 1 degree
        {  
          pwm.setPWM(1,0,pos);   // stepped position 
          delay(speed);             // mini delay for each step in milli sec
        }
    }

  if (NewPositionY < OldPositionY)
    {
      for(pos = OldPositionY ; pos > NewPositionY; pos --) // in steps of 1 degree
        {  
          pwm.setPWM(1,0,pos);   // stepped position 
          delay(speed);             // mini delay for each step in milli sec
        }
    }
  OldPositionY = NewPositionY;
  // Display X,Y screen coordinate, speed, Pan and Tilt
  OLED_CurrentPos();
  OLED_Servo();
  delay (100);

}

/*------------------------------------------------------------------------------------------------------------------ 
  Move Waist servo to specific screen position                                                                  
------------------------------------------------------------------------------------------------------------------ */
 void MoveWaist(int ScreenPosX, int speed)
 {  
  // Map the screen positions to Servo positions
  NewWaistPosition = map(ScreenPosX, 0, ScreenSizeX, MinPan, MaxPan);

  // ensure that the Y position is complete
  if (NewWaistPosition > OldWaistPosition)
    {
      for(pos = OldWaistPosition; pos < NewWaistPosition; pos ++) // in steps of 1 degree
        {  
          pwm.setPWM(2,0,pos);   // stepped position 
          delay(speed);             // mini delay for each step in milli sec
        }
    }

  if (NewWaistPosition < OldWaistPosition)
    {
      for(pos = OldWaistPosition ; pos > NewWaistPosition; pos --) // in steps of 1 degree
        {  
          pwm.setPWM(2,0,pos);   // stepped position 
          delay(speed);             // mini delay for each step in milli sec
        }
    }
  OldWaistPosition = NewWaistPosition;
  OLED_CurrentPos();
  OLED_Servo();
  delay (100);
}



/*------------------------------------------------------------------------------------------------------------------ 
  ADS1115 getting Servos feedback information to set zero slowly                                                                   
------------------------------------------------------------------------------------------------------------------ */
int getMilliVolt(int Channel)
  {
  //return the voltage in mV
    return readValue(Channel + 4);
  }

int getAngle(int Channel)
  {
  //return the angle in degrees
    return map (readValue(Channel + 4), MinV,MaxV, 0,360);
  }

int getPWM(int Channel)
  {
  //return the angle in degrees
    return map (readValue(Channel + 4), MinV,MaxV, SERVOMIN,SERVOMAX);
  }


/*------------------------------------------------------------------------------------------------------------------ 
  Display voltage information on Serial Print to calibrate Vmin and Vmax (all servos are the same)                                                                  
------------------------------------------------------------------------------------------------------------------ */
void DisplayServoData()
{
  Serial.print("Servo - Min PWM: ");
  Serial.print(SERVOMIN);     
  Serial.print(" Max PWM: ");
  Serial.println(SERVOMAX);    
  Serial.print("Pan: ");
  Serial.print(getPWM(0));     
  Serial.print("PWM V: ");  
  Serial.print(getMilliVolt(0));     
  Serial.print("mV  Tilt: ");  
  Serial.print(getPWM(1));     
  Serial.print("PWM V: ");  
  Serial.print(getMilliVolt(1));  
  Serial.print("mV  Waist: ");   
  Serial.print(getPWM(2));  
  Serial.print("PWM V: ");  
  Serial.print(getMilliVolt(2));  
  Serial.println("mV");    
}

