/*
    Main.ino - MP Example for MP Mutex
    Copyright 2019 Sony Semiconductor Solutions Corporation

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <MP.h>
#include <MPMutex.h>

#include <BMI160Gen.h>


#include <SDHCI.h>
#include <Audio.h>
#include <Camera.h>
#include <SPI.h>

#include <stdio.h>

#include "Adafruit_ILI9341.h"


// For the Adafruit shield, these are the default.
#define TFT_RST 18
#define TFT_DC  25
#define TFT_CS  -1

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

SDClass SD;

void CamCB(CamImage img)
{
  /* Check the img instance is available or not. */

  if (img.isAvailable())
  {
    // Convert color format to be able to show on the LCD
    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);

    // Draw manipulated frame buffer on the LCD
    tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 240);
    
  tft.setCursor(3,140);
  //tft.setCursor(20,140);
  //tft.setCursor(40,140);
  tft.setTextSize(3);
  tft.setTextColor(0x780F);
  tft.println("Running Pig");
  }
}


//////////////////////////////////////////////





//////////////////////超声波


int Echo = 12;  // Echo回声脚(P2.0)
int Trig = 11; //  Trig 触发脚(P2.1)

int Distance = 0;

int key = 4;              //定义按键 数字4 接口

////////////////////////////////


SDClass theSD;
AudioClass *theAudio;

File myFile;
static int count_num = 0;
static int add = 500;

static int bobo = 0;
bool ErrEnd = false;
// pin settings
int Left_motor_en       = 7;
int Left_motor_go       = 5; // PWM
int Left_motor_back     = 9; // PWM

int Right_motor_en      = 8;
int Right_motor_go      = 6; // PWM
int Right_motor_back    = 3; // PWM

int count = 0;
const int SensorRight_2 = A0;     //右红外传感器(A4 OUT2 P1.4)
const int SensorLeft_2 = A1;     //左红外传感器(A5 OUT1 P1.5)


const int Sensorm2 = A2;//左外
const int Sensorm3 = A3;//左内
const int Sensorm4 = A4;//右内
const int Sensorm5 = A5;//右外


int rl;
int rr;
int zuowai;
int zuonei;
int younei;
int youwai;


static void audio_attention_cb(const ErrorAttentionParam *atprm)
{
  puts("Attention!");

  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
  {
    ErrEnd = true;
  }
}

void set_up()
{
  /////

  theAudio = AudioClass::getInstance();

  theAudio->begin(audio_attention_cb);

  puts("initialization Audio Library");

  /* Set clock mode to normal */
  theAudio->setRenderingClockMode(AS_CLKMODE_NORMAL);

  /* Set output device to speaker with first argument.
     If you want to change the output device to I2S,
     specify "AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT" as an argument.
     Set speaker driver mode to LineOut with second argument.
     If you want to change the speaker driver mode to other,
     specify "AS_SP_DRV_MODE_1DRIVER" or "AS_SP_DRV_MODE_2DRIVER" or "AS_SP_DRV_MODE_4DRIVER"
     as an argument.
  */
  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, AS_SP_DRV_MODE_LINEOUT);

  /*
     Set main player to decode stereo mp3. Stream sample rate is set to "auto detect"
     Search for MP3 decoder in "/mnt/sd0/BIN" directory
  */
  //  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/spif/BIN", AS_SAMPLINGRATE_AUTO, AS_CHANNEL_STEREO);
  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/spif/BIN", 44100, AS_CHANNEL_STEREO);

  /* Verify player initialize */
  if (err != AUDIOLIB_ECODE_OK)
  {
    printf("Player0 initialize error\n");
    exit(1);
  }

  /* Open file placed on SD card */
  myFile = theSD.open("Jupiter.mp3");

  /* Verify file open */
  if (!myFile)
  {
    printf("File open error\n");
    exit(1);
  }
  printf("Open! %d\n", myFile);

  /* Send first frames to be decoded */

}

void read_data()
{
  rr = analogRead(SensorRight_2);
  rl = analogRead(SensorLeft_2);

  zuowai = analogRead(Sensorm2);
  zuonei = analogRead(Sensorm3);
  younei = analogRead(Sensorm4);
  youwai = analogRead(Sensorm5);
}
void print_data()
{
  Serial.print("Sensorm2:"); Serial.println(zuowai);
  Serial.print("Sensorm3:"); Serial.println(zuonei);
  Serial.print("Sensorm4:"); Serial.println(younei);
  Serial.print("Sensorm5:"); Serial.println(youwai);
  Serial.print("SensorRight_2:"); Serial.println(rr);
  Serial.print("SensorLeft_2:"); Serial.println(rl);
}
//////////////////////////////////beep音

void beep(int s)
{


  theAudio->setBeep(1, -40, s);
  //usleep(s * 10);
  return ;
}
void tingzhi()
{
  theAudio->setBeep(0, 0, 0);

  return ;
}



void keysacn()//按键扫描
{
  int val;




  val = digitalRead(key); //读取数字3 口电平值赋给val
  while (digitalRead(key)) //当按键没被按下时，一直循环
  {
    val = digitalRead(key); //此句可省略，可让循环跑空
  }
  while (!digitalRead(key)) //当按键被按下时
  {
    delay(10);  //延时10ms
    val = digitalRead(key); //读取数字3 口电平值赋给val
    if (val == LOW) //第二次判断按键是否被按下
    {
      delay(50);
      while (!digitalRead(key));   //蜂鸣器停止
    }
  }
}

////////////////////////////////////////
/* Create a MPMutex object */
MPMutex mutex(MP_MUTEX_ID0);

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  //////////////////////////////////////
  tft.begin(40000000);
  tft.setRotation(3);


  /* begin() without parameters means that
     number of buffers = 1, 30FPS, QVGA, YUV 4:2:2 format */

  Serial.println("Prepare camera");
  theCamera.begin(1);

  /* Start video stream.
     If received video stream data from camera device,
      camera library call CamCB.
  */

  Serial.println("Start streaming");
  theCamera.startStreaming(true, CamCB);

  /* Auto white balance configuration */

  Serial.println("Set Auto white balance parameter");
  theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);

  
  int ret = 0;
  int subid = 1;

  ret = MP.begin(subid);
  if (ret < 0) {
    printf("MP.begin(%d) error = %d\n", subid, ret);
  }
  /* Boot SubCore */
  set_up();

  err_t err = theAudio->writeFrames(AudioClass::Player0, myFile);

  if ((err != AUDIOLIB_ECODE_OK) && (err != AUDIOLIB_ECODE_FILEEND))
  {
    printf("File Read Error! =%d\n", err);
    myFile.close();
    exit(1);
  }
  puts("Play!");

  /* Main volume set to -16.0 dB */
  theAudio->setVolume(-160);
  ////////////////////////////////////


  BMI160.begin();
  /* Enable Shock Detection */
  BMI160.setDetectionThreshold(CURIE_IMU_SHOCK, 1200); // 1.5g = 1500 mg
  BMI160.setDetectionDuration(CURIE_IMU_SHOCK, 50);   // 50ms
  BMI160.interrupts(CURIE_IMU_SHOCK);

  pinMode(Echo, INPUT);    // 定义超声波输入脚
  pinMode(Trig, OUTPUT);   // 定义超声波输出脚

}

void Distance_test()   // 量出前方距离
{
  digitalWrite(Trig, LOW);   // 给触发脚低电平2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
  delayMicroseconds(10);//10us
  digitalWrite(Trig, LOW);    // 持续给触发脚低电
  float Fdistance = pulseIn(Echo, HIGH);  // 读取高电平时间(单位：微秒)
  Fdistance = Fdistance / 58;    //为什么除以58等于厘米，  Y米=（X秒*344）/2
  // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
  //输出距离（单位：厘米）
  Serial.println(Fdistance);         //显示距离
  Distance = Fdistance;
}
void brake()
{
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 0);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 0);
}
void loop()
{
  if (bobo == 1)
  {
    Distance_test();
    if (Distance < 20)
    {
      brake();
      exit(1);
    }
  }



  static bool Stop_In = false;
  if (BMI160.getInterruptStatus(CURIE_IMU_SHOCK)) {
    if (BMI160.shockDetected(Z_AXIS, POSITIVE)) {

      count++;
    }
  }


  if (count != count_num)
  {

    if (count < 10)
      beep(350);
    else if (count < 20)
      beep(370);
    else if (count < 30)
      beep(400);
    else if (count < 40)
      beep(450);
    else if (count < 50)
      beep(500);
    else if (count < 60)
      beep(550);
    else if (count < 70)
      beep(600);
    else if (count < 80)
      beep(650);
    else if (count < 50)
      beep(700);
    else
      beep(add);
    add += 50;
    if (add > 900)
      add = 300;
    delay(100);
    tingzhi();
    Serial.println("count:"); Serial.println(count);
    count_num = count;

    bobo = 1;

  }

  read_data();
  print_data();
  if ( zuonei > 200 && zuowai > 200 && youwai > 200 && younei > 200 && rl > 200 &&  rr > 200)
  {
    int time1 = millis();
    while ( zuonei > 200 || zuowai > 200 && youwai > 200 && younei > 200 && rl > 200 &&  rr > 200)
    {
      read_data();
    }
    int meter = millis() - time1;
    if (meter < 100)
    {
      beep(add);
      add = add + 100;
      Serial.print("哈哈哈");
      delay(100);
      tingzhi();
    }
    //    else
    //    {
    //      static int set_beep = 0;
    //      if (set_beep == 0)
    //      {
    //
    //        theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, 0, 0);
    //        add=add+100;
    //      }
    //      beep(add);
    //}
    if (add > 900)
      add = 300;
    Serial.print("hhh");
    //  }
  }
  if (zuonei < 200 && zuowai < 200 && youwai < 200 && younei < 200)
  {
    //放歌
    Serial.println("放歌:");
    while (zuonei < 200 && zuowai < 200 && youwai < 200 && younei < 200)
    {
      read_data();
    }
    Stop_In = ~Stop_In;
  }
  if (Stop_In == false)
  {
    goto stop_player;
  }
  else
  {
    static int a = 0;
    if (a == 0)
    {

      puts("Play!");

      /* Main volume set to -16.0 dB */
      theAudio->setVolume(-160);
      theAudio->startPlayer(AudioClass::Player0);
    }
    puts("loop!!");

    int err  = theAudio->writeFrames(AudioClass::Player0, myFile);
    if (err == AUDIOLIB_ECODE_FILEEND)
    {
      printf("Main player File End!\n");
    }

    /* Show error code from player and stop */
    if (err)
    {
      printf("Main player error code: %d\n", err);
      goto stop_player;
    }

    if (ErrEnd)
    {
      printf("Error End\n");
      goto stop_player;
    }
    usleep(400);
    a++;
    // goto stop_player;
  }

  /* Unlock the mutex */
  mutex.Unlock();
  return;

stop_player:
  Serial.print("哈哈");
  //sleep(1);
  theAudio->stopPlayer(AudioClass::Player0, AS_STOPPLAYER_NORMAL);
  //myFile.close();

}
