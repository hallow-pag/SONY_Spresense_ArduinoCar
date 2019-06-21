//#include <SDHCI.h>
//#include <Audio.h>
#include <MP.h>
#include <MPMutex.h>
//SDClass theSD;
//AudioClass *theAudio;

//File myFile;

bool ErrEnd = false;
// pin settings
int Left_motor_en       = 7;
int Left_motor_go       = 9; // PWM
int Left_motor_back     = 5; // PWM


int Right_motor_en      = 8;
int Right_motor_go      = 3; // PWM
int Right_motor_back    = 6; // PWM
int key=4;

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

int aa = 0;



//static void audio_attention_cb(const ErrorAttentionParam *atprm)
//{
//  puts("Attention!");
//
//  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
//  {
//    ErrEnd = true;
//  }
//}
//
//void set_up()
//{
//  theAudio = AudioClass::getInstance();
//
//  theAudio->begin(audio_attention_cb);
//
//  puts("initialization Audio Library");
//
//  /* Set clock mode to normal */
//  theAudio->setRenderingClockMode(AS_CLKMODE_NORMAL);
//
//  /* Set output device to speaker with first argument.
//     If you want to change the output device to I2S,
//     specify "AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT" as an argument.
//     Set speaker driver mode to LineOut with second argument.
//     If you want to change the speaker driver mode to other,
//     specify "AS_SP_DRV_MODE_1DRIVER" or "AS_SP_DRV_MODE_2DRIVER" or "AS_SP_DRV_MODE_4DRIVER"
//     as an argument.
//  */
//  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, AS_SP_DRV_MODE_LINEOUT);
//
//  /*
//     Set main player to decode stereo mp3. Stream sample rate is set to "auto detect"
//     Search for MP3 decoder in "/mnt/sd0/BIN" directory
//  */
//  //  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/spif/BIN", AS_SAMPLINGRATE_AUTO, AS_CHANNEL_STEREO);
//  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/spif/BIN", 44100, AS_CHANNEL_STEREO);
//
//  /* Verify player initialize */
//  if (err != AUDIOLIB_ECODE_OK)
//  {
//    printf("Player0 initialize error\n");
//    exit(1);
//  }
//
//  /* Open file placed on SD card */
//  myFile = theSD.open("Jupiter.mp3");
//
//  /* Verify file open */
//  if (!myFile)
//  {
//    printf("File open error\n");
//    exit(1);
//  }
//  printf("Open! %d\n", myFile);
//
//  /* Send first frames to be decoded */
//
//}


// the setup routine runs once when you press reset:
void setup()
{
  Serial.begin(115200);
  pinMode(key,INPUT);//定义按键接口为输入接口
   digitalWrite(key,HIGH);
  MP.begin();
  motor_enable();
  //set_up();
//  err_t err = theAudio->writeFrames(AudioClass::Player0, myFile);
//
//  if ((err != AUDIOLIB_ECODE_OK) && (err != AUDIOLIB_ECODE_FILEEND))
//  {
//    printf("File Read Error! =%d\n", err);
//    myFile.close();
//    exit(1);
//  }
//   puts("Play!");
//
//  /* Main volume set to -16.0 dB */
//  theAudio->setVolume(-160);
}

void motor_enable()
{
  pinMode(Left_motor_en, OUTPUT);
  pinMode(Right_motor_en, OUTPUT);
  digitalWrite(Left_motor_en, HIGH);
  digitalWrite(Right_motor_en, HIGH);
}

void motor_disable()
{
  pinMode(Left_motor_en, OUTPUT);
  pinMode(Right_motor_en, OUTPUT);
  digitalWrite(Left_motor_en, LOW);
  digitalWrite(Right_motor_en, LOW);
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
void run_lowlow()
{
  digitalWrite(Right_motor_go, HIGH); // 右电机前进
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 70);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速
  
  digitalWrite(Left_motor_go, HIGH);  //左轮后退
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 60);
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
}

void run_low()
{
  digitalWrite(Right_motor_go, HIGH); // 右电机前进
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 130);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速

  
  digitalWrite(Left_motor_go, HIGH);  //左轮后退
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 130);
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
}

void run_high()
{
  digitalWrite(Right_motor_go, HIGH); // 右电机前进
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 160);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速

  digitalWrite(Left_motor_go, HIGH);  //左轮后退
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 150);
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
}
void left_big()
{
  digitalWrite(Right_motor_go,HIGH);  // 右电机前进
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,200); 
  analogWrite(Right_motor_back,0);//PWM比例0~255调速
  digitalWrite(Left_motor_go,HIGH);   //左轮后退
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,0);//PWM比例0~255调速
  //delay(time * 100);  //执行时间，可以调整  
}

void left_small()
{
  digitalWrite(Right_motor_go, HIGH); // 右电机前进
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 130);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速
  digitalWrite(Left_motor_go, HIGH);  //左轮后退
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 10 );
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
  //delay(time * 100);  //执行时间，可以调整
}

void left_bz()
{
  digitalWrite(Right_motor_go, HIGH); // 右电机前进
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 150);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速

  digitalWrite(Left_motor_go, HIGH);  //左轮后退
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 30 );
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
  //delay(time * 100);  //执行时间，可以调整
}

void right_big()
{
  digitalWrite(Right_motor_go,HIGH);   //右电机后退
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,10); 
  analogWrite(Right_motor_back,0);//PWM比例0~255调速
  
  digitalWrite(Left_motor_go,HIGH);//左电机前进
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,160); 
  analogWrite(Left_motor_back,0);//PWM比例0~255调速
  //delay(time * 100);  //执行时间，可以调整  
}

void right_small()
{
  digitalWrite(Right_motor_go,HIGH);   //右电机后退
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,80); 
  analogWrite(Right_motor_back,0);//PWM比例0~255调速
  
  digitalWrite(Left_motor_go,HIGH);//左电机前进
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,130); 
  analogWrite(Left_motor_back,0);//PWM比例0~255调速
  //delay(time * 100);  //执行时间，可以调整  
}

void right_bz()
{
  digitalWrite(Right_motor_go,HIGH);   //右电机后退
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,30); 
  analogWrite(Right_motor_back,0);//PWM比例0~255调速
  
  digitalWrite(Left_motor_go,HIGH);//左电机前进
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,150); 
  analogWrite(Left_motor_back,0);//PWM比例0~255调速
  //delay(time * 100);  //执行时间，可以调整  
}


void back( int time)
{
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 100);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 100);
  delay(time * 100);
}

void brake()
{
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 0);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 0);
}


//read_data
void read_data()
{
  rr = analogRead(SensorRight_2);
  rl = analogRead(SensorLeft_2);

  zuowai = analogRead(Sensorm2);
  zuonei = analogRead(Sensorm3);
  younei = analogRead(Sensorm4);
  youwai = analogRead(Sensorm5);
}
MPMutex mutex(MP_MUTEX_ID0);
//print_data
void print_data()
{
  Serial.print("Sensorm2:"); Serial.println(zuowai);
  Serial.print("Sensorm3:"); Serial.println(zuonei);
  Serial.print("Sensorm4:"); Serial.println(younei);
  Serial.print("Sensorm5:"); Serial.println(youwai);
  Serial.print("SensorRight_2:"); Serial.println(rr);
  Serial.print("SensorLeft_2:"); Serial.println(rl);
}

// the loop routine runs over and over again forever:
void loop()
{
keysacn();
while(1){
  
  
  

  static bool Stop_In = false;

  read_data();
  //print_data();


    if(zuowai<200&&zuonei>200&&younei>200)
    {
      //左大转弯
      //Serial.println("左大转弯:");
      left_big();
    }
    else if(youwai<200&&younei>200&&zuonei>200)
    {
      //右大转弯
      //Serial.println("右大转弯:");
      right_big();
    }
     else if((zuonei<200&&younei<200)&&(zuowai>200&&youwai>200))
     {
      //Serial.println("快直走:");
      run_high();
     }
     else if((zuonei < 200 || zuowai < 200) && (youwai > 200 || younei > 200))
    {
      //左小弯
      //Serial.println("左小弯:");
      left_small();
    }
    else if((zuonei > 200 || zuowai > 200) && (youwai < 200 || younei < 200))
    {
      //右小弯
      // Serial.println("右小弯:");
      right_small();
    }
     else if (zuonei < 200 && zuowai < 200 && youwai < 200 && younei < 200)
    {
    //放歌
    //Serial.println("放歌:");
    while (zuonei < 200 && zuowai < 200 && youwai < 200 && younei < 200)
    {
      read_data();
      run_low();
    }
    Stop_In = ~Stop_In;
       run_low();
      }

  else if ((zuonei > 200 || zuowai > 200) && (youwai > 200 || younei > 200))
  {
     if (rl>200&&rr>200)
      {
    //      Serial.println("低俗直走");
           run_low();
      }    
       else if ((rl>200) && (rr<200)||(rl>rr))// 右边探测到有障碍物，有信号返回，向左转 
      {
  //Serial.println("左小弯:--------------");
      right_bz();
      }
    else if (rr>200 && rl<200||(rl<rr)) //左边探测到有障碍物，有信号返回，向右转  
      {
  
     left_bz();
     //Serial.println("右小弯-----------------:");
      
      }
    else
      {
      brake();
      }
  }
  else
  {
    //Serial.println("低俗直走");
    run_high();
  }
mutex.Unlock();

//  if (Stop_In == false)
//  {
//    goto stop_player;
//  }
//  else
//  {
//    static int a = 0;
//    if (a == 0)
//    {
//     
//      puts("Play!");
//
//      /* Main volume set to -16.0 dB */
//      theAudio->setVolume(-160);
//      theAudio->startPlayer(AudioClass::Player0);
//    }
//    puts("loop!!");
//
//    int err  = theAudio->writeFrames(AudioClass::Player0, myFile);
//    if (err == AUDIOLIB_ECODE_FILEEND)
//    {
//      printf("Main player File End!\n");
//    }
//
//    /* Show error code from player and stop */
//    if (err)
//    {
//      printf("Main player error code: %d\n", err);
//      goto stop_player;
//    }
//
//    if (ErrEnd)
//    {
//      printf("Error End\n");
//      goto stop_player;
//    }
//    usleep(400);
//    a++;
//    // goto stop_player;
//  }
//
//  return;
//
//stop_player:
//  //Serial.print("哈哈");
//  //sleep(1);
//  theAudio->stopPlayer(AudioClass::Player0, AS_STOPPLAYER_NORMAL);
//  //myFile.close();
}
}
