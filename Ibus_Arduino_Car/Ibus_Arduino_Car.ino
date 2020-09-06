#define PT_USE_TIMER
#define Left_motor_go 9 //左电机后退(IN1)
#define Left_motor_back 8 //左电机前进(IN2),no PWM
#define Right_motor_go 6 // 右电机前进(IN4)
#define Right_motor_back 7 // 右电机后退(IN3),no PWM
#define Right_motor_en 5 // 右电机使能
#define Left_motor_en 10 // 右电机使能
#include <EEPROM.h>

#define PT_USE_SEM
#include "pt.h"
#include <IBusBM.h>
IBusBM IBus; // IBus object

//global
#define TT 2000//pwm脉冲周期
int sr = 0, sl = 0;//左右轮速度
int val3 = 0;
int tl = 0, tr = 0;
int addr = 0;//eeprom地址
int slf = 0;//声光标志
int bff = 0;//播放过标志

int yp[] = { 952, 848, 756, 714, 636, 566, 505 }; //乐谱

int sli = 0;//声音频率或for标志
int st = 0;//声音周期
#define ledl  3
#define ledr  4
#define buzzer 12
static struct pt left_go;
static struct pt right_go;
static struct pt iibbuuss;
static struct pt turn_left_t;
static struct pt turn_right_t;
static struct pt my_tone_t;
static struct pt fireman_t;
static struct pt bibi_t;

static struct pt_sem slct;

void setup() {
  pinMode(Left_motor_go, OUTPUT); // PIN 8 (PWM)
  pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_go, OUTPUT);// PIN 6 (PWM)
  pinMode(Right_motor_back, OUTPUT);// PIN 7 (PWM)
  pinMode(Right_motor_en, OUTPUT);// PIN 5
  pinMode(Left_motor_en, OUTPUT);// PIN 10

  pinMode(13, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(ledl, OUTPUT);
  digitalWrite(ledl, LOW);
  pinMode(ledr, OUTPUT);
  digitalWrite(ledr, LOW);
  digitalWrite(Right_motor_en, HIGH);
  digitalWrite(Left_motor_en, HIGH);
  //    Serial.begin(115200);   // remove comment from this line if you change the Serial port in the next line
  IBus.begin(Serial);
  PT_SEM_INIT(&slct, 1);

  PT_INIT(&left_go);
  PT_INIT(&right_go);
  PT_INIT(&iibbuuss);
  PT_INIT(&turn_left_t);
  PT_INIT(&turn_right_t);
  PT_INIT(&my_tone_t);
  PT_INIT(&fireman_t);
  PT_INIT(&bibi_t);
}
static int left_go_thread(struct pt* pt) {
  PT_BEGIN(pt);
  while (1) {
    tl = sl * 20; //计算脉冲时间
    if (tl >= 0) {
      //前进
      if (tl < 2000) {
        //如果速度不是最低或最高，则进行类似pwm脉冲调速(部分接口不支持模拟输出，所以是类似pwm脉冲)
        digitalWrite(Left_motor_go, HIGH);
        PT_TIMER_MICRODELAY(pt, tl);
        digitalWrite(Left_motor_go, LOW);
        PT_TIMER_MICRODELAY(pt, TT - tl);
      }
      else if (tl >= 2000) {
        //如果速度最大，则把接口一直置为高电平
        digitalWrite(Left_motor_go, HIGH);
        PT_YIELD(pt);//多线程需要在不需要计算资源时让出计算资源
      }
      else {
        //如果速度最低则把接口一直置为低电平
        digitalWrite(Left_motor_go, LOW);
        PT_YIELD(pt);//多线程需要在不需要计算资源时让出计算资源
      }
    }
    else {
      //后退
      tl = -tl;//由于根据负的速度计算出负的脉冲时间，这里变为正值
      if (tl < 2000) {
        //调速
        digitalWrite(Left_motor_back, HIGH);
        PT_TIMER_MICRODELAY(pt, tl);
        digitalWrite(Left_motor_back, LOW);
        PT_TIMER_MICRODELAY(pt, TT - tl);
      }
      else {
        //最高速度
        digitalWrite(Left_motor_back, HIGH);
        PT_YIELD(pt);
      }
    }
  }
  PT_END(pt);
}

static int right_go_thread(struct pt* pt) {
  PT_BEGIN(pt);

  // Loop forever
  while (1) {
    tr = sr * 20;
    if (tr >= 0) {
      if (tr < 2000) {
        digitalWrite(Right_motor_go, HIGH);
        PT_TIMER_MICRODELAY(pt, tr);
        digitalWrite(Right_motor_go, LOW);
        PT_TIMER_MICRODELAY(pt, TT - tr);
      }
      else if (tr >= 2000) {
        digitalWrite(Right_motor_go, HIGH);
        PT_YIELD(pt);
      }
      else {
        digitalWrite(Right_motor_go, LOW);
        PT_YIELD(pt);
      }
    }
    else {
      tr = -tr;
      if (tr < 2000) {
        digitalWrite(Right_motor_back, HIGH);
        PT_TIMER_MICRODELAY(pt, tl);
        digitalWrite(Right_motor_back, LOW);
        PT_TIMER_MICRODELAY(pt, TT - tl);
      }
      else {
        digitalWrite(Right_motor_back, HIGH);
        PT_YIELD(pt);
      }
    }
  }
  PT_END(pt);
}
static int turn_left_light(struct pt* pt) {
  PT_BEGIN(pt);
  while (1)
  {
    PT_WAIT_UNTIL(pt, slf == 1);//在不需要计算资源时，让出计算资源
    PT_SEM_WAIT(pt, &slct);//等待获取声光控制权
    digitalWrite(ledl, HIGH);//转向灯灯亮
    for (sli = 0; sli < 25; sli++) {//发出滴声音
      digitalWrite(buzzer, HIGH);
      PT_TIMER_DELAY(pt, 1);
      digitalWrite(buzzer, LOW);
      PT_TIMER_DELAY(pt, 1);
    }
    PT_TIMER_DELAY(pt, 450);//延时一段时间
    digitalWrite(ledl, LOW);//转向灯灭
    for (sli = 0; sli < 30; sli++) {//发出嗒声
      digitalWrite(buzzer, HIGH);
      PT_TIMER_MICRODELAY(pt, 300);
      digitalWrite(buzzer, LOW);
      PT_TIMER_MICRODELAY(pt, 300);
    }
    PT_TIMER_DELAY(pt, 482);//延时一段时间
    PT_SEM_SIGNAL(pt, &slct);//释放声光控制权
  }
  PT_END(pt);

}
static int turn_right_light(struct pt* pt) {
  PT_BEGIN(pt);
  while (1)
  {
    PT_WAIT_UNTIL(pt, slf == 2);
    PT_SEM_WAIT(pt, &slct);
    digitalWrite(ledr, HIGH);
    for (sli = 0; sli < 25; sli++) {
      digitalWrite(buzzer, HIGH);
      PT_TIMER_DELAY(pt, 1);
      digitalWrite(buzzer, LOW);
      PT_TIMER_DELAY(pt, 1);
    }
    PT_TIMER_DELAY(pt, 450);
    digitalWrite(ledr, LOW);
    for (sli = 0; sli < 30; sli++) {
      digitalWrite(buzzer, HIGH);
      PT_TIMER_MICRODELAY(pt, 300);
      digitalWrite(buzzer, LOW);
      PT_TIMER_MICRODELAY(pt, 300);
    }
    PT_TIMER_DELAY(pt, 482);
    PT_SEM_SIGNAL(pt, &slct);
  }
  PT_END(pt);

}
static int my_tone(struct pt* pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, slf >= 3 && st != 0);//在不需要计算资源时，让出计算资源（slf为声光控制标志、st为根据发声频率转换的周期）
    digitalWrite(buzzer, HIGH);
    PT_TIMER_MICRODELAY(pt, st);
    digitalWrite(buzzer, LOW);
    PT_TIMER_MICRODELAY(pt, st);
  }
  PT_END(pt);
}
static int fireman(struct pt* pt) {
  PT_BEGIN(pt);
  while (1)
  {
    PT_WAIT_UNTIL(pt, slf == 3);
    PT_SEM_WAIT(pt, &slct);
    for (sli = 500; sli <= 1000; sli++)//辒出一个频率的声音
    {
      if (slf != 3)break;
      st = (1. / sli) * 500000;
      PT_TIMER_DELAY(pt, 4); //延时 4ms
    }
    for (sli = 1000; sli >= 500; sli--)//辒出另一个频率癿声音
    {
      if (slf != 3)break;
      st = (1. / sli) * 500000;
      PT_TIMER_DELAY(pt, 3);
    }
    st = 0;
    PT_SEM_SIGNAL(pt, &slct);
  }
  PT_END(pt);
}
static int bibi(struct pt* pt) {
  PT_BEGIN(pt);
  while (1)
  {
    PT_WAIT_UNTIL(pt, slf == 4);
    PT_SEM_WAIT(pt, &slct);
    st = 1500;
    PT_TIMER_DELAY(pt, 150);
    st = 0;
    PT_SEM_SIGNAL(pt, &slct);
  }
  PT_END(pt);
}



static int update_channel(struct pt* pt) {
  PT_BEGIN(pt);
  while (1) {
    while (IBus.readChannel(5) == 1000) {
      PT_TIMER_DELAY(pt, 150);
      if (IBus.readChannel(9) == 2000) {
        //录制开始
        if (addr <= 1022) {
          //EEPROM大小为1KB
          //这里使用8bit，即1Byte存储一个值(大小限制为0-255满足要求，且录制间隔为150ms这样可以尽可能放大录制时长)
          //目前录制时长为1024/3=341，341*0.15ms=51s(满足本次实验要求)
          digitalWrite(3, HIGH);//录制指示灯
          EEPROM.write(addr, sr + 100);//由于Byte型为无符号0-255，所以需要把-100到100的速度值映射到0到200存储
          EEPROM.write(++addr, sl + 100);
          EEPROM.write(++addr, slf);
          addr++;
        }
        else {
          digitalWrite(3, LOW);//停止录制时关闭录制指示灯
        }
      }
      else {
        addr = 0;//停止录制时将地址标志重新置0以便重新开始录制
        digitalWrite(3, LOW);//停止录制时关闭录制指示灯
      }
      //    Serial.println(IBus.readChannel(4));
      //channel4=>swa
      //Channel2=>油门
      //Channel3=>偏航
      //    Serial.println(sr);
      if (IBus.readChannel(4) == 1000) {
        //如果处于前进状态
        sr = sl = (IBus.readChannel(2) - 1000) * .1 + .3;//将接收机的遥控值转换为速度值
        val3 = ((int)IBus.readChannel(3) - 1500) / 5;//将接收机的遥控值转换为速度偏移量
        //左右轮速度配合偏移量实现转弯
        sr -= val3;
        sl += val3;
        PT_YIELD(pt);//在不需要计算资源时让出计算资源
      }
      else if (IBus.readChannel(4) == 2000) {
        sr = sl = -((IBus.readChannel(2) - 1000) * .1 + .1);
        val3 = ((int)IBus.readChannel(3) - 1500) / 5;
        sr += val3;
        sl -= val3;
        PT_YIELD(pt);
      }
      if (IBus.readChannel(0) < 1450) slf = 1;//左转向
      else if (IBus.readChannel(0) > 1550) slf = 2;//右转向
      else slf = 0;//关闭声光
      if (IBus.readChannel(8) > 1550) slf = 3;//消防车声
      else if (IBus.readChannel(8) < 1450) slf = 4;//喇叭声
    }
    while (IBus.readChannel(5) == 2000) {
      if (IBus.readChannel(9) == 2000) {

        if (!bff) {
          //如果处于播放状态且未播放过，防止重复播放
          for (sli = 0; sli < 3; sli++) {
          //启动3声闪3次光，以满足实验要求
            digitalWrite(ledr, HIGH);
            digitalWrite(ledl, HIGH);
            //改变声光控制标志以及发声频率的周期即可
            slf = 5;
            st = 1500;
            PT_TIMER_DELAY(pt, ((int)(sli / 2 + 1) * 500));
            digitalWrite(ledr, LOW);
            digitalWrite(ledl, LOW);
            slf = 0;
            st = 0;
            if (sli != 2) PT_TIMER_DELAY(pt, ((int)(sli / 2 + 1) * 500));
          }
          
          for (addr = 0; addr <= 1022; addr++) {
            //每150ms读出EEPROM中的值并赋值给各控制标志
            if (IBus.readChannel(9) == 2000) {
              sr = EEPROM.read(addr) - 100;
              sl = EEPROM.read(++addr) - 100;
              slf = EEPROM.read(++addr);
            }
            else {
              //播放结束时改变播放标志
              bff = 2;
              break;
            }
            PT_TIMER_DELAY(pt, 150);
          }
          if(addr>=1022) bff = 1;//播放标志位等于1为完成完整播放
        }

        if(bff==1){
          //如果完成播放结束则启动结束3s闪灯和声音，以满足实验要求
          digitalWrite(ledr, HIGH);
          digitalWrite(ledl, HIGH);
          slf = 5;
          for (sli = 0; sli < 7; sli++) {
            st = yp[sli];
            PT_TIMER_DELAY(pt, 214);
            st = 0;
            PT_TIMER_DELAY(pt, 214);
          }
          slf = 0;
          digitalWrite(ledr, LOW);
          digitalWrite(ledl, LOW);
          bff=3;//播放标志为3代表完成所有操作
        }
        addr = 0;
        sr = sl = 0;
        PT_YIELD(pt);
      }
      else {
        //手动结束结束
        sr = sl = 0;//速度置0
        bff = 0;//重置播放标志
        PT_TIMER_DELAY(pt, 150);
      }
    }
  }
  PT_END(pt);
}
void loop() {
  update_channel(&iibbuuss);
  left_go_thread(&left_go);
  right_go_thread(&right_go);
  turn_right_light(&turn_right_t);
  turn_left_light(&turn_left_t);
  my_tone(&my_tone_t);
  fireman(&fireman_t);
  bibi(&bibi_t);
}
