#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MQ135.h>

//arduino的GND接串口屏或串口工具的GND,共地
//arduino的9接串口屏或串口工具的RX
//arduino的8接串口屏或串口工具的TX
//arduino的5V接串口屏的5V,如果是串口工具,不用接5V也可以
//根据自己的实际修改对应的TX_Pin和RX_Pin
#define TJC Serial1
#define DHTTYPE    DHT11     // DHT 22 (AM2302)
#define TX_Pin 17
#define RX_Pin 18
#define FRAME_LENGTH 7
#define LED_Pin 21
#define DHTPIN 2     // Digital pin connected to the DHT sensor 
#define PIN_MQ135  A2
#define Soil_val   A0


// Sensor library:
DHT_Unified dht(DHTPIN, DHTTYPE);
MQ135 mq135_sensor(PIN_MQ135);
uint32_t delayMS;
float temperature = 21.0, humidity = 25.0, Soil_wet = 0;
unsigned long nowtime;
const uint16_t AirValue = 3015;   // Air value of the soil_sensor
const uint16_t WaterValue = 1180;  // Water value of the soil_sensor
uint16_t Soil_int = (AirValue - WaterValue) / 3;

// Function declaration:
void Soil_judge(void);

void setup() {
  // put your setup code here, to run once:
  dht.begin();
  TJC.begin(115200, SERIAL_8N1, RX_Pin, TX_Pin);  //串口1初始化
  Serial.begin(115200); //串口0初始化
  pinMode(LED_Pin, OUTPUT);

  analogReadResolution(12);  // 设置ADC分辨率位12位

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  
  //因为串口屏开机会发送88 ff ff ff,所以要清空串口缓冲区
  while (TJC.read() >= 0); //清空串口缓冲区
  TJC.print("page start\xff\xff\xff"); //发送命令让屏幕跳转到main页面
  nowtime = millis(); //获取当前已经运行的时间
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // put your main code here, to run repeatedly:
  char str[100];
  if (millis() >= nowtime + 1000) { //1s为周期
    nowtime = millis(); //获取当前已经运行的时间
    sensors_event_t event;
    dht.temperature().getEvent(&event);

    //用sprintf来格式化字符串，给n0的val属性赋值
    sprintf(str, "n0.val=%d\xff\xff\xff", (uint8_t)event.temperature);
    TJC.print(str);
    dht.humidity().getEvent(&event);
    sprintf(str, "n1.val=%d\xff\xff\xff", (uint8_t)event.relative_humidity);
    TJC.print(str);

    float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
    sprintf(str, "n4.val=%d\xff\xff\xff", (uint16_t)correctedPPM);
    TJC.print(str);

    Soil_judge();
    delay(50);  //延时50ms,才能看清楚点击效果

    //用sprintf来格式化字符串，触发b0的弹起事件,直接把结束符整合在字符串中
    //sprintf(str, "click b0,0\xff\xff\xff");
    //把字符串发送出去
    //TJC.print(str);
  }

  //串口数据格式：
  //串口数据帧长度：7字节
  //帧头     参数1    参数2   参数3       帧尾
  //0x55     1字节   1字节    1字节     0xffffff
  //当参数是01时
  //帧头     参数1    参数2   参数3       帧尾
  //0x55     01     led编号  led状态    0xffffff
  //例子1：上位机代码  printh 55 01 01 00 ff ff ff  含义：1号led关闭
  //例子2：上位机代码  printh 55 01 04 01 ff ff ff  含义：4号led打开
  //例子3：上位机代码  printh 55 01 00 01 ff ff ff  含义：0号led打开
  //例子4：上位机代码  printh 55 01 04 00 ff ff ff  含义：4号led关闭

  //当参数是02或03时
  //帧头     参数1    参数2   参数3       帧尾
  //0x55     02/03   滑动值    00    0xffffff
  //例子1：上位机代码  printh 55 02 64 00 ff ff ff  含义：h0.val=100
  //例子2：上位机代码  printh 55 02 00 00 ff ff ff  含义：h0.val=0
  //例子3：上位机代码  printh 55 03 64 00 ff ff ff  含义：h1.val=100
  //例子4：上位机代码  printh 55 03 00 00 ff ff ff  含义：h1.val=0

  //当串口缓冲区大于等于一帧的长度时
  while (TJC.available() >= FRAME_LENGTH) {
    unsigned char ubuffer[FRAME_LENGTH];
    //从串口缓冲读取1个字节但不删除
    unsigned char frame_header = TJC.peek();
    //当获取的数据是包头(0x55)时
    if (frame_header == 0x55) {
      //从串口缓冲区读取7字节
      TJC.readBytes(ubuffer, FRAME_LENGTH);
      for(int i = 0; i < FRAME_LENGTH; i++)
      {
        Serial.print(ubuffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println("");
      if (ubuffer[4] == 0xff && ubuffer[5] == 0xff && ubuffer[6] == 0xff) {
        if(ubuffer[1] == 0x01)
        {
          //下发的是LED信息
          sprintf(str, "msg.txt=\"led %d is %s\"\xff\xff\xff", ubuffer[2], ubuffer[3] ? "on" : "off");
          TJC.print(str);
          if(ubuffer[3] == 0x01){
            digitalWrite(LED_Pin, HIGH);
          }else if(ubuffer[3] == 0x00){
            digitalWrite(LED_Pin, LOW);
          }
        }else if(ubuffer[1] == 0x02)
        {
          //下发的是滑动条h0.val的信息
          sprintf(str, "msg.txt=\"h0.val is %d\"\xff\xff\xff", ubuffer[2]);
          TJC.print(str);
        }else if(ubuffer[1] == 0x03)
        {
          //下发的是滑动条h1.val的信息
          sprintf(str, "msg.txt=\"h1.val is %d\"\xff\xff\xff", ubuffer[2]);
          TJC.print(str);
        }
      }
    } else {
      TJC.read();  //从串口缓冲读取1个字节并删除
    }
  }
}

void Soil_judge(void){
  Soil_wet = analogRead(A0);  //put Sensor insert into soil
  if(Soil_wet > WaterValue && Soil_wet < (WaterValue + Soil_int))
  {
    Serial.println("Very Wet");
  }
  else if(Soil_wet > (WaterValue + Soil_int) && Soil_wet < (AirValue - Soil_int))
  {
    Serial.println("Wet");
  }
  else if(Soil_wet < AirValue && Soil_wet > (AirValue - Soil_int))
  {
    Serial.println("Dry");
  }
}