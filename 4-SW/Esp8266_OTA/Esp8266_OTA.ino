#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
 
// 闪烁时间间隔(秒)
// const int blinkInterval = 2; 
 
// 设置wifi接入信息(请根据您的WiFi信息进行修改)
const char* ssid = "DN39";
const char* password = "12345678";
 
// Ticker ticker;
 
void setup() {
  Serial.begin(115200);
  Serial.println("ok");
  // pinMode(LED_BUILTIN, OUTPUT);
  // ticker.attach(blinkInterval, tickerCount);  // 设置Ticker对象
  connectWifi();
 
  // OTA设置并启动
  ArduinoOTA.setHostname("ESP8266");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.begin();
  
  Serial.println("OTA ready");
}
void loop() {
  ArduinoOTA.handle();
}
 
// 在Tinker对象控制下，此函数将会定时执行。
// void tickerCount(){
//   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
// }
 
void connectWifi(){
  //开始连接wifi
  WiFi.begin(ssid, password);
 
  //等待WiFi连接,连接成功打印IP
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected!");  
  Serial.print("IP address:\t");            
  Serial.println(WiFi.localIP());          
}