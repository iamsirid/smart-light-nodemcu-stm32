#include <ESP8266WiFi.h>
#include<ESP8266WiFiMulti.h>
#include<ESP8266HTTPClient.h>
#include <MicroGear.h>
#include<SoftwareSerial.h>
#include<HttpClient.h>
SoftwareSerial uart(13, 15, false, 256);

#define APPID   "***REMOVED***"
#define KEY     "***REMOVED***"
#define SECRET  "***REMOVED***"

#define ALIAS   "pieled"

char* ssid = "SmartLight";
char* password = "***REMOVED***";

const int GREEN_LED = D0;
const int RED_LED = D1;
const int LED1 = D2;
const int LED2 = D5;
const int SWITCH = D4;
const int BUZZER = D6;
uint8_t msg[8];

int Sound;
int connecting;

ESP8266WiFiMulti WiFiMulti;

WiFiClient client;
MicroGear microgear(client);

int state_LED1 = 0;
int state_LED2 = 0;
int update_time = 1000;
int stateOutdated = 0;
char buff[16];
char state[10];
char token[10];
char moder[10];
char lightFeed[10];

void updateState(){
  for(int x=0;x<10;x++){
  char temp = uart.read();
  token[x] = temp;
  }
  if(token[0] == '0'){
    strncpy(state,token,10);
    sendState();
  }
  else if(token[0] == '1'){
    strncpy(moder,token,10);
    sendMode();
  }
  else if(token[0] == '2'){
    token[0] = '0';
    strncpy(lightFeed, token, 10);
    sendLightFeed();
  }
}

void sendLightFeed(){
  char temp[20];
  sprintf(temp,"ligth:%d",atoi(lightFeed));
  Serial.println(temp);
  microgear.writeFeed("LightSensor",temp);
}

//Update state to NETPIE
void sendState(){
  Serial.println(state);
  /*microgear.publish("/pieled/state/LED1",String(state[1]));
  microgear.publish("/pieled/state/LED2",String(state[2]));
  microgear.publish("/pieled/state/LED3",String(state[3]));
  microgear.publish("/pieled/state/LED4",String(state[4]));
  microgear.publish("/pieled/state/LED5",String(state[5]));
  microgear.publish("/pieled/state/LED6",String(state[6]));*/
  
  HTTPClient http;   
   http.begin("http://smart-light-embedded-lab.herokuapp.com/api/update-data");  //Specify destination for HTTP request
   http.addHeader("Content-Type", "application/x-www-form-urlencoded");            //Specify content-type header
   char buf[256];
    
snprintf(buf, sizeof buf, "%s%c%s%s%c%s%s%c%s%s%c%s%s%c%s%s%c", "LED1=", state[1], "&", "LED2=",state[2],"&","LED3=",state[3],"&","LED4=",state[4],"&","LED5=",state[5],"&","LED6=",state[6]);
  int httpResponseCode = http.POST(buf);   //Send the actual POST request

if(httpResponseCode>0){
    String response = http.getString();                       
    Serial.println(httpResponseCode);   //Print return code
    Serial.println(response);           //Print request answer
 
   }else{
 
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
 
   }
 
   http.end();  //Free resources
   
  Serial.println("send state..");
//  stateOutdated = 0;
}

void sendMode(){
  Serial.print(moder);
  /*microgear.publish("/pieled/state/MOD1",String(moder[1]));
  microgear.publish("/pieled/state/MOD2",String(moder[2]));
  microgear.publish("/pieled/state/MOD3",String(moder[3]));
  microgear.publish("/pieled/state/MOD4",String(moder[4]));
  microgear.publish("/pieled/state/MOD5",String(moder[5]));
  microgear.publish("/pieled/state/MOD6",String(moder[6]));*/
  HTTPClient http;   
   http.begin("http://smart-light-embedded-lab.herokuapp.com/api/update-data");  //Specify destination for HTTP request
   http.addHeader("Content-Type", "application/x-www-form-urlencoded");            //Specify content-type header
   char buf[256];
    
snprintf(buf, sizeof buf, "%s%c%s%s%c%s%s%c%s%s%c%s%s%c%s%s%c", "MOD1=", moder[1], "&", "MOD2=",moder[2],"&","MOD3=",moder[3],"&","MOD4=",moder[4],"&","MOD5=",moder[5],"&","MOD6=",moder[6]);
  int httpResponseCode = http.POST(buf);   //Send the actual POST request

if(httpResponseCode>0){
    String response = http.getString();                       
    Serial.println(httpResponseCode);   //Print return code
    Serial.println(response);           //Print request answer
 
   }else{
 
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
 
   }
 
   http.end();  //Free resources
  Serial.println("send mode..");
}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  if(digitalRead(SWITCH) == 1){
    //uart transmit to STM32F407
    //char token1 = token[8];
    Serial.print("topic   ");
    Serial.println(topic);
    msg[8] = '\0';
    uart.write((char *)msg);
    
    Serial.print("Incoming message -->");
    Serial.println((char *)msg);
  }
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.println("\nConnected to NETPIE...");
  microgear.setAlias(ALIAS);
  //stateOutdated = 1;
}

void setup(){
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(SWITCH, INPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  digitalWrite(RED_LED, HIGH);
  
  Serial.begin(115200);
  uart.begin(115200);
  Serial.println("Starting...");

  if (WiFi.begin(ssid, password)) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);    
  microgear.init(KEY,SECRET,ALIAS);
  microgear.connect(APPID);
}

void loop(){

  if (microgear.connected() && digitalRead(SWITCH) == 1) {
    connecting = 0;
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    //update_time -= 1;
    //if (update_time == 0){
    //  stateOutdated = 1;
    //  update_time = 1000;
    //}
    //if (stateOutdated) sendState();
    microgear.loop();

    if(uart.available()>0){
      updateState();
       
    }
  }
  else if(digitalRead(SWITCH) == 0){
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    microgear.loop();
    stateOutdated = 1;
    delay(500);
  }
  else {
    
    if(connecting != 1){
      connecting = 1;
    Serial.println("connection lost, reconnect...");
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    microgear.connect(APPID);
     }else {
      Serial.println("this not right!");
      
      }
    
  }
  delay(10);
}
