#include "DHT.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <virtuabotixRTC.h>
DynamicJsonDocument doc(1024);
#include <LiquidCrystal_I2C.h>

int stirrer_time = 20000; //20000 milli sec = 20 sec

LiquidCrystal_I2C lcd(0x27, 16,2);
virtuabotixRTC myRTC(6, 7, 8);

/*Schedule stepper stirrers*/
bool stepper_direction[4] = {false,false,false,false};
int stepper_state[4] = {-1,-1,-1,-1};
bool SCHEDULE[4] = {false,false,false,false};
unsigned long stirrer_start_time[4] = {-1,-1,-1,-1};


/*Steppers*/
bool CS_P[] = {false,false,false,false,false,false,false};
int dir_P[] = {A0,A1,A2,A3,A4,A5,A6}; //direction pins
int en_P[] = {A7,A8,A9,A10,A11,A12,A13}; //enable pins

/*Stirrers*/
bool Ss[] = {false,false,false,false};
int Spwm[] = {0,0,0,0};
int Spins[] = {2,3,4,5};

/*Timer*/
unsigned long current_time = 0;
unsigned long previous_time = 0;
unsigned long beep_time = 0;

/*DHT*/
#define DHTPIN 22
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

float temperature = 0;
float humidity = 0;
float SET_TEMP = 35; // value for buzzer to beep when current temperature increase from this (SET_TEMP+4)

/*LEDS*/
#define LEDlatch  23  //595 pin 12 with arduino pin 23
#define LEDclock  24  //595 pin  11 with arduino pin 24
#define LEDdata   25  //595 pin 14 with arduino pin 25
bool LED_STATUS = false;
bool btns[] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
int del = 80;
bool isdir = false;
bool CW = false;
bool CCW = false;
int cw = 0;
int ccw = 0;

byte leds = 0;
bool change = false;

/*Buttons*/
#define BUTTON_CHIPS   3
#define DATA_WIDTH   BUTTON_CHIPS * 8

bool Data[DATA_WIDTH];
bool PRE_Data[DATA_WIDTH];

#define BTNLoad      26 //All 165 pin 1 with arduino pin 26
#define BTNEnable    27 //All 165 pin 15 with arduino pin 27
#define BTNData      28 //Last 165 pin 9 with arduino pin 28
#define BTNClock     29 //All 165 pin 2 with arduino pin 29

/*Buzzer*/
#define BUZZER 30
bool beep = false;
int state = LOW;


void READ_BTN(){
  digitalWrite(BTNEnable, HIGH);
  digitalWrite(BTNLoad, LOW);
  delayMicroseconds(5);
  digitalWrite(BTNLoad, HIGH);
  digitalWrite(BTNEnable, LOW);
  
  for(int i = 0; i < DATA_WIDTH; i++){
    if(digitalRead(BTNData) == LOW)
      Data[i] = false;
    else
      Data[i] = true;
    
    digitalWrite(BTNClock, HIGH);
    delayMicroseconds(5);
    digitalWrite(BTNClock, LOW);
  }
}

void read_dht(){
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    //Serial.println(F("Failed to read from DHT sensor!"));
  }
}

void PWM_S(){
  for(int i=0; i<4; i++){
    if(Ss[i]  && Spwm[i]<100){
      if(Spwm[i] == 0){
        Spwm[i] = 60;
        stirrer_start_time[i] = millis();
      }
      else
        Spwm[i] += 10;
      analogWrite(Spins[i], Spwm[i]);
    }
    else if(Ss[i] == false && Spwm[i]!=0){
      Spwm[i] = 0;
      analogWrite(Spins[i], Spwm[i]);
    }
  }
}

bool update_dir(int x, bool d){
  bool valid = false;
  switch(x){
    case 1:
      if(CS_P[0] == false){
         digitalWrite(en_P[0], LOW);
        if(d)
          digitalWrite(dir_P[0], HIGH);
        else
          digitalWrite(dir_P[0], LOW);
        valid = true;
      }
      break;
    case 4:
      if(CS_P[1] == false){
        digitalWrite(en_P[1], LOW);
        if(d)
          digitalWrite(dir_P[1], HIGH);
        else
          digitalWrite(dir_P[1], LOW);
        valid = true;
      }
      break;
    case 7:
      if(CS_P[2] == false){
        digitalWrite(en_P[2], LOW);
        if(d)
          digitalWrite(dir_P[2], HIGH);
        else
          digitalWrite(dir_P[2], LOW);
        valid = true;
      }
      break;
    case 10:
      if(CS_P[3] == false){
        digitalWrite(en_P[3], LOW);
        if(d)
          digitalWrite(dir_P[3], HIGH);
        else
          digitalWrite(dir_P[3], LOW);
        valid = true;
      }
      break;
    case 13:
      if(CS_P[4] == false){
        digitalWrite(en_P[4], LOW);
        if(d)
          digitalWrite(dir_P[4], HIGH);
        else
          digitalWrite(dir_P[4], LOW);
          valid = true;
      }
      break;
    case 15:
      if(CS_P[5] == false){
        digitalWrite(en_P[5], LOW);
        if(d)
          digitalWrite(dir_P[5], HIGH);
        else
          digitalWrite(dir_P[5], LOW);
        valid = true;
      }
      break;
    case 17:
      if(CS_P[6] == false){
        digitalWrite(en_P[6], LOW);
        if(d)
          digitalWrite(dir_P[6], HIGH);
        else
          digitalWrite(dir_P[6], LOW);
        valid = true;
      }
      break;
  }
  return valid;
}

void setup(){
  Serial1.begin(115200);
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  dht.begin();
  lcd.clear();
  lcd.setCursor(0,0);

  pinMode(LEDlatch, OUTPUT);
  pinMode(LEDdata, OUTPUT);  
  pinMode(LEDclock, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  pinMode(BTNLoad, OUTPUT);
  pinMode(BTNEnable, OUTPUT);
  pinMode(BTNClock, OUTPUT);
  pinMode(BTNData, INPUT);

  for(int i=0; i<4; i++)
    pinMode(Spins[i], OUTPUT);
  for(int i=0; i<7; i++){
    pinMode(dir_P[i], OUTPUT);
    pinMode(en_P[i], OUTPUT);
  }
  PWM_S();
  READ_BTN();
  for(byte i=0; i<=DATA_WIDTH-1; i++)
    PRE_Data[i] = Data[i];

  leds = 0;
  update_LEDS();
}

void pump_state(int x, bool S, bool d){
  switch(x){
    case 1:
      if(CS_P[0] != S){
        CS_P[0] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '0', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[0], HIGH);
      }
      break;
    case 4:
      if(CS_P[1] != S){
        CS_P[1] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '1', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[1], HIGH);
      }
      break;
    case 7:
      if(CS_P[2] != S){
        CS_P[2] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '2', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[2], HIGH);
      }
      break;
    case 10:
      if(CS_P[3] != S){
        CS_P[3] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '3', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[3], HIGH);
      }
      break;
    case 13:
      if(CS_P[4] != S){
        CS_P[4] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '4', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[4], HIGH);
      }
      break;
    case 15:
      if(CS_P[5] != S){
        CS_P[5] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '5', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[5], HIGH);
      }
      break;
    case 17:
      if(CS_P[6] != S){
        CS_P[6] = S;
        Serial1.print("{'PS': 'P', 'A': '" + String(S) + "', 'S': '6', 'Pd':'" + String(d) + "'}");
        if(!S)
          digitalWrite(en_P[6], HIGH);
      }
      break;
  }
}

void READ_ESP(){
  if(Serial1.available()){
    String payload = Serial1.readString();
    Serial.println(payload);
    deserializeJson(doc, payload);
    JsonObject obj = doc.as<JsonObject>();
    String act = obj[String("A")].as<String>();
    if(act == "S"){
      int P = obj[String("Sn")].as<int>()-1;
      bool State = obj[String("Ss")].as<int>();
      if(P<4 && P>-1){
        if(State)
          Ss[P] = true;
        else
          Ss[P] = false;
      }
    }
    else if(act == "P"){
      int P = obj[String("Pn")].as<int>()-1;
      bool State = obj[String("Ps")].as<int>();
      bool Dir = obj[String("Pd")].as<int>();
      bool shed = obj[String("sh")].as<int>();
      CS_P[P] = State;
      if(P < 4 && State && shed){
        SCHEDULE[P] = true;
        stepper_state[P] = State;
        stepper_direction[P] = Dir;
        return;
      }
      else{
        if(State){
          digitalWrite(en_P[P], LOW);
          if(Dir){
            CW = true;
            cw +=1;
            digitalWrite(dir_P[P], HIGH);
          }
          else{
            Serial.println("Changing DIR");
            CCW = true;
            ccw +=1;
            digitalWrite(dir_P[P], LOW);
          }
        }
        else{
          digitalWrite(en_P[P], HIGH);
          if(digitalRead(dir_P[P])){
            cw -=1;
            if(cw<1)
              CW = false;
          }
          else{
            ccw -=1;
            if(ccw<1)
              CCW = false;
          }
        }
      }
    }
    else if(act == "DT"){
      int h = obj[String("h")].as<int>();
      int m = obj[String("m")].as<int>();
      int s = obj[String("s")].as<int>();
      int dow = obj[String("dow")].as<int>();
      int dom = obj[String("dom")].as<int>();
      int mth = obj[String("M")].as<int>();
      int yr = obj[String("Y")].as<int>();
      myRTC.setDS1302Time(s, m, h, dow, dom, mth, yr);
    }
    else if(act == "UDT"){
      Serial.println("ESP Asking for Time");
      String h = String(myRTC.hours);
      String m = String(myRTC.minutes);
      String s = String(myRTC.seconds);
      String dom = String(myRTC.dayofmonth);
      String mth = String(myRTC.month);
      String yr = String(myRTC.year);
      Serial1.print("{'PS':'UT','h':'" +h+ "','m':'" +m+ "','s':'" +s+ "','dom':'" +dom+ "','M':'" +mth+ "','Y':'" +yr+ "'}");
      delay(100);
    }
  }
}

void BTNPress(){
  for(byte i=0; i<=DATA_WIDTH-1; i++){
    if(i<18){
      if(Data[i] != PRE_Data[i]){
        int b = i+1;
        if(Data[i] == true){
          isdir = false;
          if(b %3 == 0 && (b /3)<5){
            if(!SCHEDULE[(b/3)-1])
              Ss[(b/3)-1] = true;
          }
          else if(b==1 || b==4 || b==7 || b==10 || b==13 || b==15 || b==17){
            isdir = update_dir(b, true);
            if(isdir){
              btns[b-1] = true;
              CW = true;
              cw +=1;
              pump_state(b, true, true);
            }
          }
          else if(b==2 || b==5 || b==8 || b==11 || b==14 || b==16 || b==18){
            isdir = update_dir(b-1, false);
            if(isdir){
              btns[b-1] = true;
              CCW = true;
              ccw +=1;
              pump_state(b-1, true, false);
            }
          }
          delay(500);
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Pressed");
          lcd.setCursor(0,1);
          lcd.print(String(b));
          delay(500);
        }
        else{
          if(b %3 == 0 && (b /3)<5){
            if(!SCHEDULE[(b/3)-1])
              Ss[(b/3)-1] = false;
          }
          else if(b==1 || b==4 || b==7 || b==10 || b==13 || b==15 || b==17){
            if(btns[b-1] == true){
              btns[b-1] = false;
              if(cw < 2)
                CW = false;
              cw -= 1;
              pump_state(b, false, false);
            }
          }
          else if(b==2 || b==5 || b==8 || b==11 || b==14 || b==16 || b==18){
            if(btns[b-1] == true){
              btns[b-1] = false;
              if(ccw < 2)
                CCW = false;
              ccw -= 1;
              pump_state(b-1, false, false);
            }
          }
          delay(500);
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Released");
          lcd.setCursor(0,1);
          lcd.print(String(b));
          delay(500);
        }
        PRE_Data[i] = Data[i];
      }
    }
  }
}


void loop() {
  READ_BTN();
  READ_ESP();
  BTNPress();
  current_time = millis();
  if(current_time - previous_time > 300){
    previous_time = current_time;
    PWM_S();
    
    if(temperature>SET_TEMP+4)
      beep = true;
    else{
      state = LOW;
      digitalWrite(BUZZER, state);
      beep = false;
      }
  }
  if(current_time - beep_time > 1000){
    read_dht();
    myRTC.updateTime();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(String(temperature,1));
    lcd.print(" H:");
    lcd.print(String(humidity, 1));
    lcd.setCursor(14,0);
    lcd.print(String(myRTC.seconds));
    lcd.setCursor(0,1);
    lcd.print(String(myRTC.dayofmonth) + "/" + String(myRTC.month) + "/" + String(myRTC.year));
    lcd.print(" ");
    lcd.print(String(myRTC.hours) + ":" + String(myRTC.minutes));
    

    if(CS_P[0] || CS_P[1] || CS_P[2] || CS_P[3] || CS_P[4] || CS_P[5] || CS_P[6])
      update_LEDS();
    
    if(beep){
      state = !state;
      digitalWrite(BUZZER, state);
    }
    beep_time = current_time;
  }
  for(int i=0; i<4; i++){
    if(SCHEDULE[i] && !Ss[i]){
      Serial.println("Starting stirrer: " + String(i+1) + " manuall switch for this stirrer will not work for " + String(stirrer_time) +" sec");
      Ss[i] = true;
    }
    else if(SCHEDULE[i] && Ss[i] && current_time - stirrer_start_time[i] >= stirrer_time){
      SCHEDULE[i] = false;
      Ss[i] = false;
      Serial.println("Stopping stirrer after " + String(stirrer_time) +" sec");
      Serial.println("Starting pump: " + String(i+1));
      if(stepper_state[i]){
          digitalWrite(en_P[i], LOW);
          if(stepper_direction[i]){
            CW = true;
            cw +=1;
            digitalWrite(dir_P[i], HIGH);
          }
          else{
            CCW = true;
            ccw +=1;
            digitalWrite(dir_P[i], LOW);
          }
        }
        else{
          digitalWrite(en_P[i], HIGH);
          if(digitalRead(dir_P[i])){
            cw -=1;
            if(cw<1)
              CW = false;
          }
          else{
            ccw -=1;
            if(ccw<1)
              CCW = false;
          }
        }
    }
  }
}

void update_LEDS(){
  leds = 0;
  if(CW && CCW)
      del = 1;
    else
      del = 50;
  for (int i = 0; i < 8; i++){
    bitSet(leds, i);
    digitalWrite(LEDlatch, LOW);
    if(CW)
      shiftOut(LEDdata, LEDclock, LSBFIRST, leds);
    else if(CCW)
      shiftOut(LEDdata, LEDclock, MSBFIRST, leds);
    digitalWrite(LEDlatch, HIGH);
    delay(del);
  }
  if(del != 1){
    leds = 0;
    digitalWrite(LEDlatch, LOW);
    shiftOut(LEDdata, LEDclock, LSBFIRST, leds);
    digitalWrite(LEDlatch, HIGH);
    change = false;
  }
}
