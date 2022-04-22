#include "FS.h"
#include <Time.h>
#include "SPIFFS.h"
#include <BLE2902.h>
#include <BLEUtils.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <TimeAlarms.h>
#include <ArduinoJson.h>

int Prime_time = 20000; //20000 milli sec = 20 sec
int stirrer_time = 20000; //20000 milli sec = 20 sec

DynamicJsonDocument doc(1024);
File file;
bool f = false;

int sh,sm,ss,sdow,id,del = 0;
bool setAlarm, DA, SAA = false;

int MRes = 8; //PWM Resolution
int MP[] = {4,5,13,15,18,19,23};  //Stepper Step Pins
int MC[] ={0,2,4,6,8,10,12}; //PWM Channels
int MF[] ={400,400,400,400,400,400,400}; //speed in terms of frequency Hz

bool MS[] = {false,false,false,false,false,false,false};
bool MD[] = {false,false,false,false,false,false,false};
bool btn[] = {false,false,false,false,false,false,false};
bool ble[] = {false,false,false,false,false,false,false};
float VOL[] = {100,0,0,0,0,0,0};

unsigned long ST[] = {0,0,0,0,0,0,0};

bool SCHEDULE[] = {false,false,false,false,false,false,false};
bool REMOVE[] = {false,false,false,false,false,false,false};

AlarmID_t availableTimers[28];


timeDayOfWeek_t DAYS_of_WEEK[] = {dowMonday,dowTuesday,dowWednesday,dowThursday,dowFriday,dowSaturday,dowSunday};

bool SettingUp = false;
bool isScedule[28];
bool RunnungSHED[28];
int startTime[28];
int runTime[28];
int Drops[28];
int Smode[28];
bool DIRECTION[28];

void Start_Stepper(int i){
  if(!MS[i]){
    ledcSetup(MC[i], MF[i], MRes);
    ledcAttachPin(MP[i], MC[i]);
    ledcWrite(MC[i], 125);
    ST[i] = millis();
    if(!REMOVE[i])
      Serial.println("Starting Pump: " + String(i+1));
    MS[i] = true;
  }
}

void Stop_Stepper(int i){
  if(MS[i]){
    ledcWrite(MC[i], 0);
    MS[i] = false;

    if(!REMOVE[i]){
      float duration = millis() - ST[i];
      if(SCHEDULE[i]){
        duration -= Prime_time;
        if(i<4)
          duration -= stirrer_time;
      }
      if(MD[i])
        VOL[i] += (duration)/18000;
      else
        VOL[i] -= (duration)/18000;
        
      Serial.println("Pump " + String(i+1) + " Run for " + String(duration/1000) + " Sec, New VOLUME " + String(VOL[i]));
    }
    
    if(SCHEDULE[i]){
      Serial1.print("{'A':'P','Pn':'" + String(i+1) + "','Ps':'1','Pd':'0','sh':'0'}");
      REMOVE[i] = true;
      Start_Stepper(i);
      SCHEDULE[i] = false;
      Serial.println("Rmoving From Pump: " + String(i+1));
    }
    else{
      Serial1.print("{'A':'P','Pn':'" + String(i+1) + "','Ps':'0','Pd':'0','sh':'0'}");
      Serial.println("Stopping Pump: " + String(i+1));
      REMOVE[i] = false;
    }
  }
}

typedef void (*function) () ;

//P1
void P1T1(){
  if(!SettingUp && !SCHEDULE[0]){
    RunnungSHED[0] = SCHEDULE[0] = true;
    if(!MS[0]){
      Serial1.print("{'A':'P','Pn':'1','Ps':'1','Pd':'" + String(DIRECTION[0]) +"','sh':'1'}");
      runTime[0] = 720 * Drops[0] + Prime_time + stirrer_time;
      MD[0] = DIRECTION[0];
      Start_Stepper(0);
    }
    if(Smode[0] == 0){
      isScedule[0] = false;
      availableTimers[0] = -1;
    }
  }
  SettingUp = false;
}
void P1T2(){
  if(!SettingUp && !SCHEDULE[0]){
    RunnungSHED[1] = SCHEDULE[0] = true;
    if(!MS[0]){
      Serial1.print("{'A':'P','Pn':'1','Ps':'1','Pd':'" + String(DIRECTION[1]) +"','sh':'1'}");
      runTime[1] = 720 * Drops[1] + Prime_time + stirrer_time;
      MD[0] = DIRECTION[1];
      Start_Stepper(0);
    }
    if(Smode[1] == 0){
      isScedule[1] = false;
      availableTimers[1] = -1;
    }
  }
  SettingUp = false;
}
void P1T3(){
  if(!SettingUp && !SCHEDULE[0]){
    RunnungSHED[2] = SCHEDULE[0] = true;
    if(!MS[0]){
      Serial1.print("{'A':'P','Pn':'1','Ps':'1','Pd':'" + String(DIRECTION[2]) +"','sh':'1'}");
      runTime[2] = 720 * Drops[2] + Prime_time + stirrer_time;
      MD[0] = DIRECTION[2];
      Start_Stepper(0);
    }
    if(Smode[2] == 0){
      isScedule[2] = false;
      availableTimers[2] = -1;
    }
  }
  SettingUp = false;
}
void P1T4(){
  if(!SettingUp && !SCHEDULE[0]){
    RunnungSHED[3] = SCHEDULE[0] = true;
    if(!MS[0]){
      Serial1.print("{'A':'P','Pn':'1','Ps':'1','Pd':'" + String(DIRECTION[3]) +"','sh':'1'}");
      runTime[3] = 720 * Drops[3] + Prime_time + stirrer_time;
      MD[0] = DIRECTION[3];
      Start_Stepper(0);
    }
    if(Smode[3] == 0){
      isScedule[3] = false;
      availableTimers[3] = -1;
    }
  }
  SettingUp = false;
}

//P2
void P2T1(){
  if(!SettingUp && !SCHEDULE[1]){
    RunnungSHED[4] = SCHEDULE[1] = true;
    if(!MS[1]){
      Serial1.print("{'A':'P','Pn':'2','Ps':'1','Pd':'" + String(DIRECTION[4]) +"','sh':'1'}");
      runTime[4] = 720 * Drops[4] + Prime_time + stirrer_time;
      MD[1] = DIRECTION[4];
      Start_Stepper(1);
    }
    if(Smode[4] == 0){
      isScedule[4] = false;
      availableTimers[4] = -1;
    }
  }
  SettingUp = false;
}
void P2T2(){
  if(!SettingUp && !SCHEDULE[1]){
    RunnungSHED[5] = SCHEDULE[1] = true;
    if(!MS[1]){
      Serial1.print("{'A':'P','Pn':'2','Ps':'1','Pd':'" + String(DIRECTION[5]) +"','sh':'1'}");
      runTime[5] = 720 * Drops[5] + Prime_time + stirrer_time;
      MD[1] = DIRECTION[5];
      Start_Stepper(1);
    }
    if(Smode[5] == 0){
      isScedule[5] = false;
      availableTimers[5] = -1;
    }
  }
  SettingUp = false;
}
void P2T3(){
  if(!SettingUp && !SCHEDULE[1]){
    RunnungSHED[6] = SCHEDULE[1] = true;
    if(!MS[1]){
      Serial1.print("{'A':'P','Pn':'2','Ps':'1','Pd':'" + String(DIRECTION[6]) +"','sh':'1'}");
      runTime[6] = 720 * Drops[6] + Prime_time + stirrer_time;
      MD[1] = DIRECTION[6];
      Start_Stepper(1);
    }
    if(Smode[6] == 0){
      isScedule[6] = false;
      availableTimers[6] = -1;
    }
  }
  SettingUp = false;
}
void P2T4(){
  if(!SettingUp && !SCHEDULE[1]){
    RunnungSHED[7] = SCHEDULE[1] = true;
    if(!MS[1]){
      Serial1.print("{'A':'P','Pn':'2','Ps':'1','Pd':'" + String(DIRECTION[7]) +"','sh':'1'}");
      runTime[7] = 720 * Drops[7] + Prime_time + stirrer_time;
      MD[1] = DIRECTION[7];
      Start_Stepper(1);
    }
    if(Smode[7] == 0){
      isScedule[7] = false;
      availableTimers[7] = -1;
    }
  }
  SettingUp = false;
}

//P3
void P3T1(){
  if(!SettingUp && !SCHEDULE[2]){
    RunnungSHED[8] = SCHEDULE[2] = true;
    if(!MS[2]){
      Serial1.print("{'A':'P','Pn':'3','Ps':'1','Pd':'" + String(DIRECTION[8]) +"','sh':'1'}");
      runTime[8] = 720 * Drops[8] + Prime_time + stirrer_time;
      MD[2] = DIRECTION[8];
      Start_Stepper(2);
    }
    if(Smode[8] == 0){
      isScedule[8] = false;
      availableTimers[8] = -1;
    }
  }
  SettingUp = false;
}
void P3T2(){
  if(!SettingUp && !SCHEDULE[2]){
    RunnungSHED[9] = SCHEDULE[2] = true;
    if(!MS[2]){
      Serial1.print("{'A':'P','Pn':'3','Ps':'1','Pd':'" + String(DIRECTION[9]) +"','sh':'1'}");
      runTime[9] = 720 * Drops[9] + Prime_time + stirrer_time;
      MD[2] = DIRECTION[9];
      Start_Stepper(2);
    }
    if(Smode[9] == 0){
      isScedule[9] = false;
      availableTimers[9] = -1;
    }
  }
  SettingUp = false;
}
void P3T3(){
  if(!SettingUp && !SCHEDULE[2]){
    RunnungSHED[10] = SCHEDULE[2] = true;
    if(!MS[2]){
      Serial1.print("{'A':'P','Pn':'3','Ps':'1','Pd':'" + String(DIRECTION[10]) +"','sh':'1'}");
      runTime[10] = 720 * Drops[10] + Prime_time + stirrer_time;
      MD[2] = DIRECTION[10];
      Start_Stepper(2);
    }
    if(Smode[10] == 0){
      isScedule[10] = false;
      availableTimers[10] = -1;
    }
  }
  SettingUp = false;
}
void P3T4(){
  if(!SettingUp && !SCHEDULE[2]){
    RunnungSHED[11] = SCHEDULE[2] = true;
    if(!MS[2]){
      Serial1.print("{'A':'P','Pn':'3','Ps':'1','Pd':'" + String(DIRECTION[11]) +"','sh':'1'}");
      runTime[11] = 720 * Drops[11] + Prime_time + stirrer_time;
      MD[2] = DIRECTION[11];
      Start_Stepper(2);
    }
    if(Smode[11] == 0){
      isScedule[11] = false;
      availableTimers[11] = -1;
    }
  }
  SettingUp = false;
}

//P4
void P4T1(){
  if(!SettingUp && !SCHEDULE[3]){
    RunnungSHED[12] = SCHEDULE[3] = true;
    if(!MS[3]){
      Serial1.print("{'A':'P','Pn':'4','Ps':'1','Pd':'" + String(DIRECTION[12]) +"','sh':'1'}");
      runTime[12] = 720 * Drops[12] + Prime_time + stirrer_time;
      MD[3] = DIRECTION[12];
      Start_Stepper(3);
    }
    if(Smode[12] == 0){
      isScedule[12] = false;
      availableTimers[12] = -1;
    }
  }
  SettingUp = false;
}
void P4T2(){
  if(!SettingUp && !SCHEDULE[3]){
    RunnungSHED[13] = SCHEDULE[3] = true;
    if(!MS[3]){
      Serial1.print("{'A':'P','Pn':'4','Ps':'1','Pd':'" + String(DIRECTION[13]) +"','sh':'1'}");
      runTime[13] = 720 * Drops[13] + Prime_time + stirrer_time;
      MD[3] = DIRECTION[13];
      Start_Stepper(3);
    }
    if(Smode[13] == 0){
      isScedule[13] = false;
      availableTimers[13] = -1;
    }
  }
  SettingUp = false;
}
void P4T3(){
  if(!SettingUp && !SCHEDULE[3]){
    RunnungSHED[14] = SCHEDULE[3] = true;
    if(!MS[3]){
      Serial1.print("{'A':'P','Pn':'4','Ps':'1','Pd':'" + String(DIRECTION[14]) +"','sh':'1'}");
      runTime[14] = 720 * Drops[14] + Prime_time + stirrer_time;
      MD[3] = DIRECTION[14];
      Start_Stepper(3);
    }
    if(Smode[14] == 0){
      isScedule[14] = false;
      availableTimers[14] = -1;
    }
  }
  SettingUp = false;
}
void P4T4(){
  if(!SettingUp && !SCHEDULE[3]){
    RunnungSHED[15] = SCHEDULE[3] = true;
    if(!MS[3]){
      Serial1.print("{'A':'P','Pn':'4','Ps':'1','Pd':'" + String(DIRECTION[15]) +"','sh':'1'}");
      runTime[15] = 720 * Drops[15] + Prime_time + stirrer_time;
      MD[3] = DIRECTION[15];
      Start_Stepper(3);
    }
    if(Smode[15] == 0){
      isScedule[15] = false;
      availableTimers[15] = -1;
    }
  }
  SettingUp = false;
}

//P5
void P5T1(){
  if(!SettingUp && !SCHEDULE[4]){
    RunnungSHED[16] = SCHEDULE[4] = true;
    if(!MS[4]){
      Serial1.print("{'A':'P','Pn':'5','Ps':'1','Pd':'" + String(DIRECTION[16]) +"','sh':'1'}");
      runTime[16] = 720 * Drops[16] + Prime_time;
      MD[4] = DIRECTION[16];
      Start_Stepper(4);
    }
    if(Smode[16] == 0){
      isScedule[16] = false;
      availableTimers[16] = -1;
    }
  }
  SettingUp = false;
}
void P5T2(){
  if(!SettingUp && !SCHEDULE[4]){
    RunnungSHED[17] = SCHEDULE[4] = true;
    if(!MS[4]){
      Serial1.print("{'A':'P','Pn':'5','Ps':'1','Pd':'" + String(DIRECTION[17]) +"','sh':'1'}");
      runTime[17] = 720 * Drops[17] + Prime_time;
      MD[4] = DIRECTION[17];
      Start_Stepper(4);
    }
    if(Smode[17] == 0){
      isScedule[17] = false;
      availableTimers[17] = -1;
    }
  }
  SettingUp = false;
}
void P5T3(){
  if(!SettingUp && !SCHEDULE[4]){
    RunnungSHED[18] = SCHEDULE[4] = true;
    if(!MS[4]){
      Serial1.print("{'A':'P','Pn':'5','Ps':'1','Pd':'" + String(DIRECTION[18]) +"','sh':'1'}");
      runTime[18] = 720 * Drops[18] + Prime_time;
      MD[4] = DIRECTION[18];
      Start_Stepper(4);
    }
    if(Smode[18] == 0){
      isScedule[18] = false;
      availableTimers[18] = -1;
    }
  }
  SettingUp = false;
}
void P5T4(){
  if(!SettingUp && !SCHEDULE[4]){
    RunnungSHED[19] = SCHEDULE[4] = true;
    if(!MS[4]){
      Serial1.print("{'A':'P','Pn':'5','Ps':'1','Pd':'" + String(DIRECTION[19]) +"','sh':'1'}");
      runTime[19] = 720 * Drops[19] + Prime_time;
      MD[4] = DIRECTION[19];
      Start_Stepper(4);
    }
    if(Smode[19] == 0){
      isScedule[19] = false;
      availableTimers[19] = -1;
    }
  }
  SettingUp = false;
}

//P6
void P6T1(){
  if(!SettingUp && !SCHEDULE[51]){
    RunnungSHED[20] = SCHEDULE[5] = true;
    if(!MS[5]){
      Serial1.print("{'A':'P','Pn':'6','Ps':'1','Pd':'" + String(DIRECTION[20]) +"','sh':'1'}");
      runTime[20] = 720 * Drops[20] + Prime_time;
      MD[5] = DIRECTION[20];
      Start_Stepper(5);
    }
    if(Smode[20] == 0){
      isScedule[20] = false;
      availableTimers[20] = -1;
    }
  }
  SettingUp = false;
}
void P6T2(){
  if(!SettingUp && !SCHEDULE[5]){
    RunnungSHED[21] = SCHEDULE[5] = true;
    if(!MS[5]){
      Serial1.print("{'A':'P','Pn':'6','Ps':'1','Pd':'" + String(DIRECTION[21]) +"','sh':'1'}");
      runTime[21] = 720 * Drops[21] + Prime_time;
      MD[5] = DIRECTION[21];
      Start_Stepper(5);
    }
    if(Smode[21] == 0){
      isScedule[21] = false;
      availableTimers[21] = -1;
    }
  }
  SettingUp = false;
}
void P6T3(){
  if(!SettingUp && !SCHEDULE[5]){
    RunnungSHED[22] = SCHEDULE[5] = true;
    if(!MS[5]){
      Serial1.print("{'A':'P','Pn':'6','Ps':'1','Pd':'" + String(DIRECTION[22]) +"','sh':'1'}");
      runTime[22] = 720 * Drops[22] + Prime_time;
      MD[5] = DIRECTION[22];
      Start_Stepper(5);
    }
    if(Smode[22] == 0){
      isScedule[22] = false;
      availableTimers[22] = -1;
    }
  }
  SettingUp = false;
}
void P6T4(){
  if(!SettingUp && !SCHEDULE[5]){
    RunnungSHED[23] = SCHEDULE[5] = true;
    if(!MS[5]){
      Serial1.print("{'A':'P','Pn':'6','Ps':'1','Pd':'" + String(DIRECTION[23]) +"','sh':'1'}");
      runTime[23] = 720 * Drops[23] + Prime_time;
      MD[5] = DIRECTION[23];
      Start_Stepper(5);
    }
    if(Smode[23] == 0){
      isScedule[23] = false;
      availableTimers[23] = -1;
    }
  }
  SettingUp = false;
}

//P7
void P7T1(){
  if(!SettingUp && !SCHEDULE[6]){
    RunnungSHED[24] = SCHEDULE[6] = true;
    if(!MS[6]){
      Serial1.print("{'A':'P','Pn':'7','Ps':'1','Pd':'" + String(DIRECTION[24]) +"','sh':'1'}");
      runTime[24] = 720 * Drops[24] + Prime_time;
      MD[6] = DIRECTION[24];
      Start_Stepper(6);
    }
    if(Smode[24] == 0){
      isScedule[24] = false;
      availableTimers[24] = -1;
    }
  }
  SettingUp = false;
}
void P7T2(){
  if(!SettingUp && !SCHEDULE[6]){
    RunnungSHED[25] = SCHEDULE[6] = true;
    if(!MS[6]){
      Serial1.print("{'A':'P','Pn':'7','Ps':'1','Pd':'" + String(DIRECTION[25]) +"','sh':'1'}");
      runTime[25] = 720 * Drops[25] + Prime_time;
      MD[6] = DIRECTION[25];
      Start_Stepper(6);
    }
    if(Smode[25] == 0){
      isScedule[25] = false;
      availableTimers[25] = -1;
    }
  }
  SettingUp = false;
}
void P7T3(){
  if(!SettingUp && !SCHEDULE[6]){
    RunnungSHED[26] = SCHEDULE[6] = true;
    if(!MS[6]){
      Serial1.print("{'A':'P','Pn':'7','Ps':'1','Pd':'" + String(DIRECTION[26]) +"','sh':'1'}");
      runTime[26] = 720 * Drops[26] + Prime_time;
      MD[6] = DIRECTION[26];
      Start_Stepper(6);
    }
    if(Smode[26] == 0){
      isScedule[26] = false;
      availableTimers[26] = -1;
    }
  }
  SettingUp = false;
}
void P7T4(){
  if(!SettingUp && !SCHEDULE[6]){
    RunnungSHED[27] = SCHEDULE[6] = true;
    if(!MS[6]){
      Serial1.print("{'A':'P','Pn':'7','Ps':'1','Pd':'" + String(DIRECTION[27]) +"','sh':'1'}");
      runTime[27] = 720 * Drops[27] + Prime_time;
      MD[6] = DIRECTION[27];
      Start_Stepper(6);
    }
    if(Smode[27] == 0){
      isScedule[27] = false;
      availableTimers[27] = -1;
    }
  }
  SettingUp = false;
}


function AlarmFunction[] = {P1T1,P1T2,P1T3,P1T4,P2T1,P2T2,P2T3,P2T4,P3T1,P3T2,P3T3,P3T4,P4T1,P4T2,P4T3,P4T4,P5T1,P5T2,P5T3,P5T4,P6T1,P6T2,P6T3,P6T4,P7T1,P7T2,P7T3,P7T4};

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

bool isConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    BLEDevice::stopAdvertising();
    isConnected = true;
  }

  void onDisconnect(BLEServer* pServer){
    isConnected = false;
    BLEDevice::startAdvertising();
  }
};

void DeleteAlarm(String AN){
  int pumpNumber = del/4;
  if(isScedule[del]){
    if(!SCHEDULE[pumpNumber]){
      Serial.print("Deleting Schedule for pump ");
      Serial.println(pumpNumber + 1);
      file = SPIFFS.open("/Alarms"+AN+".txt", FILE_WRITE);
      file.print("");
      file.close();
      Alarm.disable(availableTimers[del]);
      availableTimers[del] = -1;
      isScedule[del] = false;
      del = 0;
    }
    else
      Serial.println("Unable to delete schedule is Running");
  }
  else
    Serial.println("Schedule not available");
}

String Payload = "";
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0){
      for (int i = 0; i < value.length(); i++)
        Payload += value[i];
      if(value[value.length()-1] == '}'){
        Serial.print("Received: ");
        Serial.println(Payload);
        deserializeJson(doc, Payload);
        JsonObject obj = doc.as<JsonObject>();
        String act = obj[String("A")].as<String>();
        if(act == "P"){
          
          Payload = Payload.substring(0, Payload.length()-1) + ",'sh':'0'}";
          int P = obj[String("Pn")].as<int>() - 1;
          bool State = obj[String("Ps")].as<int>();
          bool PD = obj[String("Pd")].as<int>();
          if(State & !MS[P]){
            ble[P] = State;
            Start_Stepper(P);
            MD[P] = PD;
            Serial1.print(Payload);
          }
          else if(!State & !btn[P] & !SCHEDULE[P]){
            ble[P] = State;
            Stop_Stepper(P);
          }
        }
        else if(act == "S"){
          Serial1.print(Payload);
        }

        else if(act == "V"){
          int P = obj[String("Pn")].as<int>() - 1;
          float val = obj[String("VAL")].as<float>();
          VOL[P] = val;
        }
        
        else if(act == "DT"){
          Serial1.print(Payload);
          
          int h = obj[String("h")].as<int>();
          int m = obj[String("m")].as<int>();
          int s = obj[String("s")].as<int>();
          int dom = obj[String("dom")].as<int>();
          int mth = obj[String("M")].as<int>();
          int yr = obj[String("Y")].as<int>();
          setTime(h,m,s,mth,dom,yr);
          for(int a=0; a<28; a++){
            if(availableTimers[a] != 255){
              Alarm.disable(availableTimers[a]);
              isScedule[a] = false;
              availableTimers[a] = -1;
            }
          }
          SAA = true;
        }
        else if(act == "SCH"){
          int P = obj[String("Pn")].as<int>() - 1;
          int s = P*4;
          int e = s+4;
          for(int i=s; i<e; i++){
            if(!isScedule[i]){
              if(f){
                isScedule[i] = true;
                sh = obj[String("h")].as<int>();
                sm = obj[String("m")].as<int>();
                ss = obj[String("s")].as<int>();
                Drops[i] = obj[String("D")].as<int>();
                Smode[i] = obj[String("M")].as<int>();
                if(Smode[i] != 2)
                  sdow = obj[String("dow")].as<int>()-1;
                else
                  sdow = -1;
                DIRECTION[i] = obj[String("AR")].as<int>();
                setAlarm = true;
                id = i;
                String ReadData = "";
                String fileName = "/Alarms"+String(id)+".txt";
                file = SPIFFS.open(fileName, FILE_WRITE);
                ReadData = String(id) + "," + String(sh) + "," + String(sm) + "," + String(ss) + "," + String(Smode[i]) + "," + String(sdow) + "," + String(DIRECTION[i]) + "," + String(Drops[i]) + ",";
                file.println(ReadData);
                
                file.close();
                Serial.println("Schedule is Set");
                break;
              }
            }
          }
        }

        else if(act == "DS"){
          int P = obj[String("Pn")].as<int>() - 1;
          int SN = obj[String("Sn")].as<int>() - 1;
          int i = (P*4)+SN;
          DA = true;
          del = i;
        }
        Payload = "";
      }
    }
  }
};
 
void setup(){
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  for(int i=0; i<28; i++){
    RunnungSHED[i] = isScedule[i] = false;
    availableTimers[i] = -1;
  }
  if (SPIFFS.begin()){
    Serial.println(F("SPIFFS mounted correctly."));
  }
    

  Serial.println("Initializing BLE");
  BLEDevice::init("Reef Dripper");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("BLE Initialization Complete");
  
  ledcSetup(MC[0], MF[0], MRes);
  ledcAttachPin(MP[0], MC[0]);
  
  ledcSetup(MC[1], MF[1], MRes);
  ledcAttachPin(MP[1], MC[1]);
  
  ledcSetup(MC[2], MF[2], MRes);
  ledcAttachPin(MP[2], MC[2]);
  
  ledcSetup(MC[3], MF[3], MRes);
  ledcAttachPin(MP[3], MC[0]);
  
  ledcSetup(MC[4], MF[4], MRes);
  ledcAttachPin(MP[4], MC[0]);
  
  ledcSetup(MC[5], MF[5], MRes);
  ledcAttachPin(MP[5], MC[0]);
  
  ledcSetup(MC[6], MF[6], MRes);
  ledcAttachPin(MP[6], MC[0]);
}

unsigned int pre = 0;
 
void loop(){
  if(!f){
    if(millis()- pre > 5000){
      Serial1.print("{'A':'UDT'}");
      pre = millis();
    }
  }
  Alarm.delay(1);
  Read_Mega();
  for(int i=0; i<7; i++){
    if(REMOVE[i]){
      if(millis()-ST[i] >= 60000){
        Stop_Stepper(i);
      }
    }
  }
  for(int i=0; i<7; i++){
    if(SCHEDULE[i]){
      int j=i*4;
      int l=j+4;
      for(int k=j; k<l; k++){
        if(RunnungSHED[k]){
          if(millis() - ST[i] >= runTime[k]){
            Stop_Stepper(i);
            RunnungSHED[k] = false;
          }
        }
      }
    }
  }
  if(DA){
    DeleteAlarm(String(del));
    DA = false;
  }
  if(setAlarm){
    switch(Smode[id]){
      case 0:{
        availableTimers[id] = Alarm.alarmOnce(DAYS_of_WEEK[sdow], sh, sm, ss,  AlarmFunction[id]);
        break;}
      case 1:{
        availableTimers[id] = Alarm.alarmRepeat(DAYS_of_WEEK[sdow], sh, sm, ss,  AlarmFunction[id]);
        break;}
      case 2:{
        availableTimers[id] = Alarm.alarmRepeat(sh, sm, ss,  AlarmFunction[id]);
        break;}
    }
    setAlarm = false;
  }
  if(SAA)
    setAllAlarms();
}

void Read_Mega(){
  if(Serial1.available() > 0){
    String payload = Serial1.readString();
    deserializeJson(doc, payload);
    JsonObject obj = doc.as<JsonObject>();
    String act = obj[String("PS")].as<String>();
    if(act == "P"){
      int x = obj[String("S")].as<int>();
      bool State = obj[String("A")].as<int>();
      bool D = obj[String("Pd")].as<int>();
      if(State & !MS[x]){
        Start_Stepper(x);
        btn[x] = State;
        MD[x] = D;
      }
      else if(!State & btn[x]){
        Stop_Stepper(x);
        btn[x] = State;
      }
    }
    else if(act == "UT"){
      int h = obj[String("h")].as<int>();
      int m = obj[String("m")].as<int>();
      int s = obj[String("s")].as<int>();
      int dom = obj[String("dom")].as<int>();
      int mth = obj[String("M")].as<int>();
      int yr = obj[String("Y")].as<int>();
      setTime(h,m,s,dom,mth,yr);
      Serial.println("ESP time updated");
      Serial.print(weekday());
      Serial.print(":");
      Serial.print(month());
      Serial.print(":");
      Serial.print(year());
      Serial.print("  ");
      Serial.print(hour());
      Serial.print(":");
      Serial.print(minute());
      Serial.print(":");
      Serial.print(second());
      Serial.println();
      SAA = true;
      f = true;
    }
  }
}

void setAllAlarms(){
  Serial.println("Setting Alarms");
  int DATA[8];
  int x=0;
  String RD = "";
  for (int p=0; p<28; p++){
    file = SPIFFS.open("/Alarms"+String(p)+".txt", FILE_READ);
    size_t SIZE = file.size();
    if(SIZE>5){
      String ReadData = "";
      while(file.available()){
        ReadData += file.readString();
      }
      for(int a=0; a<SIZE; a++){
        if(ReadData[a] !=',')
          RD += ReadData[a];
        else{
          DATA[x] = RD.toInt();
          RD = "";
          x += 1;
        }
      }
      x=0;
      file.close();
    }
    else{
      file.close();
    }
    DIRECTION[DATA[0]] = DATA[6];
    Drops[DATA[0]] = DATA[7];
    isScedule[DATA[0]] = true;
    switch(DATA[4]){
      case 0:{
          availableTimers[DATA[0]] = Alarm.alarmOnce(DAYS_of_WEEK[DATA[5]], DATA[1], DATA[2], DATA[3],  AlarmFunction[DATA[0]]);
          break;
        }
      case 1:{
          availableTimers[DATA[0]] = Alarm.alarmRepeat(DAYS_of_WEEK[DATA[5]], DATA[1], DATA[2], DATA[3],  AlarmFunction[DATA[0]]);
          break;
      }
      case 2:{
          availableTimers[DATA[0]] = Alarm.alarmRepeat(DATA[1], DATA[2], DATA[3],  AlarmFunction[DATA[0]]);
          break;
      }
    }
    DATA[4] = -1;
  }
  SAA = false;
}
