#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

// ================= PINAGEM =================
const int frontalPins[8] = {26, 25, 33, 32, 35, 34, 39, 36}; 
const int sensorEsq = 27;  
const int sensorDir = 14;  

const int PWMA = 18, AIN1 = 22, AIN2 = 23; 
const int PWMB = 19, BIN1 = 15, BIN2 = 2;  
const int STBY = 21; 

const int botaoBoot = 13;

// ================= PID =================
float Kp = 15.0, Ki = 0.1, Kd = 8.0, lastError = 0, integral = 0;
int maxSpeed = 200;
int baseSpeed = 180;

// ================= CALIBRAÇÃO =================
int minValues[8] = {4095,4095,4095,4095,4095,4095,4095,4095};
int maxValues[8] = {0,0,0,0,0,0,0,0};

// ================= ESTADO =================
enum EstadoRobo { AGUARD_CALIB, CALIBRANDO, AGUARD_LARG, CORRENDO, PARADO };
EstadoRobo estadoAtual = AGUARD_CALIB;
bool telemetriaAtiva = false;

int voltas = 0;
unsigned long tempoTravaEsq = 0;
String btBuffer = "";

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  
  analogSetAttenuation(ADC_11db);
  
  for(int i=0; i<8; i++) pinMode(frontalPins[i], INPUT);
  pinMode(sensorEsq, INPUT); pinMode(sensorDir, INPUT);
  
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT); pinMode(PWMB,OUTPUT);
  pinMode(STBY,OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(botaoBoot, INPUT_PULLUP);

  SerialBT.println("C=Calibrar  S=Start/Stop");
  SerialBT.println("L=Telemetria  ?=Status");
  Serial.println("SEM ENCODER!");
}

// ================= FUNÇÕES =================
float lerSensor(int i) {
  return (i<2) ? (digitalRead(frontalPins[i]) ? 4095 : 0) : analogRead(frontalPins[i]);
}

void calibrarSensores() {
  SerialBT.println("CALIBRANDO 5s...");
  for(int i=0; i<8; i++) { minValues[i]=4095; maxValues[i]=0; }
  
  unsigned long t0 = millis();
  while(millis()-t0 < 5000) {
    for(int i=0; i<8; i++) {
      int v = lerSensor(i);
      minValues[i] = min(minValues[i], v);
      maxValues[i] = max(maxValues[i], v);
    }
    delay(5);
  }
  SerialBT.println("CALIB OK!");
}

void motores(int esq, int dir) {
  esq = constrain(esq, 0, maxSpeed);
  dir = constrain(dir, 0, maxSpeed);
  
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, esq);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, dir);
}

float erroLinha() {
  float p[8] = {-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5};
  float sp=0, sl=0;
  
  for(int i=0; i<8; i++) {
    int raw = lerSensor(i);
    int n = map(raw, minValues[i], maxValues[i], 1000, 0);
    n = constrain(n, 0, 1000);
    if(n > 250) { sp += n*p[i]; sl += n; }
  }
  return sl ? sp/sl : lastError;
}

float PID(float erro) {
  float prop = Kp * erro;
  integral += erro * Ki; integral = constrain(integral, -50, 50);
  float der = Kd * (erro - lastError);
  lastError = erro;
  return prop + integral + deriv;
}

void laterais() {
  if(digitalRead(sensorEsq) && millis()-tempoTravaEsq > 500) {
    voltas++; 
    tempoTravaEsq = millis();
    SerialBT.print("Volta "); SerialBT.println(voltas);
  }
}

void checarBT() {
  while(SerialBT.available()) {
    char c = SerialBT.read();
    if(c == '\n' || c == '\r') {
      if(btBuffer.length() > 0) {
        btBuffer.toUpperCase();
        Serial.print("CMD: "); Serial.println(btBuffer);
        
        if(btBuffer == "C") { 
          estadoAtual = CALIBRANDO; 
          SerialBT.println("CALIBRANDO...");
        }
        else if(btBuffer == "S") {
          if(estadoAtual == AGUARD_LARG) { 
            SerialBT.println("🚀 LARGADA!"); 
            estadoAtual = CORRENDO; 
          } else { 
            estadoAtual = PARADO; 
            SerialBT.println("🛑 PARADO");
          }
        }
        else if(btBuffer == "L") { 
          telemetriaAtiva = !telemetriaAtiva; 
          SerialBT.println(telemetriaAtiva ? "TELE ON" : "TELE OFF"); 
        }
        else if(btBuffer == "?") {
          SerialBT.printf("Kp=%.1f Kd=%.1f V=%d\n", Kp, Kd, baseSpeed);
        }
        btBuffer = "";
      }
    } else if(btBuffer.length() < 10) {
      btBuffer += c;
    }
  }
}

// ================= LOOP =================
void loop() {
  checarBT();
  
  if(digitalRead(botaoBoot) == LOW) {
    estadoAtual = CALIBRANDO;
    delay(300);
  }
  
  switch(estadoAtual) {
    case CALIBRANDO:
      calibrarSensores();
      estadoAtual = AGUARD_LARG;
      SerialBT.println("CALIBRADO! Envie S");
      break;
      
    case AGUARD_CALIB:
      SerialBT.println("Envie C");
      delay(1000);
      break;
      
    case AGUARD_LARG:
      motores(0,0);
      if(telemetriaAtiva) {
        SerialBT.printf("Aguardando...\n");
        delay(1000);
      }
      break;
      
    case PARADO:
      motores(0,0);
      delay(500);
      break;
      
    case CORRENDO:
      laterais();
      
      float e = erroLinha();
      float correcao = PID(e);
      motores(baseSpeed + correcao, baseSpeed - correcao);
      break;
  }
  
  delay(3);
}