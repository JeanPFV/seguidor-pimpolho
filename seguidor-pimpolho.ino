#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

// ================= PINAGEM =================
const int frontalPins[8] = {26, 25, 33, 32, 35, 34, 39, 36}; 

// SENSORES LATERAIS
const int sensorEsq = 27;  
const int sensorDir = 14;  

// MOTORES / PONTE H
const int PWMA = 18, AIN1 = 22, AIN2 = 23; 
const int PWMB = 19, BIN1 = 15, BIN2 = 2;  
const int STBY = 21; 

// ENCODERS
const int encEsqC1 = 16, encEsqC2 = 17;
const int encDirC1 = 4, encDirC2 = 5;

const int botaoBoot = 13;

// ================= PID =================
float Kp = 15.0, Ki = 0.1, Kd = 8.0, lastError = 0, integral = 0;
int maxSpeed = 200;
int baseSpeed = 180, targetRPM = 150;

// ================= CALIBRAÇÃO =================
int minValues[8] = {4095,4095,4095,4095,4095,4095,4095,4095};
int maxValues[8] = {0,0,0,0,0,0,0,0};

// ================= ESTADO =================
enum EstadoRobo { AGUARD_CALIB, CALIBRANDO, AGUARD_LARG, CORRENDO, PARADO };
EstadoRobo estadoAtual = AGUARD_CALIB;
bool telemetriaAtiva = false;

// VARIÁVEIS
int voltas = 0;
volatile long contadorEsq = 0, contadorDir = 0;
unsigned long ultimoTempoEnc = 0, tempoTravaEsq = 0;
String btBuffer = "";

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("OSS_FINAL"); 
  
  analogSetAttenuation(ADC_11db);
  
  // Sensores
  for(int i=0; i<8; i++) pinMode(frontalPins[i], INPUT);
  pinMode(sensorEsq, INPUT); pinMode(sensorDir, INPUT);
  
  // Motores
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT); pinMode(PWMB,OUTPUT);
  pinMode(STBY,OUTPUT); digitalWrite(STBY, HIGH);
  
  // Encoders
  pinMode(encEsqC1,INPUT_PULLUP); pinMode(encEsqC2,INPUT_PULLUP);
  pinMode(encDirC1,INPUT_PULLUP); pinMode(encDirC2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encEsqC1), [](){ 
    if(digitalRead(encEsqC2)) contadorEsq++; else contadorEsq--; 
  }, FALLING);
  attachInterrupt(digitalPinToInterrupt(encDirC1), [](){ 
    if(digitalRead(encDirC2)) contadorDir++; else contadorDir--; 
  }, FALLING);

  pinMode(botaoBoot, INPUT_PULLUP);

  SerialBT.println("C=Calibrar  S=Start/Stop");
  SerialBT.println("L=Telemetria  ?=Status");
  Serial.println("FINAL CARREGADO!");
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
  return prop + integral + der;
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
          SerialBT.printf("Kp=%.1f Kd=%.1f V=%d Estado=%d\n", Kp, Kd, baseSpeed, estadoAtual);
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
  unsigned long agora = millis();
  
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
      SerialBT.println("Envie C para calibrar");
      delay(1000);
      break;
      
    case AGUARD_LARG:
      motores(0,0);
      if(telemetriaAtiva) {
        SerialBT.printf("Aguardando... %ds\n", millis()/1000);
        delay(1000);
      }
      break;
      
    case PARADO:
      motores(0,0);
      SerialBT.println("PARADO - Envie S");
      delay(500);
      break;
      
    case CORRENDO:
      laterais();
      
      if(agora - ultimoTempoEnc > 50) {
        float v = (abs(contadorEsq) + abs(contadorDir)) * 5.0;
        baseSpeed += constrain(targetRPM - v, -5, 5);
        baseSpeed = constrain(baseSpeed, 120, 200);
        contadorEsq = contadorDir = 0;
        ultimoTempoEnc = agora;
      }
      
      float e = erroLinha();
      float correcao = PID(e);
      motores(baseSpeed + correcao, baseSpeed - correcao);
      break;
  }
  
  delay(4);
}