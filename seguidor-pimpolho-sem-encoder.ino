#include <Arduino.h>
#include "BluetoothSerial.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

// ================= PINAGEM =================
const int frontalPins[8] = { 26, 36, 33, 32, 35, 34, 39, 25 };

const int sensorEsq = 27;
const int sensorDir = 14;

// Ponte H
const int PWMA = 18;
const int AIN1 = 22;
const int AIN2 = 23;

const int PWMB = 19;
const int BIN1 = 16;
const int BIN2 = 17;

const int STBY = 21;
const int botaoBoot = 0;

// ================= PWM LEDC =================
const int freqPWM = 1000;
const int resolucaoPWM = 8;

// ================= PID =================
float Kp = 38.0;
float Ki = 0.0;
float Kd = 13.0;


float lastError = 0.0;
float integral = 0.0;
float ultimoErroValido = 0.0;

int maxSpeed = 150;
int baseSpeed = 100;

// ================= CALIBRAÇÃO =================
int minValues[8] = { 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095 };
int maxValues[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// ================= ESTADO =================
enum EstadoRobo { AGUARD_CALIB,
                  CALIBRANDO,
                  AGUARD_LARG,
                  CORRENDO,
                  PARADO };
EstadoRobo estadoAtual = AGUARD_CALIB;

bool telemetriaAtiva = false;
int voltas = 0;
unsigned long tempoTravaEsq = 0;
unsigned long ultimoEnvioTelemetria = 0;
unsigned long ultimoDebounceBoot = 0;

// NOVO: controle da volta pelo sensor direito
int contagemDireita = 0;
bool sensorDirEmLinha = false;
unsigned long tempoTravaDir = 0;
const unsigned long debounceDir = 300;

int contagemMarcas = 0;
unsigned long tempoInicioCorrida = 0;
const unsigned long tempoMinimoChegada = 1;  // ajuste conforme a pista

const int totalPontosBrancos = 28;
const int toleranciaPontos = 1;


// ================= BUFFER BT =================
char btBuffer[24];
uint8_t btIndex = 0;

// ================= FUNÇÕES AUXILIARES =================
int lerSensor(int i) {
  if (i == 0 || i == 7) {
    return digitalRead(frontalPins[i]) ? 4095 : 0;
  }
  return analogRead(frontalPins[i]);
}

int normalizarLeitura(int raw, int minV, int maxV) {
  if (maxV <= minV) return 0;

  long valor = map(raw, minV, maxV, 1000, 0);
  valor = constrain(valor, 0, 1000);
  return (int)valor;
}

void resetarControle() {
  lastError = 0.0;
  integral = 0.0;
  ultimoErroValido = 0.0;
}

void pararMotores() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void mostrarParametros() {
  SerialBT.printf("P=%.3f I=%.3f D=%.3f V=%d M=%d Estado=%d Voltas=%d\n",
                  Kp, Ki, Kd, baseSpeed, maxSpeed, estadoAtual, voltas);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("PIMPOLHO_FINAL");

  for (int i = 0; i < 8; i++) pinMode(frontalPins[i], INPUT);

  pinMode(sensorEsq, INPUT_PULLUP);
  pinMode(sensorDir, INPUT_PULLUP);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(botaoBoot, INPUT_PULLUP);

  ledcAttach(PWMA, freqPWM, resolucaoPWM);
  ledcAttach(PWMB, freqPWM, resolucaoPWM);

  pararMotores();
  resetarControle();

  btBuffer[0] = '\0';

  SerialBT.println("=== PIMPOLHO COMPETICAO ===");
  SerialBT.println("C=Calibrar | S=Start/Stop | L=Telemetria | ?=Status");
  SerialBT.println("P=15.0 | I=0.00 | D=8.0 | V=100 | M=150 | RST");
  mostrarParametros();
}

// ================= CONTROLE =================

void enviarTelemetriaSensores() {
  SerialBT.print("Sensores: ");

  for (int i = 0; i < 8; i++) {
    int raw = lerSensor(i);
    int n = normalizarLeitura(raw, minValues[i], maxValues[i]);
    SerialBT.print(n);

    if (i < 7) SerialBT.print(" ");
  }

  SerialBT.print(" | DIR=");
  SerialBT.println(digitalRead(sensorDir));
}

void calibrarSensores() {
  SerialBT.println("CALIBRANDO 5s... Mova na pista!");

  for (int i = 0; i < 8; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    for (int i = 0; i < 8; i++) {
      int v = lerSensor(i);
      minValues[i] = min(minValues[i], v);
      maxValues[i] = max(maxValues[i], v);
    }
    delay(5);
  }

  resetarControle();
  telemetriaAtiva = false;

  contagemDireita = 0;
  contagemMarcas = 0;
  sensorDirEmLinha = false;
  tempoTravaDir = 0;
  tempoInicioCorrida = 0;

  SerialBT.println("CALIB OK! Envie 'S' para largar.");
}

void motores(int esq, int dir) {
  esq = constrain(esq, -maxSpeed, maxSpeed);
  dir = constrain(dir, -maxSpeed, maxSpeed);

  if (esq >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, esq);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, -esq);
  }

  if (dir >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, dir);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, -dir);
  }
}

float erroLinha() {
  const float p[8] = {-3.5, -2.5, -1.2, -0.4, 0.4, 1.2, 2.5, 3.5};
  float somaPesada = 0.0;
  float somaLeituras = 0.0;

  for (int i = 0; i < 8; i++) {
    int raw = lerSensor(i);
    int n = normalizarLeitura(raw, minValues[i], maxValues[i]);

    if (n > 150) {
      somaPesada += n * p[i];
      somaLeituras += n;
    }
  }

  if (somaLeituras > 0.0) {
    ultimoErroValido = somaPesada / somaLeituras;
    return ultimoErroValido;
  }

  return ultimoErroValido;
}

float PID(float erro) {
  float prop = Kp * erro;

  integral += erro;
  integral = constrain(integral, -50.0, 50.0);
  float integ = Ki * integral;

  float der = Kd * (erro - lastError);
  lastError = erro;

  return prop + integ + der;
}

bool frontaisIndicamInterseccao() {
  int ativosTotal = 0;
  int ativosCentro = 0;
  int soma = 0;

  for (int i = 0; i < 8; i++) {
    int raw = lerSensor(i);
    int n = normalizarLeitura(raw, minValues[i], maxValues[i]);

    if (n > 400) {
      ativosTotal++;
      if (i >= 2 && i <= 5) ativosCentro++;
    }

    soma += n;
  }

  return (ativosTotal >= 5 && ativosCentro >= 3 && soma >= 3200);
}

void verificarVoltaSensorDireito() {
  unsigned long agora = millis();
  bool brancoAgora = (digitalRead(sensorDir) == LOW);

  if (brancoAgora && !sensorDirEmLinha && (agora - tempoTravaDir > debounceDir)) {
    sensorDirEmLinha = true;
    tempoTravaDir = agora;

    contagemMarcas++;
    SerialBT.printf("MARCA DIREITA: %d\n", contagemMarcas);

    if (tempoInicioCorrida > 0 &&
        (agora - tempoInicioCorrida) >= tempoMinimoChegada &&
        contagemMarcas >= totalPontosBrancos) {
      SerialBT.println("CHEGADA VALIDADA - PARANDO");
      estadoAtual = PARADO;
    }
  }

  if (!brancoAgora) {
    sensorDirEmLinha = false;
  }
}

bool pareceInterseccao() {
  int ativosTotal = 0;
  int ativosCentro = 0;
  int soma = 0;

  for (int i = 0; i < 8; i++) {
    int raw = lerSensor(i);
    int n = normalizarLeitura(raw, minValues[i], maxValues[i]);

    if (n > 400) {
      ativosTotal++;
      if (i >= 2 && i <= 5) ativosCentro++;
    }

    soma += n;
  }

  return (ativosTotal >= 5 && ativosCentro >= 3 && soma >= 3200);
}


int contInterseccao = 0;

bool pareceInterseccaoFiltrada() {
  if (pareceInterseccao()) contInterseccao++;
  else contInterseccao = 0;

  return (contInterseccao >= 2);
}



// ================= PARSER BT =================
bool parseFloatCmd(const char* cmd, char prefixo, float* destino) {
  if (cmd[0] == prefixo && cmd[1] == '=') {
    *destino = atof(cmd + 2);
    return true;
  }
  return false;
}

bool parseIntCmd(const char* cmd, char prefixo, int* destino) {
  if (cmd[0] == prefixo && cmd[1] == '=') {
    *destino = atoi(cmd + 2);
    return true;
  }
  return false;
}

void executarComandoBT(const char* cmd) {
  float ftmp;
  int itmp;

  if (strcmp(cmd, "C") == 0) {
    estadoAtual = CALIBRANDO;
    SerialBT.println("OK CALIB");
    return;
  }

  if (strcmp(cmd, "S") == 0) {
    if (estadoAtual == AGUARD_LARG) {
      resetarControle();
      contagemDireita = 0;
      contagemMarcas = 0;
      sensorDirEmLinha = false;
      tempoTravaDir = 0;
      tempoInicioCorrida = millis();
      estadoAtual = CORRENDO;
      SerialBT.println("LARGADA!");
    } else if (estadoAtual == CORRENDO) {
      estadoAtual = PARADO;
      SerialBT.println("PARADO");
    }
    return;
  }

  if (strcmp(cmd, "L") == 0) {
    telemetriaAtiva = !telemetriaAtiva;
    SerialBT.println(telemetriaAtiva ? "TELE ON" : "TELE OFF");
    return;
  }

  if (strcmp(cmd, "?") == 0) {
    mostrarParametros();
    return;
  }

  if (strcmp(cmd, "RST") == 0) {
    resetarControle();
    SerialBT.println("PID RESET");
    return;
  }

  if (parseFloatCmd(cmd, 'P', &ftmp)) {
    Kp = ftmp;
    SerialBT.printf("OK P=%.3f\n", Kp);
    return;
  }

  if (parseFloatCmd(cmd, 'I', &ftmp)) {
    Ki = ftmp;
    SerialBT.printf("OK I=%.3f\n", Ki);
    return;
  }

  if (parseFloatCmd(cmd, 'D', &ftmp)) {
    Kd = ftmp;
    SerialBT.printf("OK D=%.3f\n", Kd);
    return;
  }

  if (parseIntCmd(cmd, 'V', &itmp)) {
    baseSpeed = constrain(itmp, 0, 255);
    SerialBT.printf("OK V=%d\n", baseSpeed);
    return;
  }

  if (parseIntCmd(cmd, 'M', &itmp)) {
    maxSpeed = constrain(itmp, 0, 255);
    SerialBT.printf("OK M=%d\n", maxSpeed);
    return;
  }

  SerialBT.print("CMD INVALIDO: ");
  SerialBT.println(cmd);
}

void checarBT() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (btIndex > 0) {
        btBuffer[btIndex] = '\0';

        for (uint8_t i = 0; i < btIndex; i++) {
          btBuffer[i] = toupper((unsigned char)btBuffer[i]);
        }

        executarComandoBT(btBuffer);

        btIndex = 0;
        btBuffer[0] = '\0';
      }
    } else {
      if (btIndex < sizeof(btBuffer) - 1) {
        btBuffer[btIndex++] = c;
      }
    }
  }
}

bool botaoBootPressionado() {
  if (digitalRead(botaoBoot) == LOW) {
    if (millis() - ultimoDebounceBoot > 300) {
      ultimoDebounceBoot = millis();
      return true;
    }
  }
  return false;
}

// ================= LOOP =================
void loop() {
  unsigned long agora = millis();

  checarBT();

  if (botaoBootPressionado()) {
    if (estadoAtual == AGUARD_CALIB || estadoAtual == PARADO || estadoAtual == AGUARD_LARG) {
      estadoAtual = CALIBRANDO;
    } else if (estadoAtual == CORRENDO) {
      estadoAtual = PARADO;
    }
  }

  switch (estadoAtual) {
    case CALIBRANDO:
      pararMotores();
      calibrarSensores();
      estadoAtual = AGUARD_LARG;
      break;

    case AGUARD_CALIB:
      pararMotores();
      break;

    case AGUARD_LARG:
      pararMotores();
      if (telemetriaAtiva && (agora - ultimoEnvioTelemetria >= 200)) {
        ultimoEnvioTelemetria = agora;
        enviarTelemetriaSensores();
      }
      break;

    case PARADO:
      pararMotores();
      telemetriaAtiva = false;
      break;

    case CORRENDO:
      {
        telemetriaAtiva = false;

        verificarVoltaSensorDireito();

        if (estadoAtual == PARADO) {
          pararMotores();
          break;
        }

        float e = erroLinha();
        float correcao = PID(e);
        correcao = constrain(correcao, -50, 50);

        int velEsq = (int)(baseSpeed - correcao);
        int velDir = (int)(baseSpeed + correcao);

        motores(velEsq, velDir);
        break;
      }
  }

  delay(3);
}
