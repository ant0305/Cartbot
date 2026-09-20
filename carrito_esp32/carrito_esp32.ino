/*
  carrito_esp32.ino
  Control del carrito por WiFi (modo AP) + comandos TCP + failsafe.

  Esquema de motores: L298N con las 4 entradas manejadas por PWM
  (ENA/ENB puenteados a HIGH en la placa). La direccion depende de
  cual entrada de cada motor recibe el PWM.

  Pines:
    LEFT_M0 = 13   LEFT_M1 = 12
    RIGHT_M0 = 14  RIGHT_M1 = 15

  Comandos TCP (1 caracter):
    F = adelante  B = atras  L = izquierda  R = derecha  S = detener

  API PWM: ledcAttach / ledcWrite (ESP32 core 3.x).
*/

#include <WiFi.h>

// ---------- WiFi (modo AP) ----------
const char* AP_SSID = "CarritoIA";
const char* AP_PASS = "carrito123";   // minimo 8 caracteres
const uint16_t PORT  = 8888;

WiFiServer server(PORT);
WiFiClient client;

// ---------- Pines L298N ----------
#define LEFT_M0   13
#define LEFT_M1   12
#define RIGHT_M0  14
#define RIGHT_M1  15

const int PWM_FREQ = 2000;   // Hz
const int PWM_RES  = 8;      // bits -> rango 0..255
const int VEL      = 110;    // velocidad al avanzar (subir si no arranca)
const int VEL_GIRO = 80;     // velocidad al girar (mas lento = mas control)

// ---------- Failsafe ----------
const unsigned long FAILSAFE_MS = 1000;   // sin comando 1s -> detener
unsigned long ultimoComando = 0;

// ---------------- Motores ----------------
void detener() {
  ledcWrite(RIGHT_M0, 0);   ledcWrite(RIGHT_M1, 0);
  ledcWrite(LEFT_M0,  0);   ledcWrite(LEFT_M1,  0);
}
void adelante() {
  ledcWrite(RIGHT_M0, 0);   ledcWrite(RIGHT_M1, VEL);
  ledcWrite(LEFT_M0,  0);   ledcWrite(LEFT_M1,  VEL);
}
void atras() {
  ledcWrite(RIGHT_M0, VEL); ledcWrite(RIGHT_M1, 0);
  ledcWrite(LEFT_M0,  VEL); ledcWrite(LEFT_M1,  0);
}
void derecha() {
  ledcWrite(RIGHT_M0, 0);        ledcWrite(RIGHT_M1, VEL_GIRO);
  ledcWrite(LEFT_M0,  VEL_GIRO); ledcWrite(LEFT_M1,  0);
}
void izquierda() {
  ledcWrite(RIGHT_M0, VEL_GIRO); ledcWrite(RIGHT_M1, 0);
  ledcWrite(LEFT_M0,  0);        ledcWrite(LEFT_M1,  VEL_GIRO);
}

void ejecutar(char c) {
  switch (c) {
    case 'F': adelante();  break;
    case 'B': atras();     break;
    case 'L': izquierda(); break;
    case 'R': derecha();   break;
    case 'S': detener();   break;
    default: return;       // desconocido: ignorar, no resetea failsafe
  }
  ultimoComando = millis();
}

void setupMotores() {
  ledcAttach(RIGHT_M0, PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_M1, PWM_FREQ, PWM_RES);
  ledcAttach(LEFT_M0,  PWM_FREQ, PWM_RES);
  ledcAttach(LEFT_M1,  PWM_FREQ, PWM_RES);
  detener();
}

void setup() {
  Serial.begin(115200);
  setupMotores();

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println();
  Serial.print("AP creado. SSID: ");        Serial.println(AP_SSID);
  Serial.print("IP del carrito: ");         Serial.println(WiFi.softAPIP()); // 192.168.4.1
  server.begin();
  Serial.print("Servidor TCP en puerto ");  Serial.println(PORT);
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    if (client) { Serial.println("PC conectada"); ultimoComando = millis(); }
  }
  while (client && client.available()) {
    ejecutar((char)client.read());
  }
  if (millis() - ultimoComando > FAILSAFE_MS) {
    detener();
  }
}