#include <Arduino.h>
#include <ArduinoJson.h>

// =====================================================================
// DEFINICIÓN DE PINES DE HARDWARE
// =====================================================================

// Pines de dirección del motor izquierdo (H-Bridge)
#define MOTOR_L_IN1 9
#define MOTOR_L_IN2 8

// Pines de dirección del motor derecho (H-Bridge)
#define MOTOR_R_IN1 11
#define MOTOR_R_IN2 10

// Pin de habilitación global del driver de motores (EEP = Enable)
#define MOTOR_EEP 12

// Pines de los encoders cuadráticos (canal A y B por rueda)
#define ENC_L_A 4
#define ENC_L_B 5
#define ENC_R_A 6
#define ENC_R_B 7

// Configuración del PWM por hardware del ESP32
#define PWM_FREQ 20000  // 20 kHz: por encima del rango audible, evita silbido en motores
#define PWM_RES  8      // Resolución de 8 bits → valores entre 0 y 255

// =====================================================================
// PARÁMETROS FÍSICOS DEL ROBOT
// =====================================================================

const float R = 0.04f;        // Radio de la rueda en metros (40 mm)
const float L = 0.20f;        // Separación entre ruedas (track width) en metros (200 mm)
const float CPR_X4 = 3840.0f; // Counts por revolución en modo X4 (4 flancos × 960 líneas)

// =====================================================================
// VARIABLES GLOBALES
// =====================================================================

// Contadores de encoder (volatile porque se modifican en ISR)
volatile int32_t countL = 0;
volatile int32_t countR = 0;

// Estado anterior de cada encoder para la decodificación cuadrática
uint8_t prevL = 0, prevR = 0;

// Tabla de lookup para decodificación cuadrática X4
// Índice: (estado_anterior << 2) | estado_actual → 4 bits
// Valores: +1 (avance), -1 (retroceso), 0 (transición inválida o sin cambio)
static const int8_t QDEC_LUT[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Pose acumulada del robot (odometría)
float odom_x = 0.0f, odom_y = 0.0f, odom_th = 0.0f;

// Velocidades comandadas desde el dashboard (cinemática inversa)
float target_v = 0.0f; // Velocidad lineal deseada en m/s
float target_w = 0.0f; // Velocidad angular deseada en rad/s

// RPM objetivo calculados para cada rueda
float target_rpm_L = 0.0f, target_rpm_R = 0.0f;

// RPM actuales medidos desde los encoders
float current_rpm_L = 0.0f, current_rpm_R = 0.0f;

// =====================================================================
// VARIABLES DEL CONTROLADOR PID (×2, uno por rueda)
// =====================================================================

float kp = 2.0f, ki = 0.5f, kd = 0.1f; // Ganancias PID — requieren ajuste (tuning)

// Integral del error y error anterior — rueda izquierda
float err_sum_L = 0, last_err_L = 0;

// Integral del error y error anterior — rueda derecha
float err_sum_R = 0, last_err_R = 0;

// Señal de control PWM acumulada (PID incremental: pwm += ΔpwmPID)
int pwm_L = 0, pwm_R = 0;

// =====================================================================
// TEMPORIZACIÓN DEL LOOP DE CONTROL
// =====================================================================

uint32_t last_time = 0;
const uint32_t LOOP_DT_MS = 50; // Período del loop: 50 ms → frecuencia de control 20 Hz

// =====================================================================
// RUTINAS DE INTERRUPCIÓN (ISR) — Decodificación cuadrática
// =====================================================================

// ISR del encoder izquierdo: se ejecuta en cada flanco de ENC_L_A o ENC_L_B
void IRAM_ATTR isrL() {
  // Lee el estado actual de ambos canales y forma un nibble de 2 bits
  uint8_t curr = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
  // Concatena estado anterior y actual (4 bits) → índice en QDEC_LUT
  countL += QDEC_LUT[(prevL << 2) | curr];
  prevL = curr; // Guarda estado para la siguiente interrupción
}

// ISR del encoder derecho: idéntico al izquierdo
void IRAM_ATTR isrR() {
  uint8_t curr = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
  countR += QDEC_LUT[(prevR << 2) | curr];
  prevR = curr;
}

// =====================================================================
// CONTROL DE MOTORES
// =====================================================================

// Aplica señales PWM a ambos motores vía H-Bridge
// Convención: positivo = adelante, negativo = atrás
void setMotorPWM(int pwm_l, int pwm_r) {
  // Limita el PWM al rango válido de 8 bits con signo
  pwm_l = constrain(pwm_l, -255, 255);
  pwm_r = constrain(pwm_r, -255, 255);

  // Motor izquierdo: giro normal
  // IN1 activo → adelante | IN2 activo → atrás
  ledcWrite(MOTOR_L_IN1, pwm_l > 0 ? pwm_l : 0);
  ledcWrite(MOTOR_L_IN2, pwm_l < 0 ? -pwm_l : 0);

  // Motor derecho: lógica invertida por espejado físico en el chasis
  // IN1 activo → atrás | IN2 activo → adelante
  ledcWrite(MOTOR_R_IN1, pwm_r < 0 ? -pwm_r : 0);
  ledcWrite(MOTOR_R_IN2, pwm_r > 0 ? pwm_r : 0);
}

// =====================================================================
// SETUP
// =====================================================================

void setup() {
  Serial.begin(115200); // Comunicación serial con el dashboard a 115200 baud

  // Inicializa los canales PWM por hardware del ESP32 (ledc)
  ledcAttach(MOTOR_L_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_L_IN2, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_R_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_R_IN2, PWM_FREQ, PWM_RES);

  // Activa el driver de motores (pin EEP en HIGH = habilitado)
  pinMode(MOTOR_EEP, OUTPUT);
  digitalWrite(MOTOR_EEP, HIGH);

  // Configura pines de encoder con pull-up interno
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  // Adjunta interrupciones en CHANGE (flanco de subida Y de bajada) → modo X4
  attachInterrupt(ENC_L_A, isrL, CHANGE); attachInterrupt(ENC_L_B, isrL, CHANGE);
  attachInterrupt(ENC_R_A, isrR, CHANGE); attachInterrupt(ENC_R_B, isrR, CHANGE);
}

// =====================================================================
// LOOP PRINCIPAL
// =====================================================================

void loop() {

  // ------------------------------------------------------------------
  // 1. RECEPCIÓN Y PARSEO DE COMANDOS JSON (no bloqueante)
  // ------------------------------------------------------------------
  if (Serial.available()) {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, Serial);

    if (!err && doc.containsKey("v") && doc.containsKey("w")) {
      target_v = doc["v"]; // Velocidad lineal del robot [m/s]
      target_w = doc["w"]; // Velocidad angular del robot [rad/s]

      // --- Cinemática Inversa: velocidades del robot → velocidades de rueda ---
      // Modelo diferencial: rueda exterior suma contribución angular, interior la resta
      float v_l = target_v - (target_w * L / 2.0f); // Velocidad tangencial rueda izq [m/s]
      float v_r = target_v + (target_w * L / 2.0f); // Velocidad tangencial rueda der [m/s]

      // Convierte de m/s a RPM: rpm = (v / circunferencia) × 60
      target_rpm_L = (v_l / (2.0f * PI * R)) * 60.0f;
      target_rpm_R = (v_r / (2.0f * PI * R)) * 60.0f;
    }
  }

  // ------------------------------------------------------------------
  // 2. LOOP DE CONTROL A 20 Hz
  // ------------------------------------------------------------------
  uint32_t now = millis();
  if (now - last_time >= LOOP_DT_MS) {
    float dt = (now - last_time) / 1000.0f; // Delta tiempo en segundos
    last_time = now;

    // --- Lectura atómica de contadores de encoder ---
    // Se deshabilitan interrupciones para evitar condición de carrera
    int32_t cL, cR;
    noInterrupts();
    cL = countL; countL = 0; // Lee y resetea acumulador izquierdo
    cR = countR; countR = 0; // Lee y resetea acumulador derecho
    interrupts();

    // --- Cálculo de RPM actuales ---
    // (counts / dt) = counts/s → / CPR_X4 = rev/s → × 60 = RPM
    current_rpm_L = ((cL / dt) / CPR_X4) * 60.0f;
    current_rpm_R = ((cR / dt) / CPR_X4) * 60.0f;

    // --- Cinemática Directa: RPM medidos → pose del robot (Odometría) ---
    // Convierte RPM de cada rueda a velocidad tangencial en m/s
    float v_l_meas = (current_rpm_L / 60.0f) * (2.0f * PI * R);
    float v_r_meas = (current_rpm_R / 60.0f) * (2.0f * PI * R);

    // Velocidad lineal del centro del robot y velocidad angular
    float v_meas = (v_r_meas + v_l_meas) / 2.0f; // Promedio de ruedas = vel. lineal
    float w_meas = (v_r_meas - v_l_meas) / L;     // Diferencia / base = vel. angular

    // Integración de Euler: actualiza la pose acumulada
    odom_th += w_meas * dt;                    // Actualiza ángulo primero
    odom_x  += v_meas * cos(odom_th) * dt;    // Proyecta desplazamiento en X
    odom_y  += v_meas * sin(odom_th) * dt;    // Proyecta desplazamiento en Y

    // --- PID Rueda Izquierda ---
    float err_L   = target_rpm_L - current_rpm_L;  // Error proporcional
    err_sum_L    += err_L * dt;                     // Acumulación integral (× dt para discreta)
    float dErr_L  = (err_L - last_err_L) / dt;     // Derivada del error
    pwm_L        += (kp * err_L) + (ki * err_sum_L) + (kd * dErr_L); // PID incremental
    last_err_L    = err_L;

    // --- PID Rueda Derecha ---
    float err_R   = target_rpm_R - current_rpm_R;
    err_sum_R    += err_R * dt;
    float dErr_R  = (err_R - last_err_R) / dt;
    pwm_R        += (kp * err_R) + (ki * err_sum_R) + (kd * dErr_R);
    last_err_R    = err_R;

    // --- Seguridad en target cero + Anti-windup por reset ---
    // Si el target es 0 RPM, fuerza PWM a 0 y limpia el integrador
    // Evita que el integrador acumule error mientras el robot está frenado
    if (target_rpm_L == 0) { pwm_L = 0; err_sum_L = 0; }
    if (target_rpm_R == 0) { pwm_R = 0; err_sum_R = 0; }

    // Aplica la señal de control a los motores
    setMotorPWM(pwm_L, pwm_R);

    // ------------------------------------------------------------------
    // 3. SERIALIZACIÓN DE TELEMETRÍA → Dashboard
    // ------------------------------------------------------------------
    StaticJsonDocument<256> out;
    out["rpmL"] = current_rpm_L; // RPM medido rueda izquierda
    out["rpmR"] = current_rpm_R; // RPM medido rueda derecha
    out["x"]    = odom_x;        // Posición X estimada [m]
    out["y"]    = odom_y;        // Posición Y estimada [m]
    out["th"]   = odom_th;       // Ángulo estimado [rad]
    serializeJson(out, Serial);
    Serial.println(); // Newline como delimitador de frame JSON
  }
}
