//#include <SoftwareSerial.h>
#include "Nextion.h"
#include "max6675.h"

// Definiciones de pines y constantes
#define ENCODER_PIN 13  // Pin de interrupción para el encoder
#define PWM_PIN 27      // Pin PWM para controlar el motor
#define BUTTON_UP_PIN 4  // Botón para aumentar setpoint
#define BUTTON_DOWN_PIN 5  // Botón para reducir setpoint

// Variables globales para control de RPM
const int in1 = 12;           // Entrada 1 del puente H (dirección).
const int in2 = 14;           // Entrada 2 del puente H (dirección).
volatile unsigned long tiempoAnterior = 0;
volatile int frecuencia = 0; // Número de pulsos leídos
double RPMs = 0;
double RPMsFilter = RPMs;
double RPMsFilterAnt = 0;
double alpha = 0.2; // Filtro de media exponencial
int nRanuras = 44;  // Número de ranuras del encoder
double periodo = 10000; // Tiempo de lectura del encoder (ms)
unsigned long tiempoAhora = 0;  // Tiempo actual
float SetP = 6.3; // RPM deseada (Setpoint)
float kp =1, ki = 0.5, kd = 0.05; // Parámetros del PID
static volatile unsigned long lastTimeDebounce = 0;

// Variables PID
float error = 0, errorAnt = 0, errorAntAnt = 0;
float cv = 255, cvAnt = 255, Ts = 10; // Variable de control
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float elapsedTime, timePrev=0;


// Variables de temperatura
const int rele1 = 0; 
const int rele2 = 2;  
const int rele3 = 15;  

// Primer Termopar
int thermoDO_1 = 23;
int thermoCS_1 = 22;
int thermoCLK_1 = 21;
int temp1 = 0;

// Segundo Termopar
int thermoDO_2 = 19;
int thermoCS_2 = 18;
int thermoCLK_2 = 5;
int temp2 = 0;

// Tercer Termopar
int thermoDO_3 = 17;
int thermoCS_3 = 16;
int thermoCLK_3 = 4;
int temp3 = 0;


// Funciones para controlar motor
void setup() {
  Serial.begin(9600);
  
     // Motor PWM
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);  // Encoder pin
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);  // Botón para aumentar Setpoint
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);  // Botón para reducir Setpoint

  // Configuración de la interrupción del encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), interrupcion0, RISING);

  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(rele3, OUTPUT);
  
  digitalWrite(rele1, HIGH);
  digitalWrite(rele2, HIGH);
  digitalWrite(rele3, HIGH);

  // Crear las tareas FreeRTOS
    xTaskCreatePinnedToCore(
    controlMotorTask,   // Función de la tarea
    "ControlMotor",     // Nombre de la tarea
    2048,               // Tamaño del stack (ajustable)
    NULL,               // Parámetros de la tarea
    1,                  // Prioridad de la tarea
    NULL,               // Handle de la tarea
    0                   // Núcleo donde ejecutar (0 o 1)
  );

    xTaskCreatePinnedToCore(
    lecturaTemperaturasTask, // Función de la tarea para leer las temperaturas
    "LecturaTemperaturas",   // Nombre de la tarea
    4096,                    // Tamaño del stack
    NULL,                    // Parámetros de la tarea
    1,                       // Prioridad
    NULL,                    // Handle de la tarea
    0                        // Núcleo donde ejecutar (0 o 1)
  );
}

void loop() {
  // El loop no hace nada, todo se maneja en las tareas FreeRTOS
}

// Interrupción para contar pulsos del encoder
void interrupcion0() {
   // Almacenar el tiempo anterior antes de la lectura actual
    //elapsedTime = (millis() - timePrev) / 1000.0; 
  
  static bool lastState = LOW; // Almacena el último estado del pin

  bool currentState = digitalRead(ENCODER_PIN);
  if (currentState == HIGH && lastState == LOW && (millis() - lastTimeDebounce >= 100)) {
    lastTimeDebounce = millis();
    frecuencia++;
    //Serial.print(frecuencia);
    //Serial.print(",");
  }
  lastState = currentState; // Actualizar el estado previo
  //timePrev=millis();
}

// Función PID para calcular el valor de control (CV)
float funcionPID() {
  error = SetP - RPMsFilterAnt;  // Error actual
   if (abs(error) <= 0.2) {
    error = 0;
    //integral = 0; // Evita acumulación innecesaria
  }
  //pid_p =  kp*error;
  //pid_i += error;
  //pid_d = error-errorAnt;

  //cv = pid_p + pid_i*ki + pid_d*kd;

  cv = cvAnt + (kp + (kd / Ts)) * error + (-kp + ki * Ts - (2 * kd / Ts)) * errorAnt + (kd / Ts) * errorAntAnt;
   // Limita la salida PWM entre 0 y 255
    cv = constrain(cv, 100, 255);

  // Actualizar variables de estado
  cvAnt = cv;
  errorAntAnt = errorAnt;
  errorAnt = error;
  RPMsFilterAnt=RPMsFilter;
  return cv; // Valor de control para PWM
}

// Tarea FreeRTOS para el cálculo de RPM y control PID
void controlMotorTask(void *pvParameters) {
  while (true) {
    if (millis() - tiempoAnterior >= periodo) {
      RPMs = (frecuencia * 60000.0) / ((millis() - tiempoAnterior) * nRanuras); // Calcular RPM
      RPMsFilter = RPMs;
      //RPMsFilter = alpha * RPMs + (1.0 - alpha) * RPMsFilter; // Aplicar filtro exponencial
      tiempoAnterior = millis();
      frecuencia = 0; // Reiniciar contador de pulsos

      // Calcular PID y ajustar el PWM
      int valorPID = funcionPID();
      digitalWrite(in2, HIGH);
      digitalWrite(in1, LOW);
      analogWrite(PWM_PIN, valorPID);
      //int(valorPID)

      // Imprimir valores para monitoreo
      Serial.print("Error: "); 
      Serial.print(error);      // Variable de control CV
      
      Serial.print(" | RPM: ");
      Serial.print(RPMsFilter);  // RPM filtrada

      Serial.print(" | PWM: ");
      Serial.println(valorPID);  // RPM filtrada
    }
/*
    // Ajuste de Setpoint con botones
    if (digitalRead(BUTTON_UP_PIN) == LOW) {
      SetP += 10;  // Incrementar setpoint
      vTaskDelay(500);  // Debounce
    }
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
      SetP -= 10;  // Reducir setpoint
      vTaskDelay(500);  // Debounce
    }*/

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Da tiempo a otras tareas para ejecutarse
  }
}

// Tarea FreeRTOS para leer temperaturas
void lecturaTemperaturasTask(void *pvParameters) {
  while (true) {
    MAX6675 thermocouple1(thermoCLK_1, thermoCS_1, thermoDO_1);
    MAX6675 thermocouple2(thermoCLK_2, thermoCS_2, thermoDO_2);
    MAX6675 thermocouple3(thermoCLK_3, thermoCS_3, thermoDO_3);

    temp1 = thermocouple1.readCelsius();
    temp2 = thermocouple2.readCelsius();
    temp3 = thermocouple3.readCelsius();

    // Activar relés (para encender los controladores PID)
    digitalWrite(rele1, LOW);
    digitalWrite(rele2, LOW);
    digitalWrite(rele3, LOW);

    // Imprimir las temperaturas para monitoreo
    Serial.print("Temp1 = ");
    Serial.print(temp1);
    Serial.print(" | Temp2 = ");
    Serial.print(thermocouple2.readCelsius());
    Serial.print(" | Temp3 = ");
    Serial.println(thermocouple3.readCelsius());
    //delay(500);
    vTaskDelay(3000 / portTICK_PERIOD_MS); // Leer temperaturas cada 3 segundos
  }
}
