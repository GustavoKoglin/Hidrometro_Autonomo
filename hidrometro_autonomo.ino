#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <RTClib.h>
#include <EEPROM.h>

// Definição de pinos
#define FLOW_SENSOR_PIN 2       // Pino do sensor de fluxo (interrupção 0)
#define LED_PIN 12              // Pino do LED de status
#define SOLENOID_PIN 11         // Pino da válvula solenoide
#define SWITCH_PIN 10           // Pino da chave liga/desliga física
#define BUZZER_PIN 9            // Pino para buzzer de alerta
#define TAMPER_PIN 8            // Pino para sensor de violação (reed switch)
#define MAGNET_PIN 7            // Pino para detector de ímã (Hall effect)

// Constantes do sistema
const float COST_PER_CUBIC_METER = 2.00;
const float FLOW_CALIBRATION_FACTOR = 7.5;
const unsigned long MEASUREMENT_INTERVAL = 20000; // 20s
const float LEAK_THRESHOLD = 0.005; // 0.005 m³/min = 5L/min
const unsigned long TAMPER_TIMEOUT = 300000; // 5 minutos para reinício após violação
const String AUTHORIZATION_CODE = "A1B2C3"; // Código de autorização

// Endereços EEPROM
const int TOTAL_CONS_ADDR = 0;  // 4 bytes (float)
const int TAMPER_FLAG_ADDR = 4; // 1 byte (bool)

// Objetos de hardware
LiquidCrystal_PCF8574 lcd(0x27);
RTC_DS1307 rtc;

// Variáveis do sistema
volatile int pulseCount = 0;
float flowRate = 0.0;
float totalConsumption = 0.0;
float currentCost = 0.0;
unsigned long lastMeasurement = 0;
bool systemOn = false;
bool leakDetected = false;
bool tamperDetected = false;
unsigned long tamperTime = 0;
String inputCode = "";
bool adminMode = false;

void pulseCounter() {
  pulseCount++;
}

void setup() {
  // Inicialização dos pinos
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TAMPER_PIN, INPUT_PULLUP);
  pinMode(MAGNET_PIN, INPUT);
  
  digitalWrite(SOLENOID_PIN, HIGH); // Válvula fechada
  digitalWrite(LED_PIN, LOW);

  // Inicialização do LCD
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.clear();
  lcd.print("Inicializando...");

  // Inicialização do RTC
  Wire.begin();
  if (!rtc.begin()) {
    showError("Erro no RTC!");
  }

  // Carrega consumo total da EEPROM
  EEPROM.get(TOTAL_CONS_ADDR, totalConsumption);
  
  // Verifica se houve violação anterior
  tamperDetected = EEPROM.read(TAMPER_FLAG_ADDR);
  if (tamperDetected) {
    handleTamper();
  }

  // Configura interrupção para o sensor de fluxo
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // Configuração inicial do RTC (descomentar apenas na primeira execução)
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  delay(2000);
  checkSecurity();
  lcd.clear();
}

void checkSecurity() {
  // Verifica violação física
  if (digitalRead(TAMPER_PIN) {
    triggerTamper("Violacao fisica!");
  }
  
  // Verifica presença de ímã (tentativa de fraude)
  if (!digitalRead(MAGNET_PIN)) {
    triggerTamper("Imã detectado!");
  }
}

void triggerTamper(const char* reason) {
  tamperDetected = true;
  EEPROM.write(TAMPER_FLAG_ADDR, true);
  
  digitalWrite(SOLENOID_PIN, HIGH); // Fecha válvula
  digitalWrite(BUZZER_PIN, HIGH);
  
  lcd.clear();
  lcd.print("VIOLACAO!");
  lcd.setCursor(0, 1);
  lcd.print(reason);
  
  tamperTime = millis();
  while (millis() - tamperTime < TAMPER_TIMEOUT) {
    if (checkAuthorization()) {
      resetTamperState();
      return;
    }
    delay(1000);
  }
  
  // Bloqueio permanente até autorização
  while (true) {
    if (checkAuthorization()) {
      resetTamperState();
      return;
    }
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN)); // Buzzer piscando
    delay(500);
  }
}

bool checkAuthorization() {
  // Simulação de entrada de código (na prática, usar comunicação serial ou teclado)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputCode == AUTHORIZATION_CODE) {
        inputCode = "";
        return true;
      }
      inputCode = "";
    } else {
      inputCode += c;
    }
  }
  return false;
}

void resetTamperState() {
  tamperDetected = false;
  EEPROM.write(TAMPER_FLAG_ADDR, false);
  digitalWrite(BUZZER_PIN, LOW);
  lcd.clear();
}

void handleTamper() {
  lcd.clear();
  lcd.print("Sistema bloqueado");
  lcd.setCursor(0, 1);
  lcd.print("Codigo requerido");
  
  while (!checkAuthorization()) {
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    delay(500);
  }
  
  resetTamperState();
}

void showError(const char* message) {
  lcd.clear();
  lcd.print(message);
  while (true) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);
  }
}

void displaySystemStatus() {
  DateTime now = rtc.now();
  
  // Linha superior: Data/Hora e Status
  lcd.setCursor(0, 0);
  lcd.print(now.day() < 10 ? "0" : ""); lcd.print(now.day());
  lcd.print("/");
  lcd.print(now.month() < 10 ? "0" : ""); lcd.print(now.month());
  lcd.print(" ");
  lcd.print(now.hour() < 10 ? "0" : ""); lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute() < 10 ? "0" : ""); lcd.print(now.minute());
  
  lcd.print(" ");
  lcd.print(systemOn ? "ON" : "OFF");

  // Linha inferior: Dados de consumo
  lcd.setCursor(0, 1);
  if (systemOn || adminMode) {
    lcd.print(flowRate, 3); lcd.print("m3/m ");
    lcd.print("R$"); lcd.print(currentCost, 2);
    
    if (leakDetected) {
      lcd.setCursor(14, 1);
      lcd.print("L!");
    }
  } else {
    lcd.print("Total: ");
    lcd.print(totalConsumption, 2); lcd.print("m3");
    lcd.print(" R$"); lcd.print(totalConsumption * COST_PER_CUBIC_METER, 1);
  }
}

void updateFlowMeasurement() {
  detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

  flowRate = (pulseCount / FLOW_CALIBRATION_FACTOR) / 1000.0;
  totalConsumption += flowRate * (MEASUREMENT_INTERVAL / 60000.0);
  currentCost = totalConsumption * COST_PER_CUBIC_METER;
  
  // Salva na EEPROM a cada 1m³ para evitar desgaste
  static float lastSaved = 0;
  if (totalConsumption - lastSaved >= 1.0) {
    EEPROM.put(TOTAL_CONS_ADDR, totalConsumption);
    lastSaved = totalConsumption;
  }

  pulseCount = 0;
  lastMeasurement = millis();

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
}

void checkForLeaks() {
  if (flowRate > LEAK_THRESHOLD && !systemOn) {
    leakDetected = true;
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(SOLENOID_PIN, LOW); // Abre válvula em emergência
    lcd.clear();
    lcd.print("VAZAMENTO DETECT!");
    lcd.setCursor(0, 1);
    lcd.print("Taxa: ");
    lcd.print(flowRate, 3);
    lcd.print(" m3/m");
    delay(5000);
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    leakDetected = false;
  }
}

void loop() {
  checkSecurity();
  
  // Verificação do botão físico (com debounce)
  static unsigned long lastDebounceTime = 0;
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(SWITCH_PIN);
  
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > 50) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      if (!systemOn && !tamperDetected) {
        systemOn = true;
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(SOLENOID_PIN, LOW);
        lcd.clear();
        lcd.print("Sistema ONLINE");
        delay(2000);
      } else if (systemOn) {
        systemOn = false;
        digitalWrite(LED_PIN, LOW);
        digitalWrite(SOLENOID_PIN, HIGH);
        lcd.clear();
        lcd.print("Sistema OFFLINE");
        delay(2000);
      }
    }
    lastButtonState = buttonState;
  }

  // Atualiza medições se o sistema estiver ligado
  if (systemOn && (millis() - lastMeasurement >= MEASUREMENT_INTERVAL)) {
    updateFlowMeasurement();
    checkForLeaks();
  }

  // Atualiza display a cada segundo
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {
    displaySystemStatus();
    lastDisplayUpdate = millis();
  }

  // Verifica comunicação serial para modo admin
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "ADMIN") {
      adminMode = true;
      lcd.clear();
      lcd.print("Modo Administrador");
      delay(2000);
    } else if (command == "RESET") {
      totalConsumption = 0;
      EEPROM.put(TOTAL_CONS_ADDR, 0.0);
      lcd.clear();
      lcd.print("Consumo zerado");
      delay(2000);
    }
  }
}
