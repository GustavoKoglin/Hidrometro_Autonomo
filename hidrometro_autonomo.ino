#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <RTClib.h>

#define FLOW_SENSOR_PIN 2
#define LED_PIN 12
#define COST_PER_CUBIC_METER 2.00
#define SWITCH_PIN 10

LiquidCrystal_PCF8574 lcd(0x27);  // Endereço do módulo LCD I2C
RTC_DS1307 rtc;                   // Objeto RTC para o módulo Real Time Clock

volatile int pulseCount = 0;  // Contador de pulsos do sensor de fluxo de água
float flowRate = 0.0;         // Taxa de fluxo de água
float flowCubicMeters = 0.0;  // Fluxo em metros cúbicos
unsigned long oldTime = 0;    // Tempo da última leitura do sensor

bool systemOn = false;  // Indica se o sistema está ligado ou desligado

void pulseCounter() {
  pulseCount++;  // Incrementa o contador de pulsos quando ocorre uma interrupção
}

void setup() {
  pinMode(FLOW_SENSOR_PIN, INPUT);    // Define o pino do sensor de fluxo como entrada
  pinMode(LED_PIN, OUTPUT);           // Define o pino do LED como saída
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Define o pino da chave gangorra como entrada com pull-up interno

  lcd.begin(16, 2);        // Inicializa o LCD com 16 colunas e 2 linhas
  lcd.setBacklight(HIGH);  // Define o brilho do LCD como alto

  Wire.begin();  // Inicializa a comunicação I2C
  rtc.begin();   // Inicializa o RTC

  // Ajusta a data e hora do RTC com a data e hora do upload do código
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  lcd.clear();  // Limpa o LCD
}

void displayDateTime() {
  DateTime now = rtc.now();
  lcd.setCursor(0, 0);
  if (now.day() < 10) {
    lcd.print("0");
  }
  lcd.print(now.day(), DEC);
  lcd.print("/");
  if (now.month() < 10) {
    lcd.print("0");
  }
  lcd.print(now.month(), DEC);
  lcd.print("/");
  lcd.print(now.year(), DEC);
  lcd.setCursor(11, 0);
  if (now.hour() < 10) {
    lcd.print("0");
  }
  lcd.print(now.hour(), DEC);
  lcd.print(":");
  if (now.minute() < 10) {
    lcd.print("0");
  }
  lcd.print(now.minute(), DEC);
}

void displayFlowInfo() {
  lcd.setCursor(0, 1);
  lcd.print(flowCubicMeters, 2);
  lcd.setCursor(4, 1);
  lcd.print("m3");
  lcd.setCursor(8, 1);
  lcd.print("R$");
  lcd.print(flowCubicMeters * COST_PER_CUBIC_METER, 2);
}

void loop() {
  // Verifica o estado da chave liga/desliga
  if (digitalRead(SWITCH_PIN) == LOW) {
    if (!systemOn) {
      systemOn = true;              // Liga o sistema
      digitalWrite(LED_PIN, HIGH);  // Acende o LED
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("System: Online");
      delay(5000);  // Aguarda 5 segundos antes de limpar o visor

      lcd.clear();
      displayDateTime();
      displayFlowInfo();
    } else {
      lcd.clear();
      displayDateTime();
      displayFlowInfo();
    }

    // Faz a leitura do sensor de fluxo de água a cada 20 segundos
    unsigned long currentTime = millis();
    if (currentTime - oldTime >= 20000) {
      detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));  // Desabilita a interrupção para evitar conflitos

      flowRate = pulseCount / 7.5;          // Calcula a taxa de fluxo em litros por minuto
      flowCubicMeters = flowRate / 1000.0;  // Converte para metros cúbicos por minuto

      pulseCount = 0;                                                                  // Reinicia o contador de pulsos
      attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);  // Reabilita a interrupção
      oldTime = currentTime;                                                           // Atualiza o tempo da última leitura

      displayFlowInfo();

      delay(1000);  // Aguarda 1 segundo antes de fazer a próxima leitura
    }

    delay(1000);  // Aguarda 1 segundo antes de atualizar a data e hora
  } else {
    if (systemOn) {
      systemOn = false;            // Desliga o sistema
      digitalWrite(LED_PIN, LOW);  // Apaga o LED
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System: Offline");
    lcd.setCursor(0, 1);
    lcd.print("");
  }

  delay(1000);  // Aguarda 1 segundo antes de verificar novamente
}

// Melhorias e Ajustes
