/*
RFID:
SDA: 10
SCK: 13
MOSI: 11
MISO: 12
GND: F-3 (breadboard)
RST: 5
3.3V: F-9 (breadboard)

LCD:
SCL: A5
SDA: A4
VCC: E-7 (breadboard)
GND: F-57 (breadboard)

HC-SR04:
VCC: C-7 (breadboard)
TRIG: 4
ECHO: 3
GND: H-57 (breadboard)

SERVO:
INCHIS dreapta: H-3 (breadboard) - GND
CENTRU: D-7 (breadboard) - VCC
DESCHIS stanga: 6 (PWM)

BUZZER:
+: 8
-: I-50 (breadboard)

BREADBOARD:
5V (POWER): + de la A
3.3V (POWER): + de la J-7
GND (POWER): - de la J-61
*/

#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>
#include <SPI.h>
#include <Servo.h>

#define HC_SR04_TRIG_PIN 4
#define HC_SR04_ECHO_PIN 3

#define RFID_RST_PIN 5
#define RFID_SS_PIN 10

#define SERVO_PIN 6
#define BUZZER_PIN 8
#define BUTTON_PIN 2

#define CARD_UID_BYTES 4

#define NUM_OF_DISTANCE_READS 100
#define LOCK_THRESHOLD 3

#define LOCK_DISTANCE 3  // centimeters
#define OPEN_DISTANCE 50  // centimeters

#define ALARM_MAX_FREQ 1500  // Hz
#define ALARM_MIN_FREQ 500
#define DEFAULT_ALARM_FREQ_MODIFIER 35

LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);
Servo servoLock;

String allowedCards[] = {
  "20654D20"
};

bool isLocked = true;
bool isAlarmActive = false;
int alarmFreq = ALARM_MIN_FREQ;
int alarmFreqModifier = DEFAULT_ALARM_FREQ_MODIFIER;

int servoPos = 0;

ISR(TIMER1_COMPB_vect) {
  if (isAlarmActive) {
    if (alarmFreq > ALARM_MAX_FREQ || alarmFreq < ALARM_MIN_FREQ) {
      alarmFreqModifier = -alarmFreqModifier;
    }

    alarmFreq += alarmFreqModifier;
    tone(BUZZER_PIN, alarmFreq);
  } else {
    alarmFreqModifier = DEFAULT_ALARM_FREQ_MODIFIER;
    alarmFreq = ALARM_MIN_FREQ;
    noTone(BUZZER_PIN);
  }
}

ISR(INT0_vect) {
  if (isAlarmActive && (PIND & (1 << PD2)) == 0) {
    isAlarmActive = false;
  }
}

void printLockedMessage() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SEIFFE: LOCKED");
  delay(10);

  lcd.setCursor(0, 1);
  lcd.print("CARD REQUIRED");
  delay(10);
}

void printUnlockedMessage() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SEIFFE: UNLOCKED");
  delay(10);

  lcd.setCursor(0, 1);
  lcd.print("DOOR IS OPEN");
  delay(10);
}

void printDeniedMessage() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SEIFFE: LOCKED");
  delay(10);

  lcd.setCursor(0, 1);
  lcd.print("DENIED CARD");
  delay(10);
}

long getDistance() {
  double average_duration = 0;
  for (int i = 0; i < NUM_OF_DISTANCE_READS; ++i) {
    // Clear the HC_SR04_TRIG_PIN.
    // digitalWrite(HC_SR04_TRIG_PIN, LOW);
    PORTD &= ~(1 << PD4);
    delayMicroseconds(2);

    // Set the HC_SR04_TRIG_PIN on HIGH state for 10 micro seconds.
    // digitalWrite(HC_SR04_TRIG_PIN, HIGH);
    PORTD |= (1 << PD4);
    delayMicroseconds(10);
    // digitalWrite(HC_SR04_TRIG_PIN, LOW);
    PORTD &= ~(1 << PD4);

    // Read the HC_SR04_ECHO_PIN, returns the sound wave travel time in microseconds.
    double duration = pulseIn(HC_SR04_ECHO_PIN, HIGH);
    average_duration += duration;
  }

  average_duration = average_duration / NUM_OF_DISTANCE_READS;

  // Calculate the distance (in cm).
  long distance = average_duration * 0.034 / 2;

  return distance;
}

String readCardUid() {
  // Reset the loop if no new card present on the sensor/reader.
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return "";
  }

  // Select the card.
  if (!mfrc522.PICC_ReadCardSerial()) {
    return "";
  }

  String cardUid = "";

  // Four-byte UIDs.
  for (size_t i = 0; i < CARD_UID_BYTES; ++i) {
    String uidChunk = String(mfrc522.uid.uidByte[i], HEX);
    uidChunk.toUpperCase();

    cardUid += uidChunk;
  }

  mfrc522.PICC_HaltA();
  return cardUid;
}

void lockSeiffe() {
  servoLock.write(90);
  disableAlarm();
  isLocked = true;
}

void unlockSeiffe() {
  servoLock.write(0);
  enableAlarm();
  isLocked = false;
}

void configureAlarmTimer() {
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1B = 31249;
  TCCR1B |= (1 << WGM12);               // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 prescaler
}

void enableAlarmTimer() {
  // Enable timer compare interrupt.
  TIMSK1 |= (1 << OCIE1B);
}

void disableAlarmTimer() {
  // Disable timer compare interrupt.
  TIMSK1 &= ~(1 << OCIE1B);
}

void enableAlarm() {
  isAlarmActive = true;
  enableSilencer();
}

void disableAlarm() {
  isAlarmActive = false;
  disableSilencer();
}

void configureSilencer() {
  // Set INT0 to trigger on the rising edge.
  EICRA |= (1 << ISC01) | (1 << ISC00);
}

void enableSilencer() {
  // Turns on INT0.
  EIMSK |= (1 << INT0);
}

void disableSilencer() {
  // Turns off INT0.
  EIMSK &= ~(1 << INT0);
}

void setup() {
  // Deactivate global interruptions.
  cli();

  configureAlarmTimer();
  enableAlarmTimer();

  configureSilencer();

  // Activate global interruptions.
  sei();

  lcd.init();
  lcd.clear();
  lcd.backlight();

  // pinMode(HC_SR04_TRIG_PIN, OUTPUT);
  DDRD |= (1 << PD4);

  // pinMode(HC_SR04_ECHO_PIN, INPUT);
  DDRD |= (0 << PD3);
  PORTD &= ~(1 << PD3);

  // pinMode(BUZZER_PIN, OUTPUT);
  DDRB |= (1 << PB0);

  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  DDRD &= ~(1 << PD2);
  PORTD |= (1 << PD2);

  SPI.begin();
  mfrc522.PCD_Init();
  delay(4);

  servoLock.attach(SERVO_PIN);

  // Move the servo to 90 degrees - locked position.
  servoLock.write(90);

  printLockedMessage();
}

void loop() {
  long distance = getDistance();
  if (isLocked && distance < LOCK_DISTANCE) {
    // Seiffe is locked.
    String cardUid = readCardUid();
    if (cardUid == "") {
      return;
    }

    size_t allowedCardsLen = sizeof(allowedCards) / sizeof(allowedCards[0]);
    for (size_t i = 0; i < allowedCardsLen; ++i) {
      if (cardUid == allowedCards[i]) {
        printUnlockedMessage();
        unlockSeiffe();
        return;
      }
    }

    // It is not an allowed card.
    printDeniedMessage();
    delay(3000);
    printLockedMessage();
  } else if (!isLocked) {
    // Seiffe is unlocked.
    bool canLock = false;
    while (true) {
      long distance = getDistance();
      if (!canLock && distance > OPEN_DISTANCE) {
        // The door has been opened enough, it can be locked when necessary.
        canLock = true;
      } else if (canLock && distance < LOCK_DISTANCE) {
        // The door has to be closed for LOCK_THRESHOLD distance processing iterations to get locked.
        int lock_loader = 0;
        while (lock_loader < LOCK_THRESHOLD) {
          long currentDistance = getDistance();
          if (currentDistance < LOCK_DISTANCE) {
            ++lock_loader;
          } else {
            lock_loader = 0;
          }
        }

        // The door is now closed, time to lock it.
        printLockedMessage();
        lockSeiffe();
        break;
      }
    }
  }
}