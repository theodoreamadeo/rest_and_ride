#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 9
#define RST_PIN 10

// Motor pins
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

volatile long int encoder_pos = 0;
int motor_speed;
int error = 10; // permissible error
int target_position = 0;
int i;
bool movement_completed = true;
bool moving_down = false;
uint16_t last_height = 0; //Initialize the height

void setup() {
  Serial.begin(9600);
  
  // Initialize RFID
  SPI.begin();
  rfid.PCD_Init();
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  
  // Initialize motor pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder, RISING);
  
  Serial.println("Height-Controlled Motor System Ready");
}

void loop() {
  if (movement_completed) {
    Serial.print(" Target: ");
    Serial.println(target_position);
    Serial.print(" Current: ");
    Serial.println(encoder_pos);

    // Looking for new RFID cards
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      uint16_t height = readHeightFromCard();
      if (height > 0 && height != last_height) {
        last_height = height;
        // Calculate target position: 0.25 * height
        target_position = (int)(0.25 * height);
        movement_completed = false;
        moving_down = false;
        encoder_pos = 0; // Reset position before starting
        Serial.print("New height detected: ");
        Serial.print(height);
        Serial.print(" cm, Moving to position: ");
        Serial.println(target_position);
      }
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  } else {
    if (!moving_down) {
      // Move up to target position
      moveToPos(); // Up movement with full speed
      if (abs(target_position - encoder_pos) <= error) {
        moving_down = true;
        target_position = 0;
        delay(5000); // Brief pause at the top
        Serial.println("Reached top position, starting descent");
      }
    } else {
      // Move down to zero
      moveToPos(); // Down movement
      if (abs(target_position - encoder_pos) <= error) {
        movement_completed = true;
        Serial.println("Movement cycle completed");
      }
    }
    
    Serial.print("Current Position: ");
    Serial.println(encoder_pos);
    delay(10);
  }
}

uint16_t readHeightFromCard() {
  byte block = 4;
  byte buffer[18];
  byte size = sizeof(buffer);
  
  MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.println("Authentication failed");
    return 0;
  }
  
  status = rfid.MIFARE_Read(block, buffer, &size);
  if (status == MFRC522::STATUS_OK) {
    if (buffer[2] == 0xCC) { // Check for height data marker
      return (buffer[0] << 8) | buffer[1];
    }
  }
  return 0;
}

void encoder() {
  if (digitalRead(ENCODER_PIN_B) == HIGH) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
}

void MotorClockwise(int power) {
  if (power > 60) {
    analogWrite(MOTOR_PIN1, power);
    digitalWrite(MOTOR_PIN2, LOW);
  } else {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
  }
}

void MotorCounterClockwise(int power) {
  if (power > 60) {
    analogWrite(MOTOR_PIN2, power);
    digitalWrite(MOTOR_PIN1, LOW);
  } else {
    digitalWrite(MOTOR_PIN2, LOW);
    digitalWrite(MOTOR_PIN1, LOW);
  }
}

void moveToPos() {
  while (abs(target_position - encoder_pos) > error) {
    Serial.print("Target Postion: ");
    Serial.println(target_position);
    if (target_position > encoder_pos) {
      motor_speed = 180;  // set the speed of the motor
      if (i == 0) {
        motor_speed = 200;  // April 2024
        i = 1;
      }
    } else if (target_position < encoder_pos) {
      motor_speed = -180;
      if (i == 0) {
        motor_speed = -200;  // April 2024
        i = 1;
      }
    } else {
      if (abs(target_position - encoder_pos) < error) {
        motor_speed = 0;
        i = 0;
        MotorClockwise(0); 
        delay(10); 
        return; 
      }
    }

    if (motor_speed > 0) {
      MotorClockwise(motor_speed);
    } else {
      MotorCounterClockwise(abs(motor_speed));
    }
    Serial.print("Current Position: ");
    Serial.println(encoder_pos);
    delay(10); 
  }
  MotorClockwise(0); 
  delay(10); 
}