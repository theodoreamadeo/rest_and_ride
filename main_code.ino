#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

Servo myServo;  // create servo object to control a servo

#define SS_PIN 9
#define RST_PIN 10
#define INITIAL_HEIGHT 20  // Initial height in cm (from ground)
#define COUNTS_PER_CM 100  // Adjust this based on your encoder resolution
#define BUTTON_UP_DISTANCE 20     // Distance to move up in cm when button is pressed
#define MIN_HEIGHT INITIAL_HEIGHT // Minimum allowed height is the initial height

volatile long int encoder_pos = 0;
int motor_speed;
int pin1 = 5;   // motor pin1
int pin2 = 6;   // motor pin2
int encoderPinA = 2;
int encoderPinB = 3;
int error = 50; // permissible error
int setPos = 0;   
uint16_t recent_height = 0;
bool movement_completed = true;
float lastCalculatedHeight = 0;
bool heightSet = false;  // Flag to track if initial height has been set

// Define pin numbers for servo and button
const int servoPin = 4;      // Servo control pin
const int buttonPin = 7;     // Button pin
const int ledPin = 13;       // LED pin

// State variables
int val;                     // Variable for servo position
int buttonState = 0;         // Variable for reading the button status
int speedDelay = 25;
bool tiltingComplete = false;
bool movingDown = false;

enum SystemState {
    IDLE,
    INITIAL_MOVEMENT,
    WAITING_FOR_BUTTON,
    MOVING_UP,
    TILTING,
    MOVING_DOWN
} currentState = IDLE;

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

float calculateTargetHeight(uint16_t personHeight) {
    float ergonomicHeight = (personHeight * 0.25);  // Calculate total height from ground
    float minHeight = MIN_HEIGHT;  // Minimum height is INITIAL_HEIGHT from ground
    float maxHeight = 60;  // Maximum height from ground
    
    // Constrain the total height and then subtract INITIAL_HEIGHT to get relative movement
    ergonomicHeight = constrain(ergonomicHeight, minHeight, maxHeight);
    float relativeHeight = ergonomicHeight - INITIAL_HEIGHT;  // Convert to relative movement
    
    Serial.print("Person height: ");
    Serial.print(personHeight);
    Serial.print(" cm, Total target height: ");
    Serial.print(ergonomicHeight);
    Serial.print(" cm from ground (Relative movement: ");
    Serial.print(relativeHeight);
    Serial.println(" cm from initial position)");
    
    return relativeHeight;  // Return relative movement needed
}

void moveToPosition(float targetHeight) {
    // Convert target height to absolute position from ground
    float absoluteHeight = INITIAL_HEIGHT + targetHeight;
    
    // For moving down state, force the target to be exactly INITIAL_HEIGHT
    if (currentState == MOVING_DOWN) {
        absoluteHeight = INITIAL_HEIGHT;  // Force to 20cm
        targetHeight = 0;  // No relative movement from INITIAL_HEIGHT
    }
    
    // Ensure we never go below INITIAL_HEIGHT
    if (absoluteHeight < MIN_HEIGHT) {
        absoluteHeight = MIN_HEIGHT;
        Serial.println("Warning: Limiting to minimum height");
    }
    
    // Convert to encoder counts
    // testt disini
    setPos = targetHeight * COUNTS_PER_CM;  // This will be 0 for MOVING_DOWN
    
    // Add debug information
    Serial.print("Current encoder position: ");
    Serial.print(encoder_pos);
    Serial.print(", Target position: ");
    Serial.println(setPos);
    
    movement_completed = false;
    
    Serial.print("Moving to absolute height: ");
    Serial.print(absoluteHeight);
    Serial.print(" cm from ground (Relative movement: ");
    Serial.print(targetHeight);
    Serial.print(" cm, Encoder target: ");
    Serial.print(setPos);
    Serial.println(" counts)");
}

void updateMotorPosition() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 10) return;
    lastUpdate = millis();
    
    int position_error = setPos - encoder_pos;
    
    // Calculate current absolute height
    float currentRelativeHeight = (float)encoder_pos / COUNTS_PER_CM;
    float currentAbsoluteHeight = INITIAL_HEIGHT + currentRelativeHeight;
    
    Serial.print("Current absolute height: ");
    Serial.print(currentAbsoluteHeight);
    Serial.println(" cm");
    
    // Strict check for MOVING_DOWN state
    if (currentState == MOVING_DOWN) {
        if (abs(encoder_pos) <= error) {  // Check if we're very close to 0 (INITIAL_HEIGHT)
            encoder_pos = 0;  // Force exact position
            movement_completed = true;
            MotorStop();
            Serial.println("Reached exactly 20cm (INITIAL_HEIGHT) - stopping");
            return;
        }
        
        // If we're moving down and somehow below 20cm, stop immediately
        if (currentAbsoluteHeight < INITIAL_HEIGHT) {
            encoder_pos = 0;
            movement_completed = true;
            MotorStop();
            Serial.println("Below 20cm - stopping and correcting position");
            return;
        }
    }
    
    // Determine motor speed based on distance from target 
    if (abs(position_error) <= error) {
        movement_completed = true;
        MotorStop();
        Serial.println("Position reached - stopping");
        return;
    }
    
    // Use slower speed when approaching INITIAL_HEIGHT during down movement
    if (currentState == MOVING_DOWN && currentAbsoluteHeight < INITIAL_HEIGHT) {
        motor_speed = 200;  // Very slow speed for precise positioning
    } else {
        motor_speed = constrain(abs(position_error) / 2, 200, 255);
    }
    
    Serial.print("Motor speed: ");
    Serial.print(motor_speed);
    Serial.print(", Error: ");
    Serial.println(position_error);
    
    // Motor direction control
    if (position_error > 0) {
        MotorClockwise(motor_speed);
    } else if (position_error < 0) {
        MotorCounterClockwise(motor_speed);
    } else {
        MotorStop();
    }
}

void servoTilt() {
    Serial.println("Starting tilt motion");
    
    // Move the servo up
    for (val = 0; val <= 100; val++) {
        myServo.write(val);
        delay(speedDelay);
    }
    
    delay(1000);  // Hold at tilted position
    
    // Move the servo down
    for (val = 100; val >= 0; val--) {
        myServo.write(val);
        delay(15);
    }
    
    tiltingComplete = true;
    Serial.println("Tilt motion complete");
}

void setup() {
    Serial.begin(9600);
    
    myServo.attach(servoPin);
    myServo.write(0);
    
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderB, CHANGE);

    SPI.begin();
    rfid.PCD_Init();
    
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }
    
    Serial.println("System Ready - Waiting for RFID card...");
}

void loop() {
  Serial.print("encoder pos: ");
  Serial.println(encoder_pos); 
    static bool buttonWasPressed = false;
    buttonState = digitalRead(buttonPin);

    // Always check for RFID card when in IDLE or WAITING_FOR_BUTTON states
    if (currentState == IDLE || currentState == WAITING_FOR_BUTTON) {
        if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
            uint16_t height = readHeightFromCard();
            if (height > 0) {
                // Remove the height comparison check
                recent_height = height;
                lastCalculatedHeight = calculateTargetHeight(height);
                moveToPosition(lastCalculatedHeight);
                currentState = INITIAL_MOVEMENT;
                heightSet = true;
                Serial.println("Card read - Moving to initial position");
                
                // Add debug information
                Serial.print("Previous height: ");
                Serial.print(recent_height);
                Serial.print(", New height: ");
                Serial.println(height);
            }
            rfid.PICC_HaltA();
            rfid.PCD_StopCrypto1();
        }
    }
    
    switch (currentState) {
        case IDLE:
            // Reset all flags when entering IDLE state
            buttonWasPressed = false;
            tiltingComplete = false;
            movement_completed = true;  // Add this line
            digitalWrite(ledPin, LOW);
            break;
            
        case INITIAL_MOVEMENT:
            if (!movement_completed) {
                updateMotorPosition();
            } else {
                Serial.println("Initial position reached - Ready for button sequence");
                currentState = WAITING_FOR_BUTTON;
                buttonWasPressed = false;  // Reset button state
            }
            break;
            
        case WAITING_FOR_BUTTON:
            // Check for button press
            if (buttonState == LOW && !buttonWasPressed && heightSet) {
                buttonWasPressed = true;
                digitalWrite(ledPin, HIGH);
                moveToPosition(lastCalculatedHeight + BUTTON_UP_DISTANCE);
                currentState = MOVING_UP;
                Serial.println("Button pressed - Moving up");
            }
            break;
            
        case MOVING_UP:
            if (!movement_completed) {
                updateMotorPosition();
            } else {
                Serial.println("Up movement complete, starting tilt");
                currentState = TILTING;
                tiltingComplete = false;
            }
            break;
            
        case TILTING:
            if (!tiltingComplete) {
                servoTilt();
            } else {
                Serial.println("Tilt complete, moving down");
                moveToPosition(0);
                currentState = MOVING_DOWN;
            }
            break;
            
        case MOVING_DOWN:
            if (!movement_completed) {
                updateMotorPosition();
            } else {
                Serial.println("Sequence complete - Ready for next button press");
                currentState = WAITING_FOR_BUTTON;
                buttonWasPressed = false;
                digitalWrite(ledPin, LOW);
                // Don't reset heightSet flag here to allow multiple cycles
            }
            break;
    }
    
    // Reset button state when released
    if (buttonState == HIGH) {
        buttonWasPressed = false;
    }
    
    // Debug info
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        Serial.print("Current State: ");
        Serial.print(currentState);
        Serial.print(" Position: ");
        Serial.print(encoder_pos);
        Serial.print(" counts (");
        Serial.print((float)encoder_pos / COUNTS_PER_CM);
        Serial.println(" cm)");
        lastPrint = millis();
    }
}


void encoderA() {
    if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
        encoder_pos++;
    } else {
        encoder_pos--;
    }
}

void encoderB() {
    if (digitalRead(encoderPinA) != digitalRead(encoderPinB)) {
        encoder_pos++;
    } else {
        encoder_pos--;
    }
}

void MotorStop() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    Serial.println("Motor STOPPED");
}

void MotorClockwise(int power) {
    power = constrain(power, 0, 255);
    analogWrite(pin1, power);
    digitalWrite(pin2, LOW);
}

void MotorCounterClockwise(int power) {
    power = constrain(power, 0, 255);
    digitalWrite(pin1, LOW);
    analogWrite(pin2, power);
}

uint16_t readHeightFromCard() {
    byte block = 4;
    byte buffer[18];
    byte size = sizeof(buffer);
    
    Serial.println("Attempting to read card...");
    
    MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Authentication failed");
        return 0;
    }
    
    status = rfid.MIFARE_Read(block, buffer, &size);
    if (status == MFRC522::STATUS_OK) {
        if (buffer[2] == 0xCC) {  // Verification byte
            uint16_t height = (buffer[0] << 8) | buffer[1];
            Serial.print("Height read from card: ");
            Serial.println(height);
            return height;
        }
    }
    Serial.println("Failed to read height from card");
    return 0;
}