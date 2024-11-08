#include <SoftwareSerial.h>
#include <DHT11.h> // Include the DHT11 library
#include <string.h>

// Sensor and LED pin definitions
#define DHT11_PIN 7  // DHT11 connected to digital pin 2
#define RAIN_PIN A0  // Raindrop sensor connected to analog pin A0
#define LED_PIN 4    // LED connected to digital pin 4

int rainThreshold = 500;  // Threshold for raindrop sensor
int tempMin = 18;         // Minimum temperature for rain in Celsius
int tempMax = 29;         // Maximum temperature for rain in Celsius
int humidityMin = 80;     // Minimum humidity for rain in percentage

DHT11 dht11(DHT11_PIN);  // Create a DHT11 object


#define encoderPinA 2
#define encoderPinB 3
#define pin1 5
#define pin2 6
#define echoPin 8
#define trigPin 9
#define RxD 13
#define TxD 12
 

char recvChar;
String recvStr; 

const int maxDist = 32;
const int minDist = 20; 
String message = "";
String lastbus = ""; 

volatile long int encoder_pos = 0;
const int error = 15;  // permissible error
int setPos = 0;        // Value range from -32768 to 32767
int motor_speed;
int i = 0;
int distcounter = 0;
int distcountermax = 5;
int btcounter = 0; 
int btcountermax = 50; 
int israining = 0; 

SoftwareSerial BTSerial(10, 11);  // RX | TX //11 connect to RX
SoftwareSerial master(RxD, TxD);


void setup() {
  // Start the serial communication with the computer
  Serial.begin(9600);

  pinMode(RxD, INPUT);                               //set mode of receive pin (from bluetooth)
  pinMode(TxD, OUTPUT);                              //set mode of transmit pin (to bluetooth)
  master.begin(9600);                                 //start the bluetooth serial "port"

  pinMode(RAIN_PIN, INPUT);     // Set rain sensor pin as input
  pinMode(LED_PIN, OUTPUT);     // Set LED pin as output
  digitalWrite(LED_PIN, LOW);   // Turn off LED initially

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);   // Sets the echoPin as an INPUT

  pinMode(pin1, OUTPUT);  // declares pin 4 as output
  pinMode(pin2, OUTPUT);  // declares pin 5 as output
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder, RISING);  // Enable the external interrupt


  // Start the Bluetooth communication at 9600 baud rate
  BTSerial.begin(9600);
  Serial.println("Bluetooth receiver ready");
  delay(2000); 
  // setPos = 0; 
  // moveToPos(); 
}

void loop() {
  Serial.print("lastbus: ");
  Serial.println(lastbus);
  if (BTSerial.available()) {
    // Read incoming message from Bluetooth
    // if distance in range read bluetooth
    if ((distance() <= maxDist) && (distance() >= minDist)) {
      message = BTSerial.readString();
      lastbus = message; 
    } else {
      lastbus = "";
      message = "";
    }
     
    // Display the message on the serial monitor
    Serial.println("Received: " + message);
    btcounter = 0; 
  } else if (btcounter == btcountermax) message = "";
  btcounter++; 

  Serial.println(message);
  // measure distance once every while
  if (distcounter == 0) {
    int dist = distance();
    israining = isRaining(); 
    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" cm");
    if (busHere() && isRaining()) {
      setPos = distToPos(dist);
    }
  } else if (distcounter == distcountermax) {
    distcounter = -1;
  }
  distcounter++;

  if (!busHere() && lastbus != "") {
    master.print(lastbus); 
    lastbus = "";
  }

  // bus just left
  if (!busHere() && !isRetracted()) {
    delay(4000); 
    Serial.print("Bushere: ");
    Serial.println(busHere());
    if (!busHere()) {
      setPos = 0;
      for (int j = 0; j < 10; j++) {
        while (abs(setPos - encoder_pos) >= error) moveToPos(); 
        delay(20); 
      }
    }
  }

  for (int j = 0; j < 10; j++) {
    while (abs(setPos - encoder_pos) >= error) moveToPos(); 
    delay(20); 
  }

  Serial.print("Current Position: ");
  Serial.println(encoder_pos);
  
}


void encoder() {
  if (digitalRead(encoderPinB) == HIGH) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
}

void MotorClockwise(int power) {
  if (power > 60) {  // set the min value as 60 because of the motor inertia
    analogWrite(pin1, power);
    digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

void MotorCounterClockwise(int power) {
  if (power > 60) {
    analogWrite(pin2, power);
    digitalWrite(pin1, LOW);
  } else {
    digitalWrite(pin2, LOW);
    digitalWrite(pin1, LOW);
  }
}

int distance() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2;  // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}

int isRaining() {
  // return 1; 
  int rainValue = analogRead(RAIN_PIN);  // Read raindrop sensor value
    int temperature = 0;                   // Variable for temperature
    int humidity = 0;                      // Variable for humidity

    // Read temperature and humidity from DHT11 sensor
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    // Print sensor readings
    Serial.print("Rain Sensor: ");
    Serial.print(rainValue);
    Serial.print(" | Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C | Humidity: ");
    Serial.print(humidity);
    Serial.print(" %");

    if (result == 0) { // Check if DHT11 readings are successful
        // Check if both rain sensor and DHT11 readings indicate rain
        if (rainValue < rainThreshold &&
            temperature >= tempMin && temperature <= tempMax &&
            humidity >= humidityMin) {
            Serial.println(" - Raining");
            digitalWrite(LED_PIN, HIGH); // Turn on LED if conditions are met
            return 1; 
        } else {
            Serial.println(" - Not Raining");
            digitalWrite(LED_PIN, LOW);  // Turn off LED otherwise
            return 0; 
        }
    } else {
        // Print error message if DHT11 reading fails
        Serial.print(" - Error: ");
        Serial.println(DHT11::getErrorString(result));
    }
  return 0; 
}

int isRetracted() {
  return (encoder_pos < error && encoder_pos > -error);
}

int busHere() {
  // return 1; 
  int busNear = (distance() <= maxDist) && (distance() >= minDist);
  int btNear = (message != "");
  return (btNear && busNear);
}

long distToPos(int distance) {
  long res = (distance-20) * 110;
  return res;
}

void moveToPos() {
  while (abs(setPos - encoder_pos) > error) {
    Serial.print("Target Postion: ");
    Serial.println(setPos);
    if (setPos > encoder_pos) {
      motor_speed = 150;  // set the speed of the motor
      if (i == 0) {
        motor_speed = 180;  // April 2024
        i = 1;
      }
    } else if (setPos < encoder_pos) {
      motor_speed = -150;
      if (i == 0) {
        motor_speed = -180;  // April 2024
        i = 1;
      }
    } else {
      if (abs(setPos - encoder_pos) < error) {
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


