#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 9
#define RST_PIN 10

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
  
  // Prepare the security key for authentication
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  
  Serial.println("RFID Height Data Writer Ready");
}

void loop() {
  // Look for new cards
  if (!rfid.PICC_IsNewCardPresent())
    return;

  // Select one of the cards
  if (!rfid.PICC_ReadCardSerial())
    return;

  Serial.println("Card detected!");
  
  // Show card type
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.print("Card Type: ");
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // Height value to store (180 cm)
  uint16_t height = 187;
  
  // Convert height to bytes
  byte heightData[16];
  memset(heightData, 0, 16); // Clear the buffer
  heightData[0] = (height >> 8) & 0xFF;  // High byte
  heightData[1] = height & 0xFF;         // Low byte
  heightData[2] = 0xCC;                  // Unit identifier (0xCC for centimeters)
  
  // Authenticate and write to block 4 (first block of sector 1)
  byte block = 4;
  byte sector = 1;
  
  // Authenticate the sector
  MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed: ");
    Serial.println(rfid.GetStatusCodeName(status));
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }

  // Write the data
  status = rfid.MIFARE_Write(block, heightData, 16);
  if (status == MFRC522::STATUS_OK) {
    Serial.println("Height data written successfully!");
    Serial.print("Stored height: ");
    Serial.print(height);
    Serial.println(" cm");
  } else {
    Serial.print("Write failed: ");
    Serial.println(rfid.GetStatusCodeName(status));
  }

  // Read back the data to verify
  byte readBuffer[18];
  status = rfid.MIFARE_Read(block, readBuffer, &readBuffer[16]);
  if (status == MFRC522::STATUS_OK) {
    uint16_t storedHeight = (readBuffer[0] << 8) | readBuffer[1];
    Serial.print("Verification - Read height: ");
    Serial.print(storedHeight);
    Serial.println(" cm");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  
  delay(1000);
}