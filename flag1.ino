
//This program reads a keypad and displays a LCD screen

// include the library code:
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <string.h>
#define RxD 8
#define TxD 9

char recvChar; 
String recvStr; 

String test; 

SoftwareSerial slave(RxD, TxD); 

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//string S1;
#define NOTE_E6 1319
#define BUZZER A4
// Pins for 74C922 keypad encoder
int KB_DataAvailable=A0;//Data ready pin from Keyboard
int KB_A=13;
int KB_B=12;
int KB_C=11;
int KB_D=10;
int GreenLED=A3;
int OrangeLED=A2;
int RedLED=A1;
int FOR;
int REV;
char k;

// Declare keypad layout
char keys[] = {'1','2','3','F',
               '4','5','6','E',
               '7','8','9','M',
               'A','0','B','C'};

#define NO_OF_CHAR 4
char d[NO_OF_CHAR+1];//FIFO queue for characters to display
char* flagThingy = d; 

void setup(void) {
  int i;

  pinMode(RxD, INPUT); 
  pinMode(TxD, OUTPUT); 
  slave.begin(9600); 
  recvStr = ""; 
  test = "24";

  for(i=0;i<NO_OF_CHAR;i++) d[i]=' ';
  d[NO_OF_CHAR]=0; // end with NULL
  //initialize pins for keypad
  pinMode(KB_DataAvailable,INPUT);
  pinMode(KB_A,INPUT);
  pinMode(KB_B,INPUT);
  pinMode(KB_C,INPUT);
  pinMode(KB_D,INPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(GreenLED,OUTPUT);
  pinMode(OrangeLED,OUTPUT);
  pinMode(RedLED,OUTPUT);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("bus number:");
  Serial.begin(9600);
  analogWrite(GreenLED,255);
  analogWrite(RedLED,255);
  analogWrite(OrangeLED,255);
  }

void loop(void) {
  Serial.print("recvStr: ");
  Serial.println(recvStr);
  if (slave.available()) {                            //if data is available from the master arduino
    recvChar = slave.read();                        //store character received
    Serial.print(recvChar);                         //display character received on the serial monitor

    recvStr = String(recvStr + String(recvChar));   //concatenate the recvChar
    delay(10); 
  }
  else{
    //if the received string is "Hello", then blink led and send a '1' back to master arduino
    if (recvStr[0] == '2' && recvStr[1] == '4' && recvStr[2] == 'E' )
    {
      analogWrite(OrangeLED,255);
    }

    if (recvStr[0] == '1' && recvStr[1] == '7' && recvStr[2] == '9') {
      analogWrite(GreenLED,255);
    }

    if (recvStr[0] == '1' && recvStr[1] == '9' && recvStr[2] == '9') {
      analogWrite(RedLED,255);
    }
    recvStr = "";                                  //clear the recvStr
  }
    //check if the keypad is hit
  if(digitalRead(KB_DataAvailable)){
    KB_Read(); //read the keypad
    delay(100); 

    lcd.setCursor(0, 1);
    // print the KEY being pressed:
    lcd.print(d);
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    delay(300);
    Serial.println(flagThingy);
    Serial.println(d);
    if(k==3){
      Serial.println(d);
      if(strcmp(d, " 179") == 0){
        analogWrite(GreenLED,0);  
        tone(BUZZER, NOTE_E6, 1000);
        //slave.print("GreenOn"));
        delay(1000);
        noTone(BUZZER);
        delay(100);
        for(int i=0;i<(NO_OF_CHAR);i++) d[i]=' ';
        lcd.setCursor(0, 1);
        // print the KEY being pressed:
        lcd.print(d);
      }
      else if(strcmp(d, " 24E") == 0){
        analogWrite(OrangeLED,0);
        tone(BUZZER, NOTE_E6, 1000);
        slave.print("OrangeOn");
        delay(1000);
        noTone(BUZZER);
        delay(100);
        for(int i=0;i<(NO_OF_CHAR);i++) d[i]=' ';
        lcd.setCursor(0, 1);
        // print the KEY being pressed:
        lcd.print(d);
      }
      else if(strcmp(d, " 199") == 0){
        analogWrite(RedLED,0);
        tone(BUZZER, NOTE_E6, 1000);
        slave.print("RedOn");
        delay(1000);
        noTone(BUZZER);
        delay(100);
        for(int i=0;i<(NO_OF_CHAR);i++) d[i]=' ';
        lcd.setCursor(0, 1);
        // print the KEY being pressed:
        lcd.print(d);
      }
      else{
        Serial.println(d);
          for(int i=0;i<(NO_OF_CHAR);i++) d[i]=' ';
        lcd.setCursor(0, 1);
        // print the KEY being pressed:
        lcd.print(d);
        lcd.clear();
        lcd.print("WRONG NUMBER");
        delay(500);
        lcd.clear();
        lcd.print("bus number:");

      }
    }   
  }
}

void KB_Read() {
  int ka,kb,kc,kd,i;
  ka=digitalRead(KB_A); //read encoder output A
  kb=digitalRead(KB_B); //read encoder output B
  kc=digitalRead(KB_C); //read encoder output C
  kd=digitalRead(KB_D); //read encoder output D

  k=ka+kb*2+kc*4+kd*8; // combine the encoder outputs 
  if(k==3){
    //flagThingy= (d[NO_OF_CHAR-1]-'0')+10*(d[NO_OF_CHAR-2]-'0')+100*(d[NO_OF_CHAR-3]-'0');
    //flagThingy= d ;
    //S1.append(1, d[NO_OF_CHAR-1]);
    //cout << S1 << endl;
    //flagThingy= S1+S2+S3;
    //for(i=0;i<(NO_OF_CHAR);i++) d[i]=' ';
    d[NO_OF_CHAR]=0;
  }
  else if(k==14){
    d[3]=d[2];
    d[2]=d[1];
    d[1]=d[0];
    d[0]=' ';
  }
  else if(k==15){
    for(i=0;i<(NO_OF_CHAR);i++) d[i]=' ';
  }
  //else if(k==12){
  //else if(k==14){
   //d[NO_OF_CHAR]=0;

  else{
    for(i=0;i<(NO_OF_CHAR-1);i++) d[i]=d[i+1];//move displayed characters in FIFO queue forward discarding the first one
    d[NO_OF_CHAR-1]=keys[k]; // update the key into the queue
    d[NO_OF_CHAR]=0; // end with NULL
  }
// if( d[NO_OF_CHAR-1]=='F'){
  //       int flagThingy= d[NO_OF_CHAR-2]+10*d[NO_OF_CHAR-3]+100*d[NO_OF_CHAR-4];
  //     } 
}
