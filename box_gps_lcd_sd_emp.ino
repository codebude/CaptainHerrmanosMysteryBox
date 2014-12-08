#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define BACKLIGHT_PIN     13
LiquidCrystal_I2C lcd(0x27, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);  // Set the LCD I2C address
byte customChar[8] = {
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};


#include <TinyGPS.h>
TinyGPS gps;
int gpsstate = 0;
float distance = -1;
float targetLat = 51.891583;//maltermeister
float targetLon = 10.425344;//maltermeister
float falt = -1;
float fAltDistFM = -1;
float targetAltM = 380;//goslar


#include <SD.h>                      // need to include the SD library
#define SD_ChipSelectPin 53  //example uses hardware SS pin 53 on Mega2560
#include <TMRpcm.h> 
TMRpcm tmrpcm;
int mysteryState = -1;

#include <EEPROM.h> //Needed to access the eeprom read write functions

#include "OneButton.h"
OneButton button(44, true);
int buttonState = 0;


long boottime = 0;
const int smokePin = A1;


#include <NewPing.h>
#define TRIGGER_PIN  41  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     40  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.



#include "Keypad.h"
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] =
 {{'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}}; 
byte rowPins[ROWS] = {
  35, 34, 33, 32}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {
  31, 30, 29, 28}; // connect to the column pinouts of the keypad 
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

boolean final = false;

int smokeLed = 3;

#include <Servo.h>  
Servo myservo;
int pos = 0;    

void(* resetFunc) (void) = 0; //declare reset function @ address 0


void setup()
{
   
  boottime = millis();
  
  lcd.begin(20,4);               // initialize the lcd 
  lcd.home ();                   // go home
  lcd.createChar(0, customChar);
  
  Serial.begin(9600);
  Serial2.begin(9600);

 Serial.print(F("free ram="));
  Serial.println(freeRam());
 
  tmrpcm.speakerPin = 6;

  
  pinMode(53, OUTPUT); 
  if (!SD.begin(SD_ChipSelectPin, SPI_HALF_SPEED)) {  // see if the card is present and can be initialized:
    Serial.println(F("SD fail"));  
    //return;   // don't do anything more if not
  }  
   
  //Disable white noise
  tmrpcm.play("laugh.wav");
  tmrpcm.disable();   
  
  Serial.println(mysteryState);
  loadMysteryState();
  Serial.println(mysteryState);  
  
  lcd.clear();
  lcd.home();
  lcd.print(F(" AHOI, DU LANDRATTE"));  
  String setLevelCode = "";
  for (int i = 0; i < 20; i++)
  {
    lcd.setCursor ( i, 3 );
    lcd.write((uint8_t)0);
    
   for (int x = 0; x < 125; x++)
   {
      char key = keypad.getKey();
      if (key != NO_KEY)
      {
        setLevelCode += key;
      }
      delay(2);
   }
    
  }
  
  if (setLevelCode == "0160")
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Error: §%34&$86?@"));  
    lcd.setCursor ( 0, 3 );
    int nLevel = -1;
    while (nLevel == -1)
    {
        char key = keypad.getKey();
        if (key != NO_KEY)
        {
          nLevel = key - 48;
          Serial.println(key);
          Serial.println(nLevel);
          setMysteryState(nLevel);
        }
    }
    resetFunc();
  }
  
  button.attachClick(buttonClicked);
  button.setClickTicks(150);
 
  if (mysteryState != 9)
  {
    lockBox();
  }
  else
  {
   unlockBox(); 
  }
  Serial.println(freeRam());
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
      {
      byte lowByte = ((p_value >> 0) & 0xFF);
      byte highByte = ((p_value >> 8) & 0xFF);

      EEPROM.write(p_address, lowByte);
      EEPROM.write(p_address + 1, highByte);
      }

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
      {
      byte lowByte = EEPROM.read(p_address);
      byte highByte = EEPROM.read(p_address + 1);

      return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
      }



void loop()
{
  
  button.tick();
  if (mysteryState == 0)
  {
    mysteryIntro();
  }
  else if (mysteryState == 1)
  {
    mysteryKnock(); 
  }
  else if (mysteryState == 2)
  {
    mysteryFace(); 
  }
  else if (mysteryState == 3)
  {
    mysteryMail(); 
  }
  else if (mysteryState == 4)
  {
    mysterySmoke();
  }
  else if (mysteryState == 5)
  {    
     mysteryGpsAltitude();
  }
  else if (mysteryState == 6)
  {
     mysteryTemp();
  }
  else if (mysteryState == 7)
  {
     mysteryPhone(); 
  }
  else if (mysteryState == 8)
  {   
     mysteryGps();
  }
  else if (mysteryState == 9)
  {   
     mysteryFinal();
  }
  
}


void buttonClicked() {
  buttonState = 2;
} 


void loadMysteryState()
{
  int states = EEPROMReadInt(0);
  if (states == -1)
  {
    setMysteryState(0);
  }
  mysteryState = EEPROMReadInt(0);
  /*
 if (!SD.exists("mstate.txt"))
 {
   setMysteryState(0);
 } 
 else
 {
   Serial.println(F("file exists")); 
 }
 File statef = SD.open("mstate.txt", FILE_READ);
 if (statef)
 {
   
   if (statef.available())
   {     
     char a = statef.read();
     mysteryState = String(a).toInt();
     
   }
   statef.close();
 }
 else
 {
  Serial.println(F("file not available")); 
 }
 Serial.println(mysteryState);
 */
}

void setMysteryState(int sb)
{
  /*
   SD.remove("mstate.txt");
   delay(500);
   File statef = SD.open("mstate.txt", FILE_WRITE);
   if (statef)
   {
   statef.write(sb);
   statef.close(); 
   }
   else
   {
    Serial.println(F("konnte status nicht schreiben")); 
   }
   */
   EEPROMWriteInt(0, sb);
}

void mysteryKnock()
{
  
}

void mysteryTemp()
{
  if (buttonState == 0)
  {
    
    
    lcd.clear();
    lcd.home();
    lcd.print(F("Sehr kalt so hoch."));
    lcd.setCursor ( 0, 1 );
    lcd.print(F("Mein metallener"));
    lcd.setCursor ( 0, 2 );
    lcd.print(F("Finger friert."));  
    
    
    sensors.begin();
    
    sensors.requestTemperatures(); // Send the command to get temperatures
    float actTemp = sensors.getTempCByIndex(0);
    while (actTemp < 70.0)
    {      
      lcd.setCursor ( 0, 3 );
      lcd.print(F("-                  +"));
      lcd.setCursor ( 0, 3 );
      int steps = floor((actTemp / 0.7)*0.18);
      for (int i = 1; i <= steps; i++)
      {
        lcd.setCursor ( i, 3 );
        lcd.write((uint8_t)0);
      }
      Serial.println(actTemp);
      delay(1000);
      sensors.requestTemperatures(); 
      actTemp = sensors.getTempCByIndex(0);
    }
    
    lcd.clear();
    lcd.home();
    lcd.print(F("Arr! So ist's schon "));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("besser. Danke!"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
    buttonState = 1;    
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(7);
    loadMysteryState();  
  }
}



void mysterySmoke()
{
  if (buttonState == 0)
  {
    int sensorValCheck = analogRead(smokePin); 
    if (sensorValCheck > 300)
    {
      long sinceboot = (millis() - boottime) / 1000;
      while (sinceboot < 180)
      {
          long rest = 180 - sinceboot;
          lcd.clear();
          lcd.home();
          lcd.print(F("Muss mich warm"));
          lcd.setCursor ( 0, 1 );
          lcd.print(F("machen. Brauche"));
          lcd.setCursor ( 0, 2 );
          lcd.print(F("Zeit."));  
          lcd.setCursor ( 0, 3 );
          lcd.print(F("Noch ")); lcd.print(rest);lcd.print(F(" Sekunden."));  
          delay(1000);
          sinceboot = (millis() - boottime) / 1000;
      }
    }
    
    lcd.clear();
    lcd.home();
    lcd.print(F("Mein Captain hat"));
    lcd.setCursor ( 0, 1 );
    lcd.print(F("so feines Kraut"));
    lcd.setCursor ( 0, 2 );
    lcd.print(F("geraucht. Sag mir"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("kannst du das auch?")); 
    
    int sensorVal = 0;
    Serial.println(sensorVal);
    int value;
    long time=0;    
    int periode = 2000;
    int displace = 500;

    while (sensorVal <= 425)
    {
      sensorVal = analogRead(smokePin);   
      Serial.println(sensorVal);
     
      time = millis();
      value = 128+127*cos(2*PI/periode*time);
      analogWrite(smokeLed, value);           // sets the value (range from 0 to 255) 
           
    }
    
    digitalWrite(smokeLed, LOW);
    lcd.clear();
    lcd.home();
    lcd.print(F("Danke, dass tat gut."));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("Auf zu neuen Ufern."));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
    buttonState = 1;    
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(5);
    loadMysteryState();  
  }
}


void mysteryPhone()
{
  if (buttonState == 0)
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Schnappe dir einen"));
    lcd.setCursor ( 0, 1 );
    lcd.print(F("Fernsprecher, arrr!"));
    lcd.setCursor ( 0, 2 );
    lcd.print(F("021/329729943")); 
    
    tmrpcm.play("lach1.wav");    
    while (tmrpcm.isPlaying())
    { }    
    tmrpcm.disable();
    
    String loesung = "";
    boolean firstkey = true;
    int count = 0;
    while (loesung != "73")
    {     
      char key = keypad.getKey();
      
      if (key != NO_KEY)
      {
        Serial.println(key);
        loesung += key;
        if (firstkey)
        {
          firstkey = false;
          lcd.setCursor ( 0, 3 );       
          lcd.print(F("XX                  "));
        }
        
        lcd.setCursor ( 0, 3 );
        lcd.print(loesung);  
        
      
        count++;
        if (count==2 && loesung != "73")
        {
          lcd.setCursor ( 0, 3 );       
          lcd.print(F("XX"));
          count=0;
          loesung = "";
        }
      }
    }
    lcd.clear();
    lcd.home();
    lcd.print(F("Geschafft! Noch"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("eine letzte H")); lcd.write(char(245)); lcd.print(F("rde")); 
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
    buttonState = 1;    
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(8);
    loadMysteryState();    
  }
}


void mysteryMail()
{
  if (buttonState == 0)
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Die n"));  lcd.print(char(225)); lcd.print(F("chste L"));lcd.write(char(239)); lcd.print(F("sung"));
    lcd.setCursor ( 0, 1 );
    lcd.print(F("wei"));  lcd.print(char(226)); lcd.print(F(" nur der"));
    lcd.setCursor ( 0, 2 );
    lcd.print(F("Captain. Frag nach"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("dem \"Geheimnis\".")); 
    delay(6000);
    lcd.clear();
    lcd.home();
    lcd.print(F("Des Captains"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("toter Briefkasten:")); 
    lcd.setCursor ( 0, 2 );
    lcd.print(F("captain.herrmano"));  
    lcd.setCursor ( 0, 3 );
    lcd.write(char(64)); lcd.print(F("gmail.com")); 
    
    String loesung = "";
    boolean firstkey = true;
    int count = 0;
    while (loesung != "0972")
    {     
      char key = keypad.getKey();
      
      if (key != NO_KEY)
      {
        Serial.println(key);
        loesung += key;
        if (firstkey)
        {
          firstkey = false;
          lcd.clear();
          lcd.home();
          lcd.print(F("captain.herrmano"));  
          lcd.setCursor ( 0, 1 );
          lcd.write(char(64)); lcd.print(F("gmail.com")); 
          lcd.setCursor ( 0, 3 );       
          lcd.print(F("XXXX"));
        }
        
        lcd.setCursor ( 0, 3 );
        lcd.print(loesung);  
        
      
        count++;
        if (count==4 && loesung != "0972")
        {
          lcd.setCursor ( 0, 3 );       
          lcd.print(F("XXXX"));
          count=0;
          loesung = "";
        }
      }
    }
    lcd.clear();
    lcd.home();
    lcd.print(F("Geschafft!"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("Der Captain ist"));  
    lcd.setCursor ( 0, 2 );
    lcd.print(F("stolz auf dich."));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
    buttonState = 1;    
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(4);
    loadMysteryState();    
  }
}

void mysteryFace()
{
  if (buttonState == 0)
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Meine Augen liegen"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("nah beieinander."));  
    lcd.setCursor ( 0, 2 );
    lcd.print(F("Wie Siebe sehen "));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("sie aus.")); 
    delay(6000);
    lcd.clear();
    lcd.home();
    lcd.print(F("Komm, ein wenig"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("n")); lcd.print(char(225)); lcd.print(F("her. Ich will"));
    lcd.setCursor ( 0, 2 );
    lcd.print(F("dich sehen!"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Aber langsam!")); 
    
    NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
    boolean run = true;
    boolean printhalf = false;
    while (run)
    {
       delay(50); 
       unsigned int uS = sonar.ping_median(40);
       int distTemp = sonar.convert_cm(uS);
       Serial.println(distTemp); 
       
       if (distTemp < 25)
       {
         if (!printhalf)
         {
           printhalf = true;
          lcd.clear();
          lcd.home();           
          lcd.print(F("Noch n")); lcd.print(char(225)); lcd.print(F("her!"));
          lcd.setCursor ( 0, 1 );
          lcd.print(F("Komm her, du")); 
          lcd.setCursor ( 0, 2 );
          lcd.print(F("Landratte.")); 
         }
       }
       if (distTemp < 6)
       {
         run = false;
         
       }
    }
    
   NewPing::timer_stop();
   
    lcd.clear(); 
     
    tmrpcm.play("schrei1.wav");    
    while (tmrpcm.isPlaying())
    { }    
    tmrpcm.disable();
    
    
    lcd.home();
    lcd.print(F("Erschreckt? Wer"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("meinen Schatz will,"));  
    lcd.setCursor ( 0, 2 );
    lcd.print(F("muss furchtlos sein!"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
   
   buttonState = 1;
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(3);
    loadMysteryState();    
  }
    
}



void mysteryIntro()
{
  if (buttonState == 0)
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Ich bin Captain"));  
    lcd.setCursor ( 0, 1 );
    lcd.print(F("Herrmanos Schatz."));  
    lcd.setCursor ( 0, 2 );
    lcd.print(F("Um mich zu knacken,"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("musst du stark sein.")); 
    delay(4000);
    lcd.clear();
    lcd.home();
    lcd.print(F("Bist du bereit f"));  lcd.write(char(245));  lcd.print(F("r")); 
    lcd.setCursor ( 0, 1 );
    lcd.print(F("ein paar R")); lcd.print(char(225)); lcd.print(F("tsel?"));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
    buttonState = 1;
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(2);
    loadMysteryState();    
  }
  
  
}



void mysteryGpsAltitude()
{
  if (buttonState == 0)
  {
      bool newData = false;
      unsigned long chars;
      unsigned short sentences, failed;
    
      // For one second we parse GPS data and report some key values
      for (unsigned long start = millis(); millis() - start < 1000;)
      {
        while (Serial2.available())
        {
          char c = Serial2.read();
          //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
          if (gps.encode(c)) // Did a new valid sentence come in?
            newData = true;
        }
      }
    
    
      float flat, flon;
      unsigned long age;
      if (newData)
      {
           
        gps.f_get_position(&flat, &flon, &age);
        Serial.print(F("LAT="));
        Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
        Serial.print(F(" LON="));
        Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
        Serial.print(F(" SAT="));
        Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
        Serial.print(F(" PREC="));
        Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
        
        if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES && gps.satellites() > 2)
        {
           falt = gps.f_altitude();
           fAltDistFM = (targetAltM - falt)/1.8288;
           Serial.print(F(" ALT="));
           Serial.print(falt);
           gpsstate = 2; 
        }
      }
    
      if (gpsstate == 0)
      {
           lcd.clear();
           lcd.home();
           lcd.print(F("Gib mir Zeit,"));  
           lcd.setCursor ( 0, 1 );
           lcd.print(F("muss Himmel sehen,"));  
           lcd.setCursor ( 0, 2 );
           lcd.print(F("bin gleich bereit,"));  
           lcd.setCursor ( 0, 3 );
           lcd.print(F("lass mich stehen."));  
           gpsstate = 1;
      }
      else if (gpsstate == 2)
      {
        lcd.clear();
        lcd.home();
        lcd.print(F("Zeig mir Berge und"));  
        lcd.setCursor ( 0, 1 );
        lcd.print(F("kein Meer. Noch"));  
        lcd.setCursor ( 0, 2 );
        lcd.print(fAltDistFM,0); lcd.print(F(" Nautische Faden"));          
        lcd.setCursor ( 0, 3 );
        lcd.print(F("in Richtung Himmel."));  
        
        gpsstate = 1;
      }
      
      if ((falt+1.8)>=targetAltM)
      {
        
        lcd.clear();
        lcd.home();
        lcd.print(F("Jetzt f")); lcd.write(char(245)); lcd.print(F("hle ich mich"));  
        lcd.setCursor ( 0, 1 );
        lcd.print(F("frei genug f")); lcd.write(char(245)); lcd.print(F("r die"));  
        lcd.setCursor ( 0, 2 );
        lcd.print(F("neue Aufgabe."));  
        lcd.setCursor ( 0, 3 );
        lcd.print(F("Dr"));  lcd.write(char(245)); lcd.print(F("cke meinen Kopf!")); 
        buttonState = 1;    
        
      }
    
      gps.stats(&chars, &sentences, &failed);
      Serial.print(F(" CHARS="));
      Serial.print(chars);
      Serial.print(F(" SENTENCES="));
      Serial.print(sentences);
      Serial.print(F(" CSUM ERR="));
      Serial.print(failed);
      Serial.print(F(" SATS="));
       Serial.print(F(" free ram="));
  Serial.println(freeRam());
      if (chars == 0)
        Serial.println(F("** No characters received from GPS: check wiring **")); 
        
  }
  else if (buttonState == 2)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(6);
    loadMysteryState();    
  }
}




void mysteryGps()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }


  float flat, flon;
  unsigned long age;
  if (newData)
  {
       
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(F("LAT="));
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(F(" LON="));
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(F(" SAT="));
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(F(" PREC="));
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.print(F(" ALT(in m)="));
    Serial.print(gps.f_altitude());
    if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES && gps.satellites() > 2)
    {
       getDistance(targetLat, targetLon, flat, flon); 
       gpsstate = 2; 
    }
  }

  if (gpsstate == 0)
  {
       lcd.clear();
       lcd.home();
       lcd.print(F("Gib mir Zeit,"));  
       lcd.setCursor ( 0, 1 );
       lcd.print(F("muss Himmel sehen,"));  
       lcd.setCursor ( 0, 2 );
       lcd.print(F("bin gleich bereit,"));  
       lcd.setCursor ( 0, 3 );
       lcd.print(F("lass mich stehen."));  
       gpsstate = 1;
  }
  else if (gpsstate == 2)
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Finde den Schatz!"));
    lcd.setCursor ( 0, 1 );
    lcd.print(F("Nur die Strecke,"));  
    lcd.setCursor ( 0, 2 );
    lcd.print(F("keine Richtung."));  
    lcd.setCursor ( 0, 3 );
    lcd.print(F("Noch "));  
    lcd.print(distance);
    lcd.print(F(" km."));
    gpsstate = 1;
  }
  
  if (distance != -1 && distance <= 0.05)
  {
    lcd.clear();
    buttonState = 0;     
    setMysteryState(9);
    loadMysteryState();     
  }
  
    

  gps.stats(&chars, &sentences, &failed);
  Serial.print(F(" CHARS="));
  Serial.print(chars);
  Serial.print(F(" SENTENCES="));
  Serial.print(sentences);
  Serial.print(F(" CSUM ERR="));
  Serial.println(failed);
  if (chars == 0)
    Serial.println(F("** No characters received from GPS: check wiring **")); 
    
    
}


void mysteryFinal()
{
   if (!final)
   {
     final = true;
     unlockBox();
     lcd.clear();
     lcd.home();
     lcd.print(F("Der Schatz ist dein,"));  
     lcd.setCursor ( 0, 1 );
     lcd.print(F("ich ergebe mich,"));  
     lcd.setCursor ( 0, 2 );
     lcd.print(F("nun schau hinein,"));  
     lcd.setCursor ( 0, 3 );
     lcd.print(F("was da ist f")); lcd.write(char(245)); lcd.print(F("r dich."));  
     
     tmrpcm.play("win.wav");    
     while (tmrpcm.isPlaying())
     { }    
     tmrpcm.disable();
   }
}



void getDistance(float targetLat, float targetLon, float posLat, float posLon)
{
  float latRad, lonRad;
  float tlatRad, tlonRad;
  float midLat, midLon;


  //convert decimal degree into radian
  latRad = posLat * 0.017453293;
  lonRad = posLon * 0.017453293;
  tlatRad = targetLat * 0.017453293;
  tlonRad = targetLon * 0.017453293;

  midLat = tlatRad - latRad;
  midLon = tlonRad - lonRad;

  //Calculate the distance in KM
  float latSin = sin((latRad - tlatRad)/2);
  float lonSin = sin((lonRad - tlonRad)/2);
  distance = 2 * asin(sqrt((latSin*latSin) + cos(latRad) * cos(tlatRad) * (lonSin * lonSin)));
  distance = distance * 6371;
  
  
  falt = gps.f_altitude();
  fAltDistFM = (targetAltM - falt)/1.8288;
}

void lockBox()
{  
  myservo.attach(12);
  pos = myservo.read();
    myservo.write(pos);
  while (pos > 0)
  {
     pos -= 1;
    myservo.write(pos); 
    delay(20);
    myservo.write(pos+1);
    myservo.write(pos);
  }
  myservo.detach();
}

void unlockBox()
{
    myservo.attach(12);
    pos = myservo.read();
    myservo.write(pos);
  while (pos < 140)
  {
     pos += 1;
    myservo.write(pos); 
        delay(20);
        myservo.write(pos+1); 
        myservo.write(pos);
  }
    myservo.detach();
}
