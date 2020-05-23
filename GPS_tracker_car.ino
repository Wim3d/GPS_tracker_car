/*
   Written by W. Hoogervorst
   January 2019
   Arduino Pro mini 5V 328
*/

#include <TimeLib.h>
#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <Timezone.h>

#define A6Serial Serial       // A6 module is connected to hardware serial
//#define //DebugSerial Serial

//define serial connection for GPS
AltSoftSerial GPS_serial;  // RX = 8 (to TX op GPS) , TX = 9 (to RX of GPS) PWM 10 is not available

//define GPS variable
TinyGPSPlus gps;

// time variables
const int uploadinterval = 300; //upload GPS data every # seconds
time_t local, utc, lastupload;
// Change these two rules corresponding to your timezone, see https://github.com/JChristensen/Timezone
//Central European Time (Frankfurt, Paris)  120 = +2 hours in daylight saving time (summer).
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};
//Central European Time (Frankfurt, Paris)  60  = +1 hour in normal time (winter)
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};
Timezone CE(CEST, CET);

#define SPEEDLIMIT 2
#define OKLED 11
#define NOTOKLED 10
#define A6ACTIONLED 12

// variables for the incoming phone numbers
char phone_number[12] = "31610778730";
char received[13];

int uploadcounter = 0;

char end_c[2];

int8_t answer = 0, answer1 = 0;
uint32_t calltime = 0, maxtime = 5000;
boolean incoming;

boolean speedflag = true;

//temperature and thermistor values and variables
#define THERMISTORPIN A0
#define THERMISTORNOMINAL 10000   // resistance at 25 degrees C
#define TEMPERATURENOMINAL 25     // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5
#define BCOEFFICIENT 3950         // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000      // the value of the 'other' resistor
uint16_t samples[NUMSAMPLES];
int8_t temperature;               // must be signed, otherwise negative temperatures are not shown correctly
boolean tempsmsflag1 = true; // no ALARM sms is sent yet
boolean tempsmsflag2 = true; // no ALARM sms is sent yet
const int tempcheckinterval = 300; //check temp every # seconds
time_t lasttemp;
#define ALARM 1
#define NO_ALARM 0
#define TEMPERATURELIMIT1 40
#define TEMPERATURELIMIT2 55
#define THERMISTOR_POWER 13

void setup() {
  A6Serial.begin(115200);
  // ctrlZ String definition
  end_c[0] = 0x1a;
  end_c[1] = '\0';
  pinMode(OKLED, OUTPUT);
  pinMode(NOTOKLED, OUTPUT);
  pinMode(A6ACTIONLED, OUTPUT);
  pinMode(THERMISTOR_POWER, OUTPUT);
  digitalWrite(OKLED, LOW);
  digitalWrite(NOTOKLED, LOW);
  digitalWrite(A6ACTIONLED, LOW);
  digitalWrite(THERMISTOR_POWER, LOW);
  blinkLEDS(); //leds blinking

  GPS_serial.begin(9600); //This opens up communications to the GPS
  smartDelay(1000);
  A6start();
  digitalWrite(NOTOKLED, HIGH);
  while (gps.satellites.value() == 0) // wait for a GPS fix to know the UTC time
  {
    smartDelay(500);
    digitalWrite(OKLED, !digitalRead(OKLED));
    digitalWrite(NOTOKLED, !digitalRead(NOTOKLED));
  }
  digitalWrite(OKLED, HIGH);
  setthetime();
  lastupload = now();
  Serial.println("end of init");
  measuretemp();
  smartDelay(5000);
  ThingspeakGSM();
  smartDelay(1000);
}

void loop()
{
  if (gps.speed.kmph() > SPEEDLIMIT)   // upload the GPS data only if the vehicle has moved inbetween uploads
    speedflag = true;
  if (gps.satellites.value() != 0)
  {
    digitalWrite(OKLED, HIGH);
    digitalWrite(NOTOKLED, LOW);
  }
  else
  {
    digitalWrite(OKLED, LOW);
    digitalWrite(NOTOKLED, HIGH);
  }

  if (now() - lastupload > uploadinterval && gps.satellites.value() != 0 && speedflag )  // set the microcontroller time every interval, only if the vehicle has moved
  {
    setthetime();
    ThingspeakGSM();
    lastupload = now();
    speedflag = false;
    smartDelay(1000);
  }

  if (now() - lasttemp > tempcheckinterval)
  {
    measuretemp();
    lasttemp = now();
    smartDelay(1000);
    if (temperature > TEMPERATURELIMIT1 && tempsmsflag1)
    {
      SendTextMessage(ALARM);
      tempsmsflag1 = false;
    }
    if (temperature > TEMPERATURELIMIT2 && tempsmsflag2)
    {
      SendTextMessage(ALARM);
      tempsmsflag2 = false;
    }
  }

  if (sendATcommand("", "+CLIP", 1000) == 1)  // if the module is called, "+CLIP" is in the serial communication
  {
    incoming = true;
  }
  //answer is 1 if sendATcommand detects +CLIP
  while (incoming)
  {
    uint32_t starttime = millis();
    for (int i = 0; i < 15; i++) {
      //read the incoming byte of the phone number:
      while (A6Serial.available() == 0)
      {
        delay (50);
      }
      //stores phone number
      received[i] = A6Serial.read();
    }
    A6Serial.flush();

    byte j = 0;
    //phone number comes after quotes (") so discard all bytes until find'em
    while (received[j] != '"')
      j++;
    j++;
    for (byte i = 0; i < 11; i++) {
      phone_number[i] = received[i + j];
    }
    int8_t answer2 = sendATcommand("", "ERROR", 1000);  //checks whether the caller hang up (gives an "ERROR" message in the output of the A6 module)
    calltime = 0;
    while (answer2 == 0 && calltime < maxtime)
    {
      calltime = millis() - starttime;
      answer2 = sendATcommand("", "ERROR", 1000);
    }
    SendTextMessage(NO_ALARM);
    incoming = false;
  }
}


void A6start()
{
  digitalWrite(A6ACTIONLED, HIGH);
  smartDelay(2000);
  answer = 0;
  // checks if the module is started
  answer = sendATcommand("AT", "OK", 2000);
  if (answer == 0)
  {
    // waits for an answer from the module
    while (answer == 0)
    { // send AT every two seconds and wait for the answer
      answer = sendATcommand("AT", "OK", 2000);
    }
  }
  answer = 0;
  answer1 = 0;
  smartDelay(1000);
  //answer = sendATcommand("AT+CREG?", ",1", 2000);
  answer = sendATcommand("AT+CREG?", "+CREG: 1,1", 2000);   // registrered to home network
  answer1 = sendATcommand("AT+CREG?", ",5", 2000);   // registrered to network and roaming
  if (answer == 0 || answer1 == 0)
  {
    // waits for an answer from the module
    while (answer == 0 && answer1 == 0)
    { // checks whether module is connected to network
      answer = sendATcommand("AT+CREG?", "+CREG: 1,1", 2000);  // registrered to home network
      answer1 = sendATcommand("AT+CREG?", ",5", 2000);  // registrered to network and roaming
    }
  }
  answer = 0;
  smartDelay(1000);
  answer = sendATcommand("AT+CLIP=1", "OK", 1000);
  if (answer == 0)
  {
    // waits for an answer from the module
    while (answer == 0)
    { // checks whether module is connected to network
      answer = sendATcommand("AT+CLIP=1", "OK", 2000);
    }
  }
  answer = 0;
  smartDelay(1000);
  answer = sendATcommand("AT+CMGD=1,4", "OK", 1000);
  if (answer == 0)
  {
    // waits for an answer from the module
    while (answer == 0)
    { // checks whether module is connected to network
      sendATcommand("AT+CMGD=1,4", "OK", 5000);
    }
  }
  answer = 0;
  smartDelay(1000);
  answer = sendATcommand("ATE0", "OK", 1000);
  if (answer == 0)
  {
    // waits for an answer from the module
    while (answer == 0)
    { // checks whether module is connected to network
      answer = sendATcommand("ATE0", "OK", 1000);
    }
  }
  digitalWrite(A6ACTIONLED, LOW);
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout)
{
  uint8_t x = 0,  answer = 0;
  char response[100];
  unsigned long previous;

  memset(response, '\0', 100);    // Initialice the string
  delay(100);
  while (A6Serial.available() > 0) A6Serial.read();   // Clean the input buffer
  A6Serial.println(ATcommand);    // Send the AT command
  x = 0;
  previous = millis();

  // this loop waits for the answer
  do {
    // if there are data in the UART input buffer, reads it and checks for the asnwer
    if (A6Serial.available() != 0) {
      response[x] = A6Serial.read();
      x++;
      // check if the desired answer is in the response of the module
      if (strstr(response, expected_answer) != NULL)
      {
        answer = 1;
      }
    }
    while (GPS_serial.available())
      gps.encode(GPS_serial.read());
    // Waits for the answer with time out and reads the GPS data
  } while ((answer == 0) && ((millis() - previous) < timeout));
  //DebugSerial.println(response);
  return answer;
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (GPS_serial.available())
      gps.encode(GPS_serial.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

void setthetime(void)
{
  int Year = gps.date.year();
  byte Month = gps.date.month();
  byte Day = gps.date.day();
  byte Hour = gps.time.hour();
  byte Minute = gps.time.minute();
  byte Second = gps.time.second();
  // Set Time from GPS data string
  setTime(Hour, Minute, Second, Day, Month, Year);  // set the time of the microcontroller to the UTC time from the GPS
}

///SendTextMessage()
///this function is to send a sms message
void SendTextMessage(int alarm)
{
  digitalWrite(A6ACTIONLED, HIGH);
  //DebugSerial.println("\nstart SMS subroutine");
  sendATcommand("AT+CMGF = 1\r", "OK", 2000); //Because we want to send the SMS in text mode
  //smartDelay(1000);
  //DebugSerial.println("AT+CMGS =+etcetera");
  A6Serial.print("AT+CMGS = +");    // include "+" in number
  for (int i = 0; i < 11; i++)
  {
    // Print phone number:
    A6Serial.print(phone_number[i]);
  }
  A6Serial.println("");
  sendATcommand("", " > ", 10000);

  if (alarm)
  {
    A6Serial.print("Attention! Temperature of GPS tracker Auris = ");
    A6Serial.print(temperature, 1);
    A6Serial.print(" C");
  }
  else
  {
    A6Serial.print("Current location : http : //www.google.com/maps?q=");
    A6Serial.print(gps.location.lat(), 6);
    A6Serial.print(",");
    A6Serial.print(gps.location.lng(), 6);
    A6Serial.print(". speed: ");
    A6Serial.print(gps.speed.kmph());
    A6Serial.print(". course: ");
    A6Serial.print(gps.course.deg());
    A6Serial.print(". module temperature: ");
    A6Serial.print(temperature);
  }
  A6Serial.print(". This SMS was sent to: +");
  for (int i = 0; i < 11; i++)
  {
    // Print phone number:
    A6Serial.print(phone_number[i]);
  }
  // include current time in SMS
  utc = now();              // read the time in the correct format to change via the TimeChangeRules
  local = CE.toLocal(utc);  // change UTC to current time in timezone
  A6Serial.print(". at ");
  if (hour(local) < 10) // add a zero if minute is under 10
    A6Serial.print("0");
  A6Serial.print(hour(local));
  A6Serial.print(":");
  if (minute(local) < 10) // add a zero if minute is under 10
    A6Serial.print("0");
  A6Serial.print(minute(local));
  // SMS end commands
  A6Serial.println(end_c);//the ASCII code of the ctrl+z is 26
  A6Serial.println();
  sendATcommand("", "OK", 10000);
  sendATcommand("AT", "OK", 1000);
  smartDelay(1000);
  digitalWrite(A6ACTIONLED, LOW);
}

void ThingspeakGSM(void)
{
  digitalWrite(A6ACTIONLED, HIGH);
  String host = "api.thingspeak.com";
  String write_api_key = "D1JMH8H99ZBWAVGB";

  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  //sendATcommand("AT+CGATT?", "OK", 20000);
  sendATcommand("AT+CGATT=1", "OK", 20000);
  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  sendATcommand("AT+CGDCONT=1,\"IP\",\"internet\"", "OK", 20000); //bring up wireless connection
  //sendATcommand("AT+CIPSTATUS", "", 10000);
  sendATcommand("AT+CGACT=1,1", "OK", 20000);
  sendATcommand("AT+CGDCONT?", "CGDCONT", 20000);
  //sendATcommand("AT+CIPSTATUS", "", 10000);
  //delay(1000);
  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  sendATcommand("AT+CIFSR", "OK", 20000); //get local IP adress
  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  sendATcommand("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80", "CONNECT OK", 25000); //start up the connection
  //sendATcommand("AT+CIPSTART=\"TCP\",\""+host+"\",80", "CONNECT OK", 25000); //start up the connection
  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  sendATcommand("AT+CIPSEND", ">", 10000); //begin send data to remote server
  smartDelay(500);
  A6Serial.print("GET /update?api_key=");
  A6Serial.print(write_api_key);
  A6Serial.print("&field1=");
  //kan dit zo, of moet er eerst een string van de variabelen vanuit de gps gemaakt worden?
  A6Serial.print(gps.location.lat(), 6);
  A6Serial.print("&field2=");
  A6Serial.print(gps.location.lng(), 6);
  A6Serial.print("&field3=");
  A6Serial.print(gps.speed.kmph());
  A6Serial.print("&field4=");
  A6Serial.print(gps.course.deg());
  A6Serial.print("&field5=");
  A6Serial.print(uploadcounter + 1);
  A6Serial.print("&field6=");
  A6Serial.println(temperature, 1);
  A6Serial.print("\r\n");

  //A6Serial.println("HTTP/1.1 HOST: api.thingspeak.com");
  //A6Serial.println("Connection: close");
  //A6Serial.println("Content-Type: application/json");
  A6Serial.println("");

  //DebugSerial.print("GET /update?api_key=");
  //DebugSerial.print(write_api_key);
  //DebugSerial.print("&field1=");
  //DebugSerial.print(gps.location.lat(), 6);
  //DebugSerial.print("&field2=");
  //DebugSerial.print(gps.location.lng(), 6);
  //DebugSerial.print("&field3=");
  //DebugSerial.print(gps.speed.kmph());
  //DebugSerial.print("&field4=");
  //DebugSerial.print(gps.course.deg());
  //DebugSerial.print("&field5=");
  //DebugSerial.println(uploadcounter+1);
  //DebugSerial.print("\r\n");
  Serial.println("");

  sendATcommand(end_c, "CIPRCV", 30000); //begin send data to remote server
  //A6Serial.println(end_c); //sending ctrlZ
  //unsigned long   entry = millis();
  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  //sendATcommand("AT+CIPCLOSE", "OK", 15000); //sending
  //sendATcommand("AT+CIPSTATUS", "OK", 10000);
  smartDelay(2000);
  Serial.println("-------------------------End------------------------------");
  //DebugSerial.println("wait for incoming call");
  digitalWrite(A6ACTIONLED, LOW);
  uploadcounter++;
}

void blinkLEDS(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      digitalWrite(10 + j, HIGH);
      delay(500);
    }

    for (int k = 0; k < 3; k++)
    {
      digitalWrite(10 + k, LOW);
      delay(500);
    }
  }
}

void measuretemp(void)
{
  //from: https://learn.adafruit.com/thermistor/using-a-thermistor
  uint8_t i;
  float average;
  digitalWrite(THERMISTOR_POWER, HIGH); // only measure when the pin is high
  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  temperature = steinhart;
  digitalWrite(THERMISTOR_POWER, LOW);
}
