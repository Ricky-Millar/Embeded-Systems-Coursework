#include <VirtualWire.h>
#include <MyWireLib.h>
MyWireLib Sens[2];
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPIN 13
#define rfTransmitPower 2
#define rfReceivePin 0
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
///////FSM, Time & other definitions 
int state1 = S0, state2 = S0, ID = 99, BMP1read, BMP2read, BMPError;
float oldT0 = 0, oldT1 = 0, T0derivative, T1derivative;
unsigned int incomingByte = 0; 
unsigned long timestampA = 0, oldtime = 0, newtime = 0, timestampB = 0, timestampSEND = 0;
///////Transmision definitions
unsigned  int data = 0, data0 = 0, data1 = 0;  // variable used to store received data
const  unsigned  int allowedError = 1;   //lower threshold value
unsigned long timestamp, timestampNOISE;
unsigned char sentOK;
/////BMPcomunication definitions
int16_t  ac1[2], ac2[2], ac3[2], b1[2], b2[2], mb[2], mc[2], md[2]; // Store sensor PROM values from BMP180
uint16_t ac4[2], ac5[2], ac6[2];                     // Store sensor PROM values from BMP180
const uint8_t oss = 3;                      // Set oversampling setting
const uint8_t osd = 26;                     // with corresponding oversampling delay
float T[2], P[2];                                 // Set global variables for temperature and pressure
byte Ptobesend[2] = {0xf4, 0xf4};
byte Ttobesend[2] = {0xf4, 0x2e};

//NOISE DETECTION
bool NoiseDetection() {
  digitalWrite(rfTransmitPower, LOW);
  data = analogRead(rfReceivePin);    //listen for data on Analog pin
  timestampNOISE = millis();
  sentOK = 1;
  data0 = data;
  while ((millis() - timestampNOISE) < 3) // transition to state 2
  {
    data1 = data;
    data = (data1 + analogRead(rfReceivePin)) / 2;   //listen for data on Analog pin 0
    if (abs(data - data1) < allowedError ) {
      sentOK = 0 ;
      Serial.println("                                               ---Busy channel---");
      Serial.println("--------------------");
    }
    else
      sentOK = 1;
  }
  return sentOK;
  delay(random(5));
}

void init_SENSOR(int sensnr)
{
  ac1[sensnr] = Sens[sensnr].Get16bitFromRegister(0xAA);
  ac2[sensnr] = Sens[sensnr].Get16bitFromRegister(0xAC);
  ac3[sensnr] = Sens[sensnr].Get16bitFromRegister(0xAE);
  ac4[sensnr] = Sens[sensnr].Get16bitFromRegister(0xB0);
  ac5[sensnr] = Sens[sensnr].Get16bitFromRegister(0xB2);
  ac6[sensnr] = Sens[sensnr].Get16bitFromRegister(0xB4);
  b1[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xB6);
  b2[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xB8);
  mb[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xBA);
  mc[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xBC);
  md[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xBE);

  Serial.println("");
  Serial.print("Sensor ");
  Serial.print(sensnr);
  Serial.println(" calibration data:");
  Serial.print(F("AC1 = ")); Serial.println(ac1[sensnr]);
  Serial.print(F("AC2 = ")); Serial.println(ac2[sensnr]);
  Serial.print(F("AC3 = ")); Serial.println(ac3[sensnr]);
  Serial.print(F("AC4 = ")); Serial.println(ac4[sensnr]);
  Serial.print(F("AC5 = ")); Serial.println(ac5[sensnr]);
  Serial.print(F("AC6 = ")); Serial.println(ac6[sensnr]);
  Serial.print(F("B1 = "));  Serial.println(b1[sensnr]);
  Serial.print(F("B2 = "));  Serial.println(b2[sensnr]);
  Serial.print(F("MB = "));  Serial.println(mb[sensnr]);
  Serial.print(F("MC = "));  Serial.println(mc[sensnr]);
  Serial.print(F("MD = "));  Serial.println(md[sensnr]);
  Serial.println("");
}

void setup()
{
  vw_setup(2000);
  vw_set_tx_pin(3) ;   //RF Transmitter pin = digital pin 3
  vw_set_rx_pin(A0)  ;  //RF Receiver pin = Analog pin 0
  pinMode(rfTransmitPower,  OUTPUT);
  Serial.begin(9600);
  sei();
  pinMode(LEDPIN, OUTPUT);


  Serial.begin(9600);

  //declare pins
  Sens[0].SCLpinI = 4;
  Sens[0].SDApinI = 5;
  Sens[1].SCLpinI = 6;
  Sens[1].SDApinI = 7;
  Sens[0].Soss = oss;

  Sens[1].Soss = oss;

  //> for test
  Serial.print("SCLPIN for sensor 0: "); Serial.println(Sens[0].SCLpinI);
  Serial.print("SDAPIN for sensor 0: "); Serial.println(Sens[0].SDApinI);
  Serial.print("SCLPIN for sensor 1: "); Serial.println(Sens[1].SCLpinI);
  Serial.print("SDAPIN for sensor 1: "); Serial.println(Sens[1].SDApinI);
  //< for test
  delay(500);

  Sens[0].InitWire();
  Sens[1].InitWire();

  init_SENSOR(0);
  init_SENSOR(1);
}



void loop () {
/////////////////Finite State Machine to read/compute BMP values
  int32_t y1, y2, b5, AT, x1, x2, x3, b3, b6, p, UP;
  uint32_t b4, b7;
  switch (state1) {
    case S0:
      Sens[0].sendbytes(Ttobesend, 2);
      timestampA = millis();
      state1 = S1;
      break;

    case S1:
      if (timestampA + 5 <= millis()) {
        state1 = S2;
        break;
      }
      else {
        state1 = S1;
        break;
      }

    case S2:
      AT = Sens[0].Get16bitFromRegister(0xf6);
      y1 = (AT - (int32_t)ac6[0]) * (int32_t)ac5[0] >> 15;
      y2 = ((int32_t)mc[0] << 11) / (y1 + (int32_t)md[0]);
      b5 = y1 + y2;
      T[0]  = (b5 + 8) >> 4;
      T[0] = T[0] / 10.0;                           // Temperature in celsius
      state1 = S3;
      break;

    case S3:
      Sens[0].sendbytes(Ptobesend, 2);
      timestampA = millis();
      state1 = S4;
      break;

    case S4:
      if (timestampA + osd <= millis()) {
        state1 = S5;
        break;
      }
      else {
        state1 = S4;
        break;
      }

    case S5:
      UP = Sens[0].Get24bitFromRegister(0xf6);
      b6 = b5 - 4000;
      x1 = (b2[0] * (b6 * b6 >> 12)) >> 11;
      x2 = ac2[0] * b6 >> 11;
      x3 = x1 + x2;
      b3 = (((ac1[0] * 4 + x3) << oss) + 2) >> 2;
      x1 = ac3[0] * b6 >> 13;
      x2 = (b1[0] * (b6 * b6 >> 12)) >> 16;
      x3 = ((x1 + x2) + 2) >> 2;
      b4 = (ac4[0] * (uint32_t)(x3 + 32768)) >> 15;
      b7 = ((uint32_t)UP - b3) * (50000 >> oss);
      if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
      }
      else {
        p = (b7 / b4) << 1;  
      }
      x1 = (p >> 8) * (p >> 8);
      x1 = (x1 * 3038) >> 16;
      x2 = (-7357 * p) >> 16;
      P[0] = (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; // Return pressure in mbar
      BMP1read = 1;
      state1 = S0;
      break;
  }


  int32_t w1, w2, g5, BT, z1, z2, z3, g3, g6, n, H;
  uint32_t g4, g7;

  switch (state2) {
    case S0:
      Sens[1].sendbytes(Ttobesend, 2);
      timestampB = millis();
      state2 = S1;
      break;

    case S1:
      if (timestampB + 5 <= millis()) {
        state2 = S2;
        break;
      }
      else {
        state2 = S1;
        break;
      }

    case S2:
      BT = Sens[1].Get16bitFromRegister(0xf6);
      w1 = (BT - (int32_t)ac6[1]) * (int32_t)ac5[1] >> 15;
      w2 = ((int32_t)mc[1] << 11) / (w1 + (int32_t)md[1]);
      g5 = w1 + w2;
      T[1]  = (g5 + 8) >> 4;
      T[1] = T[1] / 10.0;                           // Temperature in celsius
      state2 = S3;
      break;

    case S3:
      Sens[1].sendbytes(Ptobesend, 2);
      timestampB = millis();
      state2 = S4;
      break;

    case S4:
      if (timestampB + 26 <= millis()) {
        state2 = S5;
        break;
      }
      else {
        state2 = S4;
        break;
      }

    case S5:
      H = Sens[1].Get24bitFromRegister(0xf6);
      g6 = g5 - 4000;
      z1 = (b2[1] * (g6 * g6 >> 12)) >> 11;
      z2 = ac2[1] * g6 >> 11;
      z3 = z1 + z2;
      g3 = (((ac1[1] * 4 + z3) << oss) + 2) >> 2;
      z1 = ac3[1] * g6 >> 13;
      z2 = (b1[1] * (g6 * g6 >> 12)) >> 16;
      z3 = ((z1 + z2) + 2) >> 2;
      g4 = (ac4[1] * (uint32_t)(z3 + 32768)) >> 15;
      g7 = ((uint32_t)H - g3) * (50000 >> oss);
      if (g7 < 0x80000000) {
        n = (g7 << 1) / g4;
      }
      else {
        n = (g7 / g4) << 1;  // or n = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
      }
      z1 = (n >> 8) * (n >> 8);
      z1 = (x1 * 3038) >> 16;
      z2 = (-7357 * n) >> 16;
      P[1] = (n + ((z1 + z2 + 3791) >> 4)) / 100.0f; // Return pressure in mbar
      BMP2read = 1;
      state2 = S0;
      break;
  }
///////////BMP ERROR DETECTION
  BMPError = 2; // 2 is the ok state with no error

  float averageT, averageP;
  averageT = (T[0] + T[1]) / 2;
  averageP = (P[0] + P[1]) / 2;

if (oldtime + 500 <= millis()) { //Takes the derivitive of the Temperiture value
    oldtime = newtime;
    newtime = millis();
    float interval = newtime - oldtime;
    T0derivative = (T[0] - oldT0) / interval;
    int oldT0 = T[0];
    T1derivative = (T[1] - oldT1) / interval;
    int oldT1 = T[1] ;
  }


if (-10 > T[0] > 50 || T0derivative > 0.3  ) { //Derivative value was chosen through experimentation.
    averageT = T[1];
    averageP = P[1];
    BMPError = 0;
    Serial.println ("BMP error 0");
  }
if (-10 > T[1] > 50 || T1derivative > 0.3 ) {
    averageT = T[0];
    averageP = P[0];
    BMPError = 1;
    Serial.println ("BMP error 1");
  }
  
if ( -5 > T[0]-T[1] || T[0]-T[1] > 5) {
      Serial.println("ERROR:Unknown sensor giving false value");
    }


  //DEBUG PRINTING
  
  Serial.print("Temperature: ");
  Serial.print(averageT);
  Serial.println("C");
  Serial.print("Pressure:    ");
  Serial.print(averageP);
  Serial.println("mbar");
  Serial.println("--------------------");

////////////////////////////////Start of Transmission///////////////////////////////////
  char SendData[36];
  char IDarray[10];
  char BMParray [2];
  char IncomingData[34];

  if (BMP1read + BMP2read == 2 && NoiseDetection()) { //Only transmit if there are no other transmissions detected and both bmps have new data
    BMP1read = 0, BMP2read = 0; 
    memset(SendData, 0, 32); // Empties Send data array
    sprintf(BMParray, "%1d" , BMPError);
    strcat(SendData, BMParray);
    strcat(SendData, ",");

    /////Send ID////////////
    sprintf(IDarray, "%03d" , ID);//adds following 0's to keep the message length consistent
    strcat(SendData, IDarray);// Add system ID to start of Msg array
    strcat(SendData, ","); // Add a comma next in the Msg array

    /////SendTemp//////////
    int Wholetemp = averageT ;
    int Remaindertemp = (averageT - Wholetemp) * 1000;
    char Wholetemparry[10];
    char Remaindertemparry[10];
    sprintf(Wholetemparry, "%03d" , Wholetemp);
    strcat(SendData, Wholetemparry); 
    strcat(SendData, ",");
    sprintf(Remaindertemparry, "%03d" ,  Remaindertemp);
    strcat(SendData, Remaindertemparry); 
    strcat(SendData, ","); 

    /////SendPressure//////////Sends Float as 2 intigers for the intiger value + the remainder. floats could be sent but this made life easier.
    int Wholepress = averageP ;
    int Remainderpress = (averageP - Wholepress) * 1000;
    char Wholepressarry[10];
    sprintf(Wholepressarry, "%04d" , Wholepress);
    strcat(SendData, Wholepressarry);
    strcat(SendData, ",");
    char Remainderpressarry[10];
    sprintf(Remainderpressarry, "%03d" , Remainderpress);
    strcat(SendData, Remainderpressarry);
    strcat(SendData, ",");

    /////SendErrorcheck//////////
    int intaverageT = averageT;
    int intaverageP = averageP;
    int Errorcheck = ID + BMPError + intaverageT + intaverageP ;
    char errorsending[2];
    int error = Errorcheck % 2;
    if (error == 0) Errorcheck = 0;
    else Errorcheck = 1;
    sprintf(errorsending, "%02d" , Errorcheck);
    strcat(SendData, errorsending);
    ////Transmission/////
   //vw_rx_start();
    digitalWrite(rfTransmitPower, HIGH);      //power up the transmitter then wait for 5ms before sending
    timestampSEND = millis();
    if (timestampSEND + 5 <= millis()) {
    vw_send(((uint8_t*) SendData), strlen(SendData));
    Serial.println("                                       - - - - - Package Sent! - - - - - -");
    Serial.print("--------------------                       ");
    Serial.println(SendData);
  }}


  ///This code was meant for checking the data was correctly transmitted
  ///Bus as found in the tutorial session virtualwire would not correctly send and recieve at the same time
  /// this code wanted to recieve the sent value and compare it to itself later but it never recieved anything correctly
/*
  uint8_t data[50];
  uint8_t buflen = 50;
   while (vw_tx_active() == 1) {     // transition to state 2
  vw_get_message(data, &buflen);
  /*Serial.println("-----------");
    Serial.print("Sent:");
    Serial.println(SendData);
    Serial.print("Recived:");
    int i;
    for(i=0; i<buflen; i++){
    IncomingData[i] = char(data[i]);
    Serial.print( IncomingData[i]);
    }
  */
}
