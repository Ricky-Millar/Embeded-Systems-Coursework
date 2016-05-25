// receiver.pde
//
// Simple example of how to use VirtualWire to receive messages
// Implements a simplex (one-way) receiver with an Rx-B1 module
//
// See VirtualWire.h for detailed API docs
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2008 Mike McCauley
// $Id: receiver.pde,v 1.3 2009/03/30 00:07:24 mikem Exp $

#include <VirtualWire.h> 
int temp,tempR,pressure,errorsending,Error, ID, SelectedID = 99;
unsigned int pressR;  
uint8_t buf[VW_MAX_MESSAGE_LEN], buflen = VW_MAX_MESSAGE_LEN, tempC[4];

void setup()
{
  Serial.begin(9600);	// Debugging only
  Serial.println("Setup complete ");
  Serial.println("-------------------------");
  vw_setup(2000);	 // Bits per sec
  vw_set_rx_pin(A0);
  vw_rx_start();       // Start the receiver PLL running
}



bool SystemIDCheck(){
  if(ID == SelectedID){
  return 1;}
  else{
  return 0;}
  };

void loop ()
{

  if (vw_get_message(buf, &buflen))  {
    int i;
    char data[25];
    char num[20];
    digitalWrite(13, true);
    
    for (i = 0; i < buflen; i++) {
      data[i] = buf[i];
    }
    String dataCopy=data; //To display data later once "data" is deleted
    char * strtokIndx;
    strtokIndx = strtok(data, ",");
    Error = atoi(strtokIndx);

   
    if (Error != 2) {
    Serial.println(Error ? "Sensor 1 Broken" : "Sensor 0 Broken") ;    
    }

    
    strtokIndx = strtok(NULL, ",");
    ID = atoi (strtokIndx);
    strtokIndx = strtok(NULL, ",");
    temp = atoi (strtokIndx);
    strtokIndx = strtok(NULL, ",");
    tempR = atoi (strtokIndx);
    strtokIndx = strtok(NULL, ",");
    pressure = atoi (strtokIndx);
    strtokIndx = strtok(NULL, ",");
    pressR = atoi (strtokIndx);
    strtokIndx = strtok(NULL, ",");
    errorsending = atoi (strtokIndx);
    digitalWrite(13, false);
    
    
    int errorrec = ID + temp  + pressure + Error ;    
    int error = errorrec % 2;
    if (error == 0) errorrec = 0;
    else errorrec = 1;

    
    if (errorrec == errorsending && SystemIDCheck()) {
      Serial.print("System ID:   ");
      Serial.println(ID);
      Serial.print("Temperature: ");
      Serial.print(temp);
      Serial.print(".");
      Serial.print(tempR);
      Serial.println (" C"); 
      Serial.print("Pressure:    ");
      Serial.print(pressure);
      Serial.print(".");
      Serial.print(pressR);
      Serial.println (" mbar"); 
      Serial.print("-------------------------       ");
      Serial.println(dataCopy);
    }
    else {
      Serial.print(" Sending Error");
      Serial.println(errorrec);
    }
 }
}
