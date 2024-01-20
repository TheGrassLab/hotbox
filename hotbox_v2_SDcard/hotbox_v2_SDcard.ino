//variables to change
  int Tset = 0;
  int TempInc = 1; //temperature increase (Celsius)
  int TimeInc = 1; //time interval (minutes) for temperature increases
  int TempMax = 60; // max temperature (Celsius)
  int TimeTempMax = 30; //the number of minutes tempearture is held (minutes)
    //vpd variables
  float VPDMax = 3.94; // max VPD (kPa)
  int vpd_set = 2; //(kPa)

//Import Libaries
  #include <Arduino.h>
  #include <Wire.h>
  #include <SPI.h>
  #include "Adafruit_SHT31.h"
  #include <Adafruit_SSD1306.h>
  #include <Adafruit_MAX31856.h>

//time interval variables
  unsigned long previousMillis = 0; // For storing previous timestep
  int counter = 1;
  int INTERVAL = 1000;        // speed the sketch runs (ms)

//for thermistors and thermocouples
  Adafruit_MAX31856 TCleaf1 = Adafruit_MAX31856(5, 8, 10, 9);//define pins for TC1
  Adafruit_MAX31856 TCleaf2 = Adafruit_MAX31856(4, 8, 10, 9);//defiune pins for TC2 (only difference from above is CS pin)
  byte tsoil1_pin = 0;//ananlog pin thermistor is connected
  byte tsoil2_pin = 1;//ananlog pin thermistor is connected

//for SHT31
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  bool enableHeater = false;

//for SD module
  #include <SD.h>
  File myFile;
  #define FILE_NAME "HOTBOXv2.csv"
  byte CS_pin = 7;

//for SSD1306 I2C OLED Display
  Adafruit_SSD1306 display(128, 64, &Wire);

//various variables
  byte heatPin1 = 0;
  byte heatPin2 = 1;
  byte pumpPin = 3;
  byte heatswitch = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(10);  

  Serial.println("Beginning Setup Routine");
  //SHT31 test
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
      Serial.println(F("Couldn't find SHT31"));
    }

  //OLED setup
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C )) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
        //set additional display parameters
      display.setTextSize(1);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
//SD test
    Serial.print("\nInitializing SD card...");
    if (!SD.begin(CS_pin)) {
      Serial.println(F("SD card initialization failed!"));
    }else{
      Serial.println(F("card initialized"));
    }

    myFile = SD.open(FILE_NAME, FILE_WRITE);
    myFile.print("Time, Tleaf_1, Tleaf_2, Tsoil_1, Tsoil_2, Tair, VPDair"); myFile.println(""); myFile.close();

  //MAX31856 - thermocouple setup for leaf 1
    if (!TCleaf1.begin()) {
        Serial.println("Could not initialize thermocouple.");
        while (1) delay(10);
      }

      TCleaf1.setThermocoupleType(MAX31856_TCTYPE_T);
      TCleaf1.setConversionMode(MAX31856_CONTINUOUS);

  //MAX31856 - thermocouple setup for leaf 1
    if (!TCleaf2.begin()) {
        Serial.println("Could not initialize thermocouple.");
        while (1) delay(10);
      }

      TCleaf2.setThermocoupleType(MAX31856_TCTYPE_T);
      TCleaf2.setConversionMode(MAX31856_CONTINUOUS);


  //pin assignments
    pinMode(heatPin1, OUTPUT);
    pinMode(heatPin2, OUTPUT);
    digitalWrite(heatPin1, HIGH);
    digitalWrite(heatPin2, HIGH);
    pinMode(pumpPin, OUTPUT);
    digitalWrite(pumpPin, LOW);
    delay(10);

  Serial.println("Done with setup");
}


//additional functions
void Tpid(float Td, byte heatPin1, byte heatPin2, float Tdmin, float Tdmax){
  int Td_last = 0;

  if (Td < Tdmin * 4){
    digitalWrite(heatPin1, LOW);
    digitalWrite(heatPin2, LOW);
  }
  else if(Td > Tdmin*4 & Td < Tdmin){
    digitalWrite(heatPin1, LOW);
    digitalWrite(heatPin2, LOW);
  }
  else if(Td > Tdmin and Td < Tdmax){
    digitalWrite(heatPin1, LOW);
    digitalWrite(heatPin2, LOW);
  }
  else if(Td > Tdmax){
    digitalWrite(heatPin1, HIGH);
    digitalWrite(heatPin2, HIGH);
  }

  Td_last <- Td_last - Td;

}


void VPDpid(float vpd, float vpd_set, int pumpPin){
    float vpdd = vpd_set - vpd;
  
  if (vpdd < -0.2){
    digitalWrite(pumpPin, HIGH);
    //Serial.println(F("Humidity, ON, Humidity Control 5 (sec)"));
  }
  else if (vpdd > -0.2  & vpdd < 0){
    digitalWrite(pumpPin, HIGH);
    //Serial.println(F("Humidity, ON, Humidity Control 1 (sec)."));
  }
  else if(vpdd >= 0){
    digitalWrite(pumpPin, LOW);
    //Serial.println(F("Humidity, OFF, Humidity Control 1 (sec)."));
  }

}

float Tread(byte xpin){
      float Traw = analogRead(xpin);
      float logR2 = log(10000 * (1023.0 / Traw - 1.0));
      float T = ((1.0 / (1.009249522e-03 + 2.378405444e-04*logR2 + 2.019202697e-07*logR2*logR2*logR2))) - 273.15;
      return T;
}

void loop() {
  byte Tref = 20;
  float TimeRm;
  float Tair;
  float vpd = 0;
  float vpd_sat;
  char* units[] = {"sec"};

  //check if it is time to run the code
  unsigned long currentMillis = millis(); 

  if ((currentMillis - previousMillis) >= INTERVAL) {
      // ..If yes, save current time.  Then update the LED pin and LED state.
    previousMillis = currentMillis;  //reset previous time for interval

    Tair = sht31.readTemperature();
    vpd = sht31.readHumidity();//read humidity from sht31
    vpd_sat = 0.611*exp((17.502*Tair)/(Tair+240.97));
    vpd = vpd_sat - (vpd_sat*(vpd/100));

    float Td = Tair - Tset;

    //set initial Tset
      if (Tset == 0){ // set at 0 initially
        Tset = Tair+1; // set the temperature to 1 degree above the integer of room temperature
      }

        //Changes Tset and updates time arrays if conditions are met
      if(counter >= (TimeInc*60)){ 
        counter = 1;
        Tset += TempInc;
        Serial.println(); Serial.println();Serial.println();
        Serial.print("New Tset is:  ");Serial.println(Tset);
        Serial.println(); Serial.println();Serial.println();
      }
    //determine upper/lower limits of temperature control
    float Tdmin = -1*(1-(Tset - Tref)/45);
    float Tdmax = (Tset - Tref)/45;

    if(Tset < TempMax){
      VPDpid(vpd, vpd_set, pumpPin);
      Tpid(Td, heatPin1, heatPin2, Tdmin, Tdmax);
      TimeRm = TimeInc*60 - counter;
      //sprintf(Tsetdisp,"%3d", Tset);
      }
      else if (Tset == TempMax){
      VPDpid(vpd, vpd_set, pumpPin);
      Tpid(Td, heatPin1, heatPin2, Tdmin, Tdmax);
      TimeRm = TimeInc*60 - counter;
      //sprintf(Tsetdisp,"%d", Tset);
      }
      else {
      digitalWrite(heatPin1, LOW);
      digitalWrite(heatPin2, LOW);
      digitalWrite(pumpPin,LOW);
      TimeRm = 0;
      //sprintf(Tsetdisp,"%s", "done");
    }

    // measure and record temperature data from chamber
      //TCleaf1.triggerOneShot();
      delay(10);
      float tleaf1 = TCleaf1.readThermocoupleTemperature();
      //TCleaf2.triggerOneShot();
      delay(10);
      float tleaf2 = TCleaf2.readThermocoupleTemperature();
      float tsoil1 = Tread(tsoil1_pin);
      float tsoil2 = Tread(tsoil2_pin);

    //print data to serial monitor
      Serial.print(F("Tset:"));Serial.println(Tset);
      Serial.print(F("T_air: ")); Serial.print(Tair);Serial.print(F("   RH: ")); Serial.println(vpd);
      Serial.print(F("Tleaf1: ")); Serial.print(tleaf1);Serial.print(F("   Tleaf2:")); Serial.println(tleaf2);
      Serial.print(F("Tsoil1: ")); Serial.print(tsoil1);Serial.print(F("   Tsoil2:")); Serial.println(tsoil2);
      Serial.print("Heater: ");Serial.println(heatswitch);
      Serial.print("counter:  ");Serial.print(counter);Serial.print("  nexttime:   ");Serial.println(TimeRm);

    //get state of heatpin to display
      if(digitalRead(heatPin1) == HIGH){
        heatswitch = 0;
      }else{
        heatswitch = 1;
      }

    int TiTotalRm = (TempMax - Tset) * (TimeInc*60) + (TimeTempMax*60);
    if(TiTotalRm >60){
      TiTotalRm = TiTotalRm/60;
      units[0] = "m";
      units[1] = "i";
      units[2] = "n";
    }

    display.clearDisplay(); display.setCursor(0, 0);display.write("T/Tset:   ");display.print(Tair); display.write("| "); display.print(Tset); display.println();
    display.write("vpd/vset:  "); display.print(vpd);display.write("| "); display.print(vpd_set,1); display.println();
    display.write("Tleaf 1/2: "); display.print(tleaf1,1);display.write("| "); display.print(tleaf2,1); display.println();
    display.println();
    display.write("heater: ");display.print(heatswitch); display.println();
    display.write("TimeRm:");display.println(TimeRm,0);
    display.write("TiTotalRm:");display.print(TiTotalRm);display.write(" ");display.write(units[0]);display.write(units[1]);display.write(units[2]);display.display();
    
    // float array[] = {float(currentMillis), tleaf1, tleaf2, tsoil1, tsoil2, Tair, vpd};
    // myFile = SD.open(FILE_NAME, FILE_WRITE);
      
    //   for (int x = 0 ; x < 7 ; x++){
    //     if (! isnan(array[x])) {  // check if 'is not a number
    //       char ychar[12];
    //       dtostrf(array[x], 6, 3, ychar);
    //       if (myFile) {     //so long as the file is openable
    //         myFile.write(ychar); myFile.write(","); //writes to file in CSV format
    //       }else{     // return error message
    //         Serial.print(x); Serial.println(F("File not open!"));
    //       }
    //       memset(ychar, 0, 12);
    //     }
    //   }
    //     myFile.println(""); myFile.close();

    
      counter = counter+1;
    }
 
}

