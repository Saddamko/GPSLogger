
#include <SoftwareSerial.h>
#include <Wire.h> //I2C library
#include <TinyGPS++.h>

/*
The TinyGPS++ Object Model

The main TinyGPS++ object contains several core sub-objects:

    location – the latest position fix
    date – the latest date fix (UT)
    time – the latest time fix (UT)
    speed – current ground speed
    course – current ground course
    altitude – latest altitude fix
    satellites – the number of visible, participating satellites
    hdop – horizontal diminution of precision
*/

/*
 
    charsProcessed() – the total number of characters received by the object
    sentencesWithFix() – the number of $GPRMC or $GPGGA sentences that had a fix
    failedChecksum() – the number of sentences of all types that failed the checksum test
    passedChecksum() – the number of sentences of all types that passed the checksum test

*/


/* This sample code demonstrates the normal use of a TinyGPSPlus object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 2(rx) and 3(tx).
Summary:
This software displays information from an EM 406A GPS sensor on Phi-1 shield's LCD screen. It also logs the GPS information to the onboard EEPROM of the Phi-1 shield. 

Sample GPS data:
Lat/Long(10^-5 deg): 45xxxxx, -94xxxxx
Date(ddmmyy): 281210 Time(hhmmsscc): 134000
Alt(cm): 31750 Speed(mph): 1.81

List of functions:
* Menu gives you several choices:
* Send to PC: sends recorded GPS information to PC via the USB connection.
Two modes are available:
Verbose mode generates information as shown above.
Non-verbose mode sends only the column labels followed by tab-separated data, ideal for spreasheet programs to import. You may copy and paste.
* Erase EEPROM: erases the EEPROM
* Record: records GPS information, lattitude, longitude, altitude, date, time, speed to EEPROM
* Display: displays the GPS coordinates without recording
* Parameters: allows the user to adjust parameters such as period between two consecutive recordings,, PC data format, to start recording at which data entry and end at which entry.
* Up and down cycle through the menu options.
* Left, right, B act as confirm or enter.
* A escapes and only will preserve the latest value.
*/

#define EEPROM_size 32768UL // This is the maximal bytes on your eeprom, depending on the chip you use. I'm using 24LC256, with 256KiloBits so 32KiloBytes=32768 bytes.
#define n_menu_items 5
#define menu_PC 0     //'Pp'
#define menu_erase 1  //'Ee'
#define menu_record 2 //'Rr'
#define menu_dump 3   //'Dd'
#define menu_para 4
#define menu_KML 5    //'Kk'
#define menu_GPX 6    //'Gg'
#define menu_none 7
#define GPS_Tx 3
#define GPS_Rx 2

// Global Variables
int command = menu_none;       // This is the command char, in ascii form, sent from the serial port
boolean GotCommand=false;
boolean recording=false; // This indicates whether the program is recording the GPS info. If it is recording, a symbol "a chip" appears on the bottom right corner of the LCD.
boolean verbose=false; // This indicates the output to PC format. Verbose is for a human to read with "Lat/long:" and such. While not in verbose mode, it transfers faster and is more compatible with a spreadsheet program.
unsigned long period=5; // This is the period in seconds between two consecutive GPS recordings.
unsigned long pointer=0; // This pointer points to the next empty EEPROM byte for storing or next unread EEPROM byte for reading.
unsigned long lower_limit=0; // This is the lower limit to the pointer value. Save mode startssaving from this EEPROM address. If you use less than 4MBit EEPROM, there is no problem. If not, the limits can go beyond the limit of the signed integer taken by the input function.
unsigned long upper_limit=EEPROM_size; // This is the upper limit to the pointer value. Once the pointer is equal to this value, save mode will stop saving and quits to menu.
unsigned long start;

TinyGPSPlus gps;
SoftwareSerial ss(GPS_Rx, GPS_Tx);


void setup()
{
  
 Wire.begin(); // initialize wire
 Serial.begin(9600);
 ss.begin(9600);

 Serial.println("");
 Serial.print("Library version: ");
 Serial.println(TinyGPSPlus::libraryVersion());
 Serial.println("P - Send to PC");
 Serial.println("K - Send to PC in KML format");
 Serial.println("D - HEX dump of EEPROM memory");
 Serial.println("E - Erase EEPROM data");
 Serial.println("R - Start recording of GPS data");
 Serial.println("G - Send to PC in GPX format");
}

void loop()
{
  do_menu();
  
  if ((millis()> 180000)and (GotCommand==false))
  {
//    Serial.println ("No commands is received, start recording!");
    _record();
  };
}

void GPS_to_EEPROM(unsigned long *pointer)
/*
The function assumes that the TinyGPSPlus object gps is already initialized and ready to send data.
It also assumes that the caller feeds the GPS and checks the pointer so the pointed address will not exceed the address space of the EEPROM, or cross page boundaries while writing.
*/
{
  
  double spdf;
  unsigned long spd;
  float lat, lon;
  long alt;
  unsigned long age, dat, tim;
  float buf[4];

  lat=gps.location.lat();
  lon=gps.location.lng();

  
  age=gps.location.age();
  alt=gps.altitude.meters();
  spdf=gps.speed.kmph();
  dat=gps.date.value();
  tim=gps.time.value();

  spd=spdf*100;
  buf[0]=lat;
  buf[1]=lon;
  buf[2]=((alt/10)<<16)|((dat/10000)<<11)|(((dat%10000)/100)<<7)|(dat%100); // Altitude only takes 16 bit. It is in the unit of 0.1m instead 
  //of 1cm, which is not necessary in accuracy. date(when expressed in binary 5bit date-4bit month-7bit year, takes no more than 16 bit. Combine them together.
  buf[3]=(spd<<17)|((tim/1000000)*3600+((tim%1000000)/10000)*60+(tim%10000)/100); // Speed, expressed in 1/100 mph, takes less than 15 bits. 
  //Compact hhmmsscc into seconds since 00:00 and lose the 1/100 seconds. This is 17 bit long.
  i2c_eeprom_write_page(0x50, (unsigned int) (*pointer), (byte*) buf, 16); // Store 16 bytes of data at location of the pointer.
  (*pointer)=(*pointer)+16; // Increment pointer.
  delay(5); // Make sure the data is written to the EEPROM.
}

boolean EEPROM_to_GPS(float *lat, float *lon, long *alt, unsigned long *tim, unsigned long *dat, double *spdf, unsigned long *pointer)
/*
This function reads one EEPROM entry, if it's non zero, parse it into long integer forms (double precision for speed), and returns true.
If the EEPROM entry is empty, return false.
To stay isolated from the main program, this function doesn't check if the pointer will be beyond the EEPROM size, which is left to the caller to do.
*/
{
  float buf[4];
  i2c_eeprom_read_buffer (0x50, (unsigned int) (*pointer), (byte*) buf, 16);
  if ((buf[0]==0)&&(buf[1]==0)&&(buf[2]==0)&&(buf[3]==0)) return false;
  *lat=(float)buf[0];
  *lon=(float)buf[1];
  *dat=(long)buf[2]&0xFFFF;
  *dat=(*dat>>11)*10000+((*dat>>7)&15)*100+(*dat&127); //Process data to turn into 12/27/10 form
  *alt=10*((long)buf[2]>>16);
  *tim=((long)buf[3]&0x1FFFF);
  *tim=(*tim/3600)*1000000+((*tim%3600)/60)*10000+(*tim%60)*100; //Process time to turn into 12:59:0000 form
  *spdf=((double)(((long)buf[3])>>17))/100.0;
  (*pointer)=(*pointer)+16; // Increment pointer by 16 bytes.
  return true;
}

void do_menu()
{

  int temp1=AskMode ();

  switch (temp1)
  {
    case menu_dump:
    _print_EEPROM_dump();
    break;

    case menu_erase:
    _erase();
    break;

    case menu_record:
    _record();
    break;

    case menu_para:
    _parameters();
    break;

    case menu_PC:
    _send_to_PC();
    break;
    
    case menu_KML:
    _send_to_PC_KML();
    break; 
    
    case menu_GPX:
    _send_to_PC_GPX();
    break; 
    }
  
    if (temp1 != menu_none) {
    Serial.println("Command is completed");
    command = menu_none;
  }
}

 int AskMode ()
 {
    if (Serial.available()) {      // Look for char in serial que and process if found
      command = Serial.read();

      if (command == (byte)'P' || command == (byte)'p') {           //If command = "Pp" Send GPS to PC
        command = menu_PC;
        GotCommand=true;
      }
      else if (command == (byte)'E' || command == (byte)'e') {      //If command = "Ee" Erase EEPROM
        command = menu_erase;
        GotCommand=true;
      }
      else if (command == (byte)'D' || command == (byte)'d') {      //If command = "Dd" Dump EEPROM
        command = menu_dump;
        GotCommand=true;
      }
      else if (command == (byte)'R' || command == (byte)'r') {      //If command = "Rr"  Record GPS
        command = menu_record;
        GotCommand=true;
      } 
       else if (command == (byte)'G' || command == (byte)'g') {      //If command = "Gg" send GPX
        command = menu_GPX;
        GotCommand=true;
      }      
      else if (command == (byte)'K' || command == (byte)'k') {      //If command = "Kk"  send KML
        command = menu_KML;
        GotCommand=true;
      }
      else {
      command = menu_none;                 // none command 
      };
      if (command != menu_none){
           Serial.print("Command: ");      Serial.println(command);     // Echo command CHAR in ascii that was sent
        }
      }
      delay(100);
      return command;
}

void _send_to_PC()
{
  double spdf;
  unsigned long spd;
  float lat, lon;
  long alt;
  unsigned long age, dat, tim;
  
  unsigned long buf[4];
  pointer=0;

  
  while (EEPROM_to_GPS(&lat, &lon, &alt, &tim, &dat, &spdf, &pointer)&&(pointer<=EEPROM_size-16))
  {
      Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat,6); Serial.print(", "); Serial.print(lon,6); Serial.println("");
      Serial.print("Date(ddmmyy): "); Serial.print(dat); Serial.print(" Time(hhmmsscc): "); Serial.print(tim); Serial.println("");
      Serial.print("Alt(cm): "); Serial.print(alt); Serial.print(" Speed(mph): ");  Serial.print(spdf,2); Serial.println(""); Serial.println("");
  }
  pointer=0;
}

void _send_to_PC_GPX()
{
  double spdf;
  unsigned long spd;
  float lat, lon;
  long alt;
  unsigned long age, dat, tim;
  unsigned long buf[4];
  int hour, minute, second, day, month, year;
  

  
  pointer=0;     

  Serial.println ("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  Serial.println ("<gpx xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" version=\"1.0\"");
  Serial.println ("xmlns=\"http://www.topografix.com/GPX/1/0\" creator=\"Polar WebSync 2.3 - www.polar.fi\"");
  Serial.println ("xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">");
  
  Serial.println ("<time>2016-10-31T12:00:00Z</time>");
  Serial.println ("<trk>");
  Serial.println ("<name>exercise</name>");
  Serial.println ("<trkseg>");
 

  //dat=(*dat>>11)*10000+((*dat>>7)&15)*100+(*dat&127); //Process data to turn into 12/27/10 form
  
  while (EEPROM_to_GPS(&lat, &lon, &alt, &tim, &dat, &spdf, &pointer)&&(pointer<=EEPROM_size-16))
  {
      hour=(tim/1000000);
      minute=(tim%10000)/100;
      second=(tim%100);
      day=(dat - dat%10000)/10000;
      month=(dat%10000-dat%100)/100;
      year=2000+(dat%100);
  
     Serial.print ("<trkpt lat=\""); Serial.print(lat, 6); Serial.print("\" lon=\""); Serial.print(lon,6); Serial.print ("\">");
     Serial.print ("<time>"); Serial.print(year);  Serial.print ("-"); Serial.print(month); Serial.print ("-"); Serial.print(day); Serial.print ("T"); Serial.print(hour); Serial.print (":");  Serial.print(minute);  Serial.print (":"); Serial.print(second); Serial.print ("Z</time>");
     Serial.println ("</trkpt>");    
  }
 
  Serial.println ("</trkseg>");
  Serial.println ("</trk>");
  Serial.println ("</gpx>");
  pointer=0;
}


void _send_to_PC_KML()
{
  double spdf;
  unsigned long spd;
  float lat, lon;
  long alt;
  unsigned long age, dat, tim;
  unsigned long buf[4];
  pointer=0;
  
  Serial.println ("<?xml version=\"1.0\" standalone=\"yes\"?>");
  Serial.println ("<kml xmlns=\"http://earth.google.com/kml/2.1\">");
  Serial.println ("<Placemark>");
  Serial.println ("<name>Kiev Vashurin Vladimir</name>");
  Serial.println ("<description>Data from Arduino GPS DIY Logger</description>");
  
  while (EEPROM_to_GPS(&lat, &lon, &alt, &tim, &dat, &spdf, &pointer)&&(pointer<=EEPROM_size-16))
  {
     Serial.println ("<Point>");
     Serial.print ("<coordinates>"); Serial.print(lon, 6); Serial.print(","); Serial.print(lat,6); Serial.print ("</coordinates>");
     Serial.println ("</Point>");     
  }
  Serial.println ("</Placemark>");
  Serial.println ("</kml>");
  pointer=0;
}

void _erase()
{
  int temp1, temp2;
  unsigned long buf[4]={0,0,0,0};
  temp1=1;
  Serial.println ("EEPROM Erasing...");
  if (temp1)
    {
    for (unsigned long addr=0;addr<EEPROM_size;addr+=16)
    {
      i2c_eeprom_write_page( 0x50, addr, (byte*) buf, 16);
      delay(5);
    }
  }
  Serial.println ("Erased!");
}

void _print_EEPROM_dump ()
{
  int addr, i, j=0;
  byte b = i2c_eeprom_read_byte(0x50, addr);
  
//  Serial.println("Start dump:");
  for (i=0;i<10; i++)
  {
    for (j=0;j<16; j++)
    {
      Serial.print(b, HEX); Serial.print(" ");
      addr++;
      b = i2c_eeprom_read_byte(0x50, addr);
    }
    Serial.println("#");
  }
}

void _record()
{
  double spdf;
  unsigned long spd;
  float lat, lon, alt;
  unsigned long age, dat, tim;
  unsigned long buf[4];
  char msg[17];

  bool newdata = false;
  unsigned long start = millis();
  
  Serial.println ("Record GPS data is started now!");
  
  // Every few seconds we print an update
  pointer=lower_limit; // Load the lower limit.
  recording = true;
  while(pointer <upper_limit)
  {
    while (ss.available() > 0)
    gps.encode(ss.read());
    start=millis();
  if (gps.location.isUpdated() || gps.altitude.isUpdated())
  {

    GPS_Test ();
      
      if (recording)
      {
        if (pointer<upper_limit)
        {
          GPS_to_EEPROM(&pointer);
          delay (60000);
        }
        
        else
        {
          Serial.print("Limit reached");
          return;
        }
      }
    }
  }
}

void _parameters()
{
}

void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
  int rdata = data;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
}

// WARNING: address is a page address, 6-bit end will wrap around
// also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes
void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length )
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddresspage >> 8)); // MSB
  Wire.write((int)(eeaddresspage & 0xFF)); // LSB
  byte c;
  for ( c = 0; c < length; c++)
    Wire.write(data[c]);
  Wire.endTransmission();
}

byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress )
{
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// maybe let's not read more than 30 or 32 bytes at a time!
void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length )
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,length);
  int c = 0;
  for ( c = 0; c < length; c++ )
    if (Wire.available()) buffer[c] = Wire.read();
}

void GPS_Test ()
{
Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
Serial.print(gps.location.rawLat().negative ? "-" : "+");
Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
Serial.print(gps.location.rawLng().negative ? "-" : "+");
Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
Serial.println(gps.date.year()); // Year (2000+) (u16)
Serial.println(gps.date.month()); // Month (1-12) (u8)
Serial.println(gps.date.day()); // Day (1-31) (u8)
Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
Serial.println(gps.time.hour()); // Hour (0-23) (u8)
Serial.println(gps.time.minute()); // Minute (0-59) (u8)
Serial.println(gps.time.second()); // Second (0-59) (u8)
Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
Serial.println(gps.speed.knots()); // Speed in knots (double)
Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
Serial.println(gps.speed.mps()); // Speed in meters per second (double)
Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
Serial.println(gps.course.deg()); // Course in degrees (double)
Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
Serial.println(gps.altitude.meters()); // Altitude in meters (double)
Serial.println(gps.altitude.miles()); // Altitude in miles (double)
Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
Serial.println(gps.altitude.feet()); // Altitude in feet (double)
Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
}


