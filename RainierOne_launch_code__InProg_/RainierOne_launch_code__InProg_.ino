
#define NUMDATAFIELDS   21      // Number of data fields for each packet.
#define MAXBUFSIZE      200     // Maximum buffer size for serial packet.
#define MAXFIELDSIZE    20      // Maximum size of any data field in the serial packet.

#define SUCCESS         0       // Success return code.
#define ERROR           -1      // Error return code.

#define cycles (bool)   false   //vibration motor cycle process
#define vibCycle (int)  20      //number of seconds the motor will be on at a time (subject to change)

#include <SPI.h>
#include <SD.h>

const int chipSelect = 4; //4 is the pin number- subject to change

// Struct to contain all of the NRNSP flight data in one object.
typedef struct NRdata
{
  char flight_state;
  double exptime;
  double altitude;
  double velocity[3];
  double acceleration[3];
  double attitude[3];
  double angular_velocity[3];
  bool liftoff_warn;
  bool rcs_warn;
  bool escape_warn;
  bool chute_warn;
  bool landing_warn;
  bool fault_warn;
} NRdata;


int parse_serial_packet(const char* buf, NRdata* flight_data);


void setup()
{
  Serial.begin(115200);     // Set baud rate to 115200 (Default serial configureation is 8N1).
  Serial.setTimeout(20);    // Set timeout to 20ms (It may take up to 17ms for all of the data to
                            // transfer from the NRNSP, this ensures that enough time has passed
                            // to allow for a complete transfer before timing out).
  
////////////////////////////////////////////////////////////////////////////////////////////////////
// Set up LED light source (specific pins TBD)
// also set up camera (not sure what this will look like... need to experiment with actual camera)
////////////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(2, OUTPUT); //LED 1
  digitalWrite(2, LOW);
  pinMode(4, OUTPUT); //camera
  digitalWrite (4, LOW);
////////////////////////////////////////////////////////////////////////////////////////////////////
    // Open serial communications and wait for port to open:
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}
}


void loop()
{
  char buffer[MAXBUFSIZE + 1] = {0};      // Buffer for receiving serial packets.
  int res;                                // Value for storing results of function calls.
  NRdata flight_info;                     // Struct for holding current flight data.

  // Initialize the struct and all its data to 0.
  memset(&flight_info, 0, sizeof(NRdata));

  // Loop forever on receiving data packets.
  while (1)
  {
    // Wait for serial input (Delay 1ms between polling serial port).
    while(!Serial.available())
    {
      delay(1);
    }

    // Read in the serial data up to the maximum serial packet size.
    res = Serial.readBytes(buffer, MAXBUFSIZE);

    // If no bytes are read then go back to waiting.
    if (res == 0)
    {
      continue;
    }

    // Null terminate the buffer (Possibly unnecessary).
    buffer[res] = 0;

    // Update the current flight info with the new data.
    res = parse_serial_packet(buffer, &flight_info);

    // If the program failed to parse the new data then go back to waiting.
    if (res != SUCCESS)
    {
      continue;
    }
    if (cycles= true)
    {
      //turn on motor here
      delay(vibCycle);
      //turn off motor here
    }
    
    
////////////////////////////////////////////////////////////////////////////////////////////////////
// Turn on light & camera when power is on (this part is a loop where all on/off founctions should 
// go I think)
// then turn on/off vibration motor
// then turn LED and camera off
// i don't know the actually commands for these components until we get them
// i'd like to test this loop ASAP to see if it even works correctly
////////////////////////////////////////////////////////////////////////////////////////////////////
    (flight_info.flight_state =='@') ? digitalWrite(2, HIGH)); // LED 1 on 
    (flight_info.flight_state =='A') ? digitalWrite(4, HIGH); //turn on camera here
    (flight_info.flight_state =='D') ? cycles= true //vibration motor cycles
    (flight_info.flight_state =='F') ? cycles= false 
    (flight_info.flight_state =='I') ? digitalWrite(2, LOW); //turn off LED1
    (flight_info.flight_state =='I') ? digitalWrite(4,LOW); //turn off camera
////////////////////////////////////////////////////////////////////////////////////////////////////
    
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// parse_serial_packet:
//
// Takes in a buffer containing an NRNSP serial data packet and parses the information into the 
// provided data struct.
//
// buf            =   Buffer containing serial data packet.
// flight_data    =   Struct containing current flight data.
//
// return   =   SUCCESS or ERROR
//
////////////////////////////////////////////////////////////////////////////////////////////////////
int parse_serial_packet(const char* buf, NRdata* flight_data)
{
  int res;                          // Value for storing result of function calls.
  int fieldnum = 1;                 // Index for field number being parsed.
  int index = 0;                    // Index for position in serial buffer.
  char temp[MAXFIELDSIZE] = { 0 };  // Temporary buffer for holding a data field.

  // If the buffer is empty then return with an error.
  if (strlen(buf) == 0)
  {
    return (ERROR);
  }

  // Continue parsing until the end of the buffer is reached
  while (index < strlen(buf))
  {
    // Scan the buffer from the current index until the next comma and place the data into the 
    // temporary buffer.
    res = sscanf((buf + index), "%[^,]", temp);
   
    // If sscanf failed to get a parameter then the buffer was not in the correct format so the 
    // function should return with an error.
    if (res == 0)
    {
      return (ERROR);
    }

    // Increment the buffer index by the length of the data field plus the comma.
    index += (strlen(temp) + 1);

    // If the index for the field number is greater than the expected number of data fields then 
    // the buffer is not correctly formatted so the function should return with an error.
    if (fieldnum > NUMDATAFIELDS)
    {
      return (ERROR);
    }

    // Depending upon the current data field being parsed, convert the data in the temporary buffer 
    // to the appropriate format and store it in the flight data struct.
    switch (fieldnum)
    {
      case 1:
        // Flight state is just a single character.
        flight_data->flight_state = temp[0];
        break;
      case 2:
        // atof() converts a c-string to a floating point number.
        flight_data->exptime = atof(temp);
        break;
      case 3:
        flight_data->altitude = atof(temp);
        break;
      case 4:
        flight_data->velocity[0] = atof(temp);
        break;
      case 5:
        flight_data->velocity[1] = atof(temp);
        break;
      case 6:
        flight_data->velocity[2] = atof(temp);
        break;
      case 7:
        flight_data->acceleration[0] = atof(temp);
        break;
      case 8:
        flight_data->acceleration[1] = atof(temp);
        break;
      case 9:
        flight_data->acceleration[2] = atof(temp);
        break;
      case 10:
        flight_data->attitude[0] = atof(temp);
        break;
      case 11:
        flight_data->attitude[1] = atof(temp);
        break;
      case 12:
        flight_data->attitude[2] = atof(temp);
        break;
      case 13:
        flight_data->angular_velocity[0] = atof(temp);
        break;
      case 14:
        flight_data->angular_velocity[1] = atof(temp);
        break;
      case 15:
        flight_data->angular_velocity[2] = atof(temp);
        break;
      case 16:
        // For the flight warnings, assign a boolean value of false for a 0 and true for anything 
        // else.
        flight_data->liftoff_warn = (temp[0] == '0') ? false : true;
        break;
      case 17:
        flight_data->rcs_warn = (temp[0] == '0') ? false : true;
        break;
      case 18:
        flight_data->escape_warn = (temp[0] == '0') ? false : true;
        break;
      case 19:
        flight_data->chute_warn = (temp[0] == '0') ? false : true;
        break;
      case 20:
        flight_data->landing_warn = (temp[0] == '0') ? false : true;
        break;
      case 21:
        flight_data->fault_warn = (temp[0] == '0') ? false : true;
        break;
    }

    // Increment the index for the current data field.
    fieldnum++;
  }

  // If the correct number of fields have been parsed then return with succcess, otherwise return 
  // with an error.
  if (fieldnum == (NUMDATAFIELDS + 1))
  {
    return (SUCCESS);
  }
  else
  {
    return (ERROR);
  }
    // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File flight_info = SD.open("flightdata.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (flight_info) {
    flight_info.println(temp);
    flight_info.close();
    // print to the serial port too:
    Serial.println(temp);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
