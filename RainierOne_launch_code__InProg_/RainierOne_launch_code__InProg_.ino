// WIRING//////////////////////////////////////////////////////

//SD:
//DI- pin 11
//DO- pin 12
//CLK- pin 13
//CS- pin 4
//5v to 5v
//GND to GND

//Servo: 
//Red- 5v
//Black- GND
//Orange- pin 8

//LED #1:
//Positive terminal- pin 9
//Negative terminal- GND

//LED #2:
//Positive terminal- pin 6
//Negative terminal- GND

//rPi:
//GPIO: pin 7
//5V, GND

 
#include <SPI.h>      
#include <SD.h>
#include <Servo.h>

Servo spinnySam;
int angle;
int n=0;

#define NUMDATAFIELDS   21      // Number of data fields for each packet.
#define MAXBUFSIZE      200     // Maximum buffer size for serial packet.
#define MAXFIELDSIZE    20      // Maximum size of any data field in the serial packet.

#define SUCCESS         0       // Success return code.
#define ERROR           -1      // Error return code.

#define ledOne          9 
#define ledTwo          6
#define rPiStart        7
#define rPiStop         11

// Struct to contain all of the NRNSP flight data in one object.
typedef struct NRdata {
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

File rainierFlight;


int parse_serial_packet(const char* buf, NRdata* flight_data);


void setup()
{
	Serial.begin(115200);     // Set baud rate to 115200 (Default serial configureation is 8N1).
	Serial.setTimeout(20);    // Set timeout to 20ms (It may take up to 17ms for all of the data to
	                        // transfer from the NRNSP, this ensures that enough time has passed
	                        // to allow for a complete transfer before timing out).

	pinMode(ledOne, OUTPUT);
	pinMode(ledTwo, OUTPUT); 
	pinMode(rPiStart, OUTPUT); 
	pinMode(rPiStop, OUTPUT); 
	spinnySam.attach(8); 
}


void loop()
{
	char buffer[MAXBUFSIZE + 1] = {0};      // Buffer for receiving serial packets.
	int res;                                // Value for storing results of function calls.
	NRdata flight_info;                     // Struct for holding current flight data.
	bool logOpened = false;
	bool ran = false;

	// Initialize the struct and all its data to 0.
	memset(&flight_info, 0, sizeof(NRdata));
	// Loop forever on receiving data packets.
	while (1)
	{
	// Wait for serial input (Delay 1ms between polling serial port).
		while(!Serial.available()) {
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
		buffer[res] = 0; //!!!does this actually null terminate? wtf is res?

		// Update the current flight info with the new data.
		res = parse_serial_packet(buffer, &flight_info); //!!!why are we redefining res

		// If the program failed to parse the new data then go back to waiting.
		if (res != SUCCESS)
		{
		  continue;
		}

		if (flight_info.flight_state == 'A'){ //TAKEOFF
		  digitalWrite(ledOne, HIGH);  //LED #1 on
		  digitalWrite(ledTwo, HIGH); // LED #2 on
		  digitalWrite(rPiStart, HIGH); //start video recording
		  spinnySam.write(45); // reset servo

		}   

		else if (flight_info.flight_state == 'I') { //SAFING
		  digitalWrite(ledOne, LOW);   //LED #1 off
		  digitalWrite(ledTwo, LOW);
		  digitalWrite(rPiStop, HIGH); //stop video recording
		  rainierFlight.close(); //save and close flight data text file
		  logOpened = false;
		}   

		else if (flight_info.flight_state == 'D' && ran==false) { // COAST START
		  while (n<3){ //spin 5 times
		    spinnySam.write(175);    //spin up 
		    delay(330);
		    spinnySam.write(45);   //spin down
		    delay(330);
		    n++;  
		  }
		  delay(15000);
		  for (int n = 0; n < 3; n++) { //spin 5 times
		    spinnySam.write(175);    //spin up 
		    delay(330);
		    spinnySam.write(45);   //spin down
		    delay(330); 
		  }
		  ran=true;  //it should only go through this spin cycle once
		}



		if ((flight_info.flight_state == 'A') && !logOpened) { //begin logging
      SD.begin(4);
      rainierFlight = SD.open("flightdata.txt", FILE_WRITE);
		  logOpened= true;
		}
		else if (logOpened) {
		  rainierFlight.print(flight_info.flight_state + ","); // will print current flight state to SD card, with comma in between
		}
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
}


