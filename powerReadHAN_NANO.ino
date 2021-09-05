#include <WiFiNINA.h>
#include "crc16.h"

/*
 Example of a incoming message: 
 ------------------- BEGIN ---------------
/ELL5\253833635_A

0-0:1.0.0(210828152453W)
1-0:1.8.0(00000119.841*kWh)
1-0:2.8.0(00000533.758*kWh)
1-0:3.8.0(00000003.410*kvarh)
1-0:4.8.0(00000187.265*kvarh)
1-0:1.7.0(0000.532*kW)
1-0:2.7.0(0000.000*kW)
1-0:3.7.0(0000.193*kvar)
1-0:4.7.0(0000.407*kvar)
1-0:21.7.0(0000.000*kW)
1-0:41.7.0(0000.584*kW)
1-0:61.7.0(0000.033*kW)
1-0:22.7.0(0000.085*kW)
1-0:42.7.0(0000.000*kW)
1-0:62.7.0(0000.000*kW)
1-0:23.7.0(0000.000*kvar)
1-0:43.7.0(0000.000*kvar)
1-0:63.7.0(0000.193*kvar)
1-0:24.7.0(0000.056*kvar)
1-0:44.7.0(0000.350*kvar)
1-0:64.7.0(0000.000*kvar)
1-0:32.7.0(247.3*V)
1-0:52.7.0(247.3*V)
1-0:72.7.0(245.1*V)
1-0:31.7.0(000.5*A)
1-0:51.7.0(003.0*A)
1-0:71.7.0(000.8*A)
!2B41
------------------- END ---------------

The protocol:
(ref: https://hanporten.se/svenska/protokollet.html)
   ======================================================================
   OBIS         Beskrivning                                Kommentar
   ----         -----------                                ---------
   0-0.1.0.0    Datum och tid                              Formatet YYMMDDhhmmssX.
   1-0:1.8.0    Mätarställning Aktiv Energi Uttag.  
   1-0:2.8.0    Mätarställning Aktiv Energi Inmatning 
   1-0:3.8.0    Mätarställning Reaktiv Energi Uttag 
   1-0:4.8.0    Mätarställning Reaktiv Energi Inmatning 
   1-0:1.7.0    Aktiv Effekt Uttag                         Momentan trefaseffekt
   1-0:2.7.0    Aktiv Effekt Inmatning                     Momentan trefaseffekt
   1-0:3.7.0    Reaktiv Effekt Uttag                       Momentan trefaseffekt
   1-0:4.7.0    Reaktiv Effekt Inmatning                   Momentan trefaseffekt
   1-0:21.7.0   L1 Aktiv Effekt Uttag                      Momentan effekt
   1-0:22.7.0   L1 Aktiv Effekt Inmatning                  Momentan effekt
   1-0:41.7.0   L2 Aktiv Effekt Uttag                      Momentan effekt
   1-0:42.7.0   L2 Aktiv Effekt Inmatning                  Momentan effekt
   1-0:61.7.0   L3 Aktiv Effekt Uttag                      Momentan effekt
   1-0:62.7.0   L3 Aktiv Effekt Inmatning                  Momentan effekt
   1-0:23.7.0   L1 Reaktiv Effekt Uttag                    Momentan effekt
   1-0:24.7.0   L1 Reaktiv Effekt Inmatning                Momentan effekt
   1-0:43.7.0   L2 Reaktiv Effekt Uttag                    Momentan effekt
   1-0:44.7.0   L2 Reaktiv Effekt Inmatning                Momentan effekt
   1-0:63.7.0   L3 Reaktiv Effekt Uttag                    Momentan effekt
   1-0:64.7.0   L3 Reaktiv Effekt Inmatning                Momentan effekt
   1-0:32.7.0   L1 Fasspänning                             Momentant RMS-värde
   1-0:52.7.0   L2 Fasspänning                             Momentant RMS-värde
   1-0:72.7.0   L3 Fasspänning                             Momentant RMS-värde
   1-0:31.7.0   L1 Fasström                                Momentant RMS-värde
   1-0:51.7.0   L2 Fasström                                Momentant RMS-värde
   1-0:71.7.0   L3 Fasström                                Momentant RMS-värde
*/


//#define DEBUG       1    // Uncomment to enable debug
#define BUFFER_LEN  1000 // Message buffer length in bytes
#define CRC_LEN     4    // Checksum length in bytes

enum State
{
  WAITING = 0,
  READ_MESSAGE = 1,
  READ_CHECKSUM = 2,
  DECODE_MESSAGE = 3
};

enum Field
{
  ENERGY_DELIVERED = 0, // 1.8.0 (kWh)
  ENERGY_RETURNED,      // 2.8.0 (kWh)
  POWER_DELIVERED,      // 1.7.0 (kW)
  POWER_RETURNED        // 2.7.0 (kW)
};

enum State currentState; // Keeps track of current state

char buffer[BUFFER_LEN] = {0};  // Main buffer for message
char checksumBuffer[CRC_LEN+1]; // Buffer for storing checksum plus space for '\0'

char incomingByte = 0;        // For incoming serial data
int numberOfBytesRead = 0;    // Count number of message bytes received
int numberOfCrcBytesRead = 0; // Count number of checksum bytes received


// WiFi
WiFiClient client;
char ssid[] = "my-ssid";        // Your network SSID (name)
char pass[] = "wifi-password";      // Your network password (use for WPA, or use as key for WEP)
int wifiStatus = WL_IDLE_STATUS;     // The Wi-Fi radio's status

int ledState = LOW;

/* ======================================================================
 * 
*/
void setup() 
{
  // Initialize crc table
  initCrcTable();

  // Set initial state
  setState(WAITING);
  
  // Initialize both serial ports:
  #ifdef DEBUG
  Serial.begin(9600);
  //while (!Serial); // Wait for Serial Monitor to open
  #endif
  Serial1.begin(115200);

  // Set built in LED pin to output mode
  pinMode(LED_BUILTIN, OUTPUT);

  //
  // Flash the LED while connecting to WiFi
  //
  int timerCount = 0;
  while (true)
  {
    if (timerCount == 0)
    {
      if (wifiStatus == WL_CONNECTED)
      {
        break; // Connected
      }

      #ifdef DEBUG
      Serial.print("Attempting to connect to network: ");
      Serial.println(ssid);
      #endif

      // Connect to WPA/WPA2 network:
      wifiStatus = WiFi.begin(ssid, pass);
    }
    
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) 
    {
      ledState = HIGH;
    } 
    else 
    {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
    timerCount += 500;

    if (timerCount >= 10000)
    {
      timerCount = 0;
    }

    delay(500); // Sleep 500ms
  }

  // We are connected now
  #ifdef DEBUG
  Serial.println("Connected to the network"); 
  Serial.println("---------------------------------------");
  #endif

  // Light the onboard LED to indicate WiFi connection ok
  digitalWrite(LED_BUILTIN, HIGH);
}


/* ======================================================================
 * Set the current state
*/
void setState(enum State newState)
{
  currentState = newState;
}


/* ======================================================================
 * Extracts a value from the message
*/
void getValue(enum Field id, char buff[])
{
  char colon = ':';
  char digit1;
  char digit2;
  char digit3;
  bool skipDecimal = true;
  
  switch (id)
  {
    case ENERGY_DELIVERED:
    {
      digit1 = '1';
      digit2 = '8';
      digit3 = '0';
      skipDecimal = false;
    }
    break;

    case ENERGY_RETURNED:
    {
      digit1 = '2';
      digit2 = '8';
      digit3 = '0';
      skipDecimal = false;
    }
    break;
    
    case POWER_DELIVERED:
    {
      digit1 = '1';
      digit2 = '7';
      digit3 = '0';
    }
    break;

    case POWER_RETURNED:
    {
      digit1 = '2';
      digit2 = '7';
      digit3 = '0';
    }
    break;
    
    default:
    {
      // ERROR return "0"
      buff[0] = '0';
      buff[1] = '\0';
      return;
    }
  }

  //
  // Find the value in the main message buffer
  //
  for (int i=0; i<numberOfBytesRead; i++)
  {
    // Look for a string like this ":2.7.0" in "1-0:2.7.0(0003.023*kW)"
    if ((buffer[i] == colon) &&
        (buffer[i+1] == digit1) &&
        (buffer[i+3] == digit2) &&
        (buffer[i+5] == digit3))
    {
      // Read the value Ex: "0003.023" from "1-0:2.7.0(0003.023*kW)"
      int pos = i+7; // Step 7 characters to reach the start-point
      int count = 0;
      do
      {
        if ((buffer[pos] == '.') && skipDecimal)
        {
          pos++; // Skip the '.'
        }
        else
        {
          buff[count++] = buffer[pos++];
        }
      } while ((buffer[pos] != '*') && (buffer[pos] != '\n'));

      // Sanity check - if we reached the newline char something went wrong - abort
      if (buffer[pos] == '\n')
      {
        buff[0] = '0';
        buff[1] = '\0';
        break;
      }

      buff[count] = '\0';
      break;
    }
  }
}


/* ======================================================================
 * Sends selected values to Domoticz server
*/
void sendValuesToServer(enum Field fields[], unsigned short numberOfFields)
{
  char deviceId[5]; // Device ID in Domoticz
  
  for (int field = 0; field < numberOfFields; field++)
  {
    switch (fields[field])
    {
      case ENERGY_DELIVERED:
      strcpy(deviceId, "267\0");
      break;

      case ENERGY_RETURNED:
      strcpy(deviceId, "270\0");
      break;
      
      case POWER_DELIVERED:
      strcpy(deviceId, "268\0");
      break;

      case POWER_RETURNED:
      strcpy(deviceId, "269\0");
      break;

      default:
      // Error
      return;
    }

    char valueBuffer[20] = {0};
    getValue(fields[field], valueBuffer);
    #ifdef DEBUG
    Serial.println(valueBuffer);
    #endif

    char url[150];
    int len = sprintf(url, "GET /json.htm?type=command&param=udevice&idx=%s&nvalue=0&svalue=%s HTTP/1.1", deviceId, valueBuffer);
    url[len] = '\0';
    
    #ifdef DEBUG
    Serial.println(url);
    #endif

    if (client.connect("my-domoticz-adress.com", 8080)) 
    {
      #ifdef DEBUG
      Serial.println("Connected to server");
      #endif
      // Make a HTTP request:
      //client.println("GET /json.htm?type=command&param=udevice&idx=265&nvalue=0&svalue=000226 HTTP/1.1");
      client.println(url);
      client.println("Host: my-domoticz-adress.com");
      client.println("Authorization: Basic dXNlcm5hbWU6cGFzc3dvcmQ="); // BASE64 encoding of Domoticz login "username:password"
      client.println("Connection: close");
      client.println();
      #ifdef DEBUG
      Serial.println("Request sent");
      #endif
    }
    else
    {
      #ifdef DEBUG
      Serial.println("Connection failed");
      #endif
    }
  }
}


/* ======================================================================
 * Read incoming serial bytes and determine if it is the start or stop mark
*/
void readData()
{
    incomingByte = (char)Serial1.read();

    if (incomingByte == '/')
    {
      setState(READ_MESSAGE);
    }

    if ((incomingByte == '!') && (currentState == READ_MESSAGE))
    {
      setState(READ_CHECKSUM);
    }
}

/* ======================================================================
 * Process the data
*/
void processData()
{
    if (currentState == READ_CHECKSUM)
    {
      if (incomingByte == '!')
      {
        buffer[numberOfBytesRead] = incomingByte;
        numberOfBytesRead++;
      }
      else
      {
        if (numberOfCrcBytesRead < CRC_LEN)
        {
          checksumBuffer[numberOfCrcBytesRead] = incomingByte;
          numberOfCrcBytesRead++;
        }
        else
        {
          // Checksum reading done
          
          // Add end-of-line
          checksumBuffer[CRC_LEN] = '\0';
          
          // Calculate the checksum on the received message
          unsigned short calculatedCrc = crc16(buffer, numberOfBytesRead);
          #ifdef DEBUG
          Serial.print("Calculated CRC16:");
          Serial.println(calculatedCrc, HEX);
          #endif

          // Convert the checksum string to a 16 bit integer
          unsigned short receivedCrc = (unsigned short)strtoul(checksumBuffer, NULL, 16);
          #ifdef DEBUG
          Serial.print("Received CRC16:");
          Serial.println(receivedCrc, HEX);
          #endif

          // Check if checksum matches
          if (calculatedCrc == receivedCrc)
          {
            // CRC ok - Start decode
            setState(DECODE_MESSAGE);
          }
          else
          {
            // CRC failed wait for next message
            numberOfBytesRead = 0;
            numberOfCrcBytesRead = 0;
            setState(WAITING);
            #ifdef DEBUG
            Serial.println("CRC16 failed. Waiting for next message.");
            #endif
          }
        }
      }
    }

    if (currentState == READ_MESSAGE)
    {
      buffer[numberOfBytesRead] = incomingByte;
      numberOfBytesRead++;
    }

    if (currentState == DECODE_MESSAGE)
    {
      #ifdef DEBUG
      for (int i=0; i<numberOfBytesRead; i++)
      {
        Serial.print(buffer[i]);
      }

      for (int i=0; i<numberOfCrcBytesRead; i++)
      {
        Serial.print(checksumBuffer[i]);
      }
      Serial.println("");
      #endif

      // Select the Fields we wanna send
      enum Field fields[] = { POWER_DELIVERED, POWER_RETURNED, ENERGY_DELIVERED, ENERGY_RETURNED };
      sendValuesToServer(fields, 4); // 4 = number of fields

      #ifdef DEBUG
      Serial.println("-------");
      #endif

      numberOfBytesRead = 0;
      numberOfCrcBytesRead = 0;
      setState(WAITING);
    }
}

/* ======================================================================
 * Main loop
*/
void loop() 
{
  // Check for incoming byte(s) on Serial1
  if (Serial1.available())
  {
    readData();
    processData();
  }
}
