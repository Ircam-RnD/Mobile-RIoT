// DOC - http://www.ti.com/lit/ug/swru455c/swru455c.pdf

/* R-IoT :

 Current version : 1.8.1

 Texas Instrument CC3200 Internet of Things / Sensor hub / Dev Platform
 80 MHz 32 Bit ARM MCU + Wifi stack / modem

 IRCAM - Emmanuel FLETY - Music Bricks

 Battery voltage : protection voltage divider of 0.364. ADC vRef (and max input) = 1.7V (ish)
 Battery reading in volt = [analogRead() * 1.7] / (4096 * 0.364)
 or, simplified to
 Battery reading in volt = analogRead() / 877.

 Rev History :

 1.8 : GPIO switch inverted logic and single OSC message to fix max/MSP udpreceive object FIFO issue

 1.7 : corrected yaw drift and error + auto calibration for gyro every 5sec if no movement / stable + improved mag calibration

 1.6 : added I2C support of the LMS9DS0 for external sensors

 1.5 : adding a AP style connection to allow streaming to multiple computers / devices

 1.4 :  loads of fixing in the calibration process for the absolute angles (madgwick)
 and webserver

 */

#include <stdio.h>
#include <strings.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <Wire.h>

// Handles the file system of the FLASH, to store parameters
#include <SLFS.h>

#include "common.h"
#include "LSM9DS0.h"
// #include "osc.h"
#include "web.h"
#include <WebSocket.h>

/////////////////////////////////////////////////////////////
// DEFAULT parameters
// your network name also called SSID
byte mac[6];
const char TheSSID[] = "riot";
const uint16_t TheDestPort = DEFAULT_UDP_PORT;
const uint8_t TheLocalIP[] = {
  192,168,1,40};
const uint8_t TheSubnetMask[] = {
  255,255,255,0};
const uint8_t TheGatewayIP[] = {
  192,168,1,1};
const uint8_t TheDestIP[] = {
  192,168,1,100};
const unsigned long TheSampleRate = DEFAULT_SAMPLE_RATE;
const uint8_t TheID = 0;

/////////////////////////////////////////////////////////////
// Global vars
WiFiServer server(80);
WiFiClient client;
WiFiClient clientStreaming[MAX_CLIENTS];

webSocketServer webSocketServer;

byte APorStation = STATION_MODE;
char ssid[32];
char ssidAP[32];
char password[32] = "12345678";
IPAddress LocalIP;
IPAddress APIP;
IPAddress SubnetMask;
IPAddress GatewayIP;
IPAddress DestIP;
uint16_t DestPort;
uint8_t ModuleID;
unsigned long SampleRate;
boolean UseDHCP = true;
boolean UseSecurity = false;
int status = WL_IDLE_STATUS;
int statusAP = false;
int PacketStatus;
boolean ConfigurationMode = false;
boolean AcceptOSC = true;
byte PageToDisplay = CONFIG_WEB_PAGE;
unsigned int ConfigModeCounter = 0;
int TempInt = 0;

char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP UdpPacket;
WiFiUDP ConfigPacket;
// OscBuffer RawSensors;
// OscBuffer Quaternions;
// OscBuffer EulerAngles;
// OscBuffer AnalogInputs;
// OscBuffer Message;

unsigned long ElapsedTime = 0;
unsigned long ElapsedTime2 = 0;

unsigned long now = 0;

////////////////////////////////////////////////////////////
// Sensor storage
short unsigned int BatteryVoltage = 0, SwitchState, ActualSwitchState;
Word AccelerationX, AccelerationY, AccelerationZ;
Word GyroscopeX, GyroscopeY, GyroscopeZ;
Word MagnetometerX, MagnetometerY, MagnetometerZ;
Word Temperature;
byte CommunicationMode = SPI_MODE;
//byte CommunicationMode = I2C_MODE;

short unsigned int AnalogInput1, AnalogInput2;

// Defines whether you want the "raw" value with the stored offset or not
//byte SendCalibrated = true;
byte SendCalibrated = false;

byte wsHeaderAck = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Absolute angle (madgwick)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PI 3.14159265358979323846264338327950
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
// Beta is called the rate of convergence of the filter. Higher value lead to a noisy output but fast response
// If beta = 0.0 => uses gyro only. A very high beta like 2.5 uses almost no gyro and only accel + magneto.

#define BETA_DEFAULT 0.4f   // Much faster - noisier
#define BETA_MAX     2.0f
//#define beta 0.041f    // 10 seconds or more to have the quaternion converge to the right orientation - super smooth

float beta = BETA_DEFAULT;
float madgwick_beta_max = BETA_MAX;
float madgwick_beta_gain = 1.0f;
float acc_lp_alpha = 0.01;  // 10 ms

int gyroOffsetAutocalTime = 5000; //ms = 1000 samples @5ms
long gyroOffsetAutocalThreshold = 100; //LSB
long gyroOffsetAutocalCounter; //internal
boolean gyroOffsetAutocalOn = false;
boolean gyroOffsetCalDone = false;
long gyroOffsetCalElapsed = 0;
long gyroOffsetAutocalMin[3];
long gyroOffsetAutocalMax[3];
long gyroOffsetAutocalSum[3];
long magOffsetAutocalMin[3];
long magOffsetAutocalMax[3];
long accOffsetAutocalSum[3];


float pitch, yaw, roll, heading;
float Declination = DECLINATION;
float deltat = 0.005f;        // integration interval for both filter schemes - 5ms by default

int gyro_bias[3] = { 0, 0, 0};
int accel_bias[3] = { 0, 0, 0};
int mag_bias[3] = { 0, 0, 0};
int bias_samples = 32;

float abias[3] = { 0., 0., 0.};
float gbias[3] = { 0., 0., 0.};
float mbias[3] = { 0., 0., 0.};

float gRes, aRes, mRes;		// Resolution = Sensor range / 2^15
float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z; // variables to hold latest sensor data values
float gyro_norm; // used to tweak Beta
float mag_nobias[3];
float mag_cal[3];  // calibrated version of the mag sensor data

float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

/////////////////////////////////////////////////////////////////////////////
// Magnetic Sensors calibration using a rotation / transformation matrix
float rotation_matrix[3][3] =
{
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 }
};

float identity_matrix[3][3] =
{
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 }
};

float rotation_matrix_result[3][3];
byte CalibrationStage = START;

// MAG Vectors for all cardinal positions (3 items per array, X, Y, Z)
// Used for elipsoid fitting
float Xplus0[3];
float Xplus180[3];
float Xminus0[3];
float Xminus180[3];

float Yplus0[3];
float Yplus180[3];
float Yminus0[3];
float Yminus180[3];

float Zplus0[3];
float Zplus180[3];
float Zminus0[3];
float Zminus180[3];

String sg_x;
String sg_y;
String sg_z;
String sa_x;
String sa_y;
String sa_z;
unsigned long before;
unsigned long after;
unsigned long dt;

/////////////////////////////////////////////////////////////////
// Serial port message / buffers / temporary strings
char SerialBuffer[MAX_SERIAL];
unsigned char SerialIndex = 0;
boolean FlagSerial = FALSE;
char StringBuffer[MAX_STRING];


// To get printf to work, we redirect STDOUT and the myWrite functions
ssize_t myWrite(void *cookie, const char *buf, size_t n)
{
  return Serial.write((uint8_t*)buf, n);
}

cookie_io_functions_t myVectors = {
  0, myWrite, 0, 0 };

void setup() {
  // Basic I/Os
  pinMode(POWER_LED, OUTPUT);
  pinMode(SWITCH_INPUT, INPUT_PULLUP);

  // POWER On indicator
  digitalWrite(POWER_LED, HIGH);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.println("=========================================================");
  Serial.println(VERSION_DATE);
  Serial.println("=========================================================");

  // Needed to have printf working
  stdout = fopencookie((void *)0, "w", myVectors);
  setlinebuf(stdout);

  digitalWrite(POWER_LED, HIGH);

  // Check if we are going in configuration mode
  // 2-3 second pressing on the switch during boot
  while(!digitalRead(SWITCH_INPUT))
  {
    // Serial.println("switch pressed");
    delay(20);
    TempInt++;
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);

    if(TempInt > WEB_SERVER_DELAY)
    {
      ConfigurationMode = true;
      Serial.println("Configuration / Web Server Mode");
      digitalWrite(POWER_LED, HIGH);
      break;
    }
  }

  // Init motion sensor
  Serial.println("Init Motion Sensor");
  // Start SPI with defaults
  SPI.begin();
  // SPI settings
  // 16 MHz max bit rate, clock divider 1:2 => 8 MHZ SPI clock
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  InitLSM9DS0();

  // Scaling to obtain gs and deg/s
  // Must match the init settings of the LSM9DS0
  // Beware, absolute dynamic range of each sensor isn't equivalent.
  // Accelerometers are using the whole dynamic range, gyro / mag aren't
  gRes = 2000.0 / 32768; 	// +- 2000 deg/s
  aRes = 8.0 / 32768.0;         // +- 8g
  mRes = 2.0 / 32768.0;         // +- 2 gauss

  // Starts the file system
  Serial.println("Loading up params");

  SerFlash.begin();

  // Retrieve saved params in FLASH using the file system
  LoadParams();

  // Finalize the bias unit conversion
  for(int i = 0 ; i < 3 ; i++)
  {
    gbias[i] = gRes * (float)gyro_bias[i];
    abias[i] = aRes * (float)accel_bias[i];
    mbias[i] = mRes * (float)mag_bias[i];
  }

  if(!ConfigurationMode)
  {
    digitalWrite(POWER_LED, LOW);

    if(APorStation == STATION_MODE)
    {
      // attempt to connect to Wifi network:
      Serial.print("R-IoT connecting to: ");
      // print the network name (SSID);
      Serial.println(ssid);

      Connect();

      // kill built-in http server, cf.
      // https://github.com/energia/Energia/blob/master/hardware/lm4f/libraries/WiFi/utility/netapp.h
      // http://forum.43oh.com/topic/6318-how-to-delete-mysimplelinknet-web-page/
      sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);

      Serial.println("Starting webserver on port 80");
      server.begin();
      Serial.println("Webserver started!");
    }
    else  // AP mode
    {
      // attempt to connect to Wifi network:
      Serial.print("R-IoT creates network: ");
      // print the network name (SSID);
      Serial.println(ssid);

      // Creates the AP & config
      APIP = IPAddress(TheGatewayIP);
      Serial.println(APIP);

      WiFi.config(APIP);
      if(!UseSecurity)
        WiFi.beginNetwork((char *)ssid);
      else
        WiFi.beginNetwork((char *)ssid, (char*)password);

      sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);

      Serial.println("Starting webserver on port 80");
      server.begin();
      Serial.println("Webserver started!");
    }

    WiFi.macAddress(mac);

    // Open the service port to talk to the module (config, calibration)
    ConfigPacket.begin(DEFAULT_UDP_SERVICE_PORT);
  } // END OF IF NORMAL (!CONFIG) MODE

  // If in configuration mode we setup a webserver for configuring the unit
  // The module becomes an AP with DHCP
  else
  {
    APIP = IPAddress(TheGatewayIP);

    WiFi.config(APIP);
    randomSeed(analogRead(BATT_MONITORING));
    sprintf(ssidAP, "RIOT-%04x\0",random(16000));
    Serial.print("Setting up Access Point named: ");
    Serial.println(ssidAP);
    WiFi.beginNetwork((char *)ssidAP);
    WiFi.macAddress(mac);
  }

  ElapsedTime = millis();
  ElapsedTime2 = millis();

  now = millis();
}

void loop() {

  // if (millis() - now > 200) {
  //   now = millis();

  //   if (digitalRead(POWER_LED))
  //     digitalWrite(POWER_LED, LOW);
  //   else
  //     digitalWrite(POWER_LED, HIGH);
  // }


  if(!ConfigurationMode)
  {
    if((millis() - ElapsedTime2) > 300) // Perform the check not too often
    {
      ElapsedTime2 = millis();
      if(APorStation == STATION_MODE)
      {
        int CurrentStatus = WiFi.status();

        if(CurrentStatus != WL_CONNECTED)
        {
          //printf("wifi status = %d\n", WiFi.status());
          // print dots while we wait to connect and blink the power led
          Serial.print(".");
          if(digitalRead(POWER_LED))
            digitalWrite(POWER_LED, LOW);
          else
            digitalWrite(POWER_LED, HIGH);
        }
        // Newly connected to the network, locks until DHCP answers
        // if enabled
        if((status != WL_CONNECTED) && (WiFi.status() == WL_CONNECTED))
        {
          status = WiFi.status();
          digitalWrite(POWER_LED, HIGH);
          Serial.println("\nConnected to the network");

          while ((WiFi.localIP() == INADDR_NONE))
          {
            // print dots while we wait for an ip addresss
            Serial.print(".");
            delay(300);
          }
          // you're connected now, so print out the status
          printCurrentNet();
          printWifiData();
        }

        // Disconnected from the network, try to reconnect
        if((status == WL_CONNECTED) && (WiFi.status() != WL_CONNECTED))
        {
          Serial.println("Network Lost, trying to reconnect");
          status = WiFi.status();
          // Debug
          //Serial.print("Current Status = ");
          //Serial.println(status);
        }
      } // End of Station mode connection
      else  // AP mode
      {
        if(WiFi.localIP() == INADDR_NONE)   // Indicates AP isn't ready yet
        {
          // print dots while we wait to connect and blink the power led
          Serial.print(".");
          if(digitalRead(POWER_LED))
            digitalWrite(POWER_LED, LOW);
          else
            digitalWrite(POWER_LED, HIGH);
        }
        else if (!statusAP)
        {
          statusAP = true;
          digitalWrite(POWER_LED, HIGH);
          Serial.println("AP active.");
          printCurrentNet();
          printWifiData();
        }
      } // End of AP mode connection
    } // end of IF(elapsed time)


    if (
      (millis() - ElapsedTime >= SampleRate) &&
      !ConfigurationMode &&
      (
        ((WiFi.status() == WL_CONNECTED) && (APorStation == STATION_MODE)) ||
        (statusAP && (APorStation == AP_MODE))
      )
    ) {
      ElapsedTime = millis();
      digitalWrite(POWER_LED, HIGH);
      // read the battery status
      BatteryVoltage = analogRead(BATT_MONITORING);
      SwitchState = digitalRead(SWITCH_INPUT);
      ActualSwitchState = !SwitchState;

      // Comment those 2 if you don't need the analog inputs to be exported by OSC
      AnalogInput1 = analogRead(ANALOG_INPUT1);
      AnalogInput2 = analogRead(ANALOG_INPUT2);

      if(!SwitchState)
      {
        ConfigModeCounter++;

        if(ConfigModeCounter > 600)
        {
          ConfigModeCounter = 0;
          CalibrateAccGyroMag();
        }
      }
      else
      {
        ConfigModeCounter = 0;
      }


      ReadAccel();
      ReadGyro();
      // ReadMagneto();
      ReadTemperature();



      // ----------------------------------------------------------------
      // ----------------------------------------------------------------
      // TEST WEBSOCKET
      // ----------------------------------------------------------------
      // ----------------------------------------------------------------

      client = server.available();

      if (client.connected() && !wsHeaderAck) {
        if (webSocketServer.handshake(client)) {
          wsHeaderAck = true;
          Serial.println("Websocket connected");
        }
      }

      if (wsHeaderAck && !client.connected()) {
        Serial.println("Websocket disconnected");
        wsHeaderAck = false;
      }

      // ----------------------------------------------------------------
      // ----------------------------------------------------------------
      // END TEST WEBSOCKET
      // ----------------------------------------------------------------
      // ----------------------------------------------------------------


      if((millis() - gyroOffsetCalElapsed > gyroOffsetAutocalTime) && gyroOffsetCalDone)
      {
        gyroOffsetCalElapsed = millis();
        gyroOffsetCalDone = false;
      }

      if(!gyroOffsetCalDone && gyroOffsetAutocalOn)
      {
        gyroOffsetCalibration();
      }

      g_x = (gRes * (float)GyroscopeX.Value) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
      g_y = (gRes * (float)GyroscopeY.Value) - gbias[1];
      g_z = (gRes * (float)GyroscopeZ.Value) - gbias[2];

      a_x = (aRes * (float)AccelerationX.Value) - abias[0];   // Convert to g's, remove accelerometer biases
      a_y = (aRes * (float)AccelerationY.Value) - abias[1];
      a_z = (aRes * (float)AccelerationZ.Value) - abias[2];

      if (client.connected() && wsHeaderAck) {
        sg_x = String((int) g_x);
        sg_y = String((int) g_y);
        sg_z = String((int) g_z);
        sa_x = String((int) (a_x * 100));
        sa_y = String((int) (a_y * 100));
        sa_z = String((int) (a_z * 100));

        before = millis();

        String msg = String(sg_x + " " + sg_y + " " + sg_z + " " + sa_x + " " + sa_y + " " + sa_z);
        webSocketServer.sendData(msg);

        after = millis();

        // log
        dt = after - before;
        if (dt > 50) {
          Serial.println(dt);
        }
      }
    }
  }
  //////////////////////////////////////////////////////////////////////////////////
  // Handles the web server
  else
  {
    if(WiFi.localIP() == INADDR_NONE)
    {
      if((millis() - ElapsedTime2) > 100) // Blinks faster than during normal mode
      {
        ElapsedTime2 = millis();
        // print dots while we wait to connect and blink the power led
        Serial.print(".");
        if(digitalRead(POWER_LED))
          digitalWrite(POWER_LED, LOW);
        else
          digitalWrite(POWER_LED, HIGH);
      }
    }

    else if (!statusAP) {
      statusAP = true;
      digitalWrite(POWER_LED, HIGH);
      Serial.println("AP active.");
      printCurrentNet();
      printWifiData();
      Serial.println("Starting webserver on port 80");
      server.begin();
      Serial.println("Webserver started!");
    }

    if(statusAP) // We can accept clients
    {
      char c;
      char LocalHttpBuffer[300];
      int HttpBufferIndex = 0;
      unsigned int Index, Rank;

      client = server.available();

      if (client) {
        Serial.println("new client");
        // an http request ends with a blank line
        boolean currentLineIsBlank = true;
        boolean StayConnected = true;

        while (client.connected() && StayConnected )
        {
          if (client.available())
          {

            c = client.read();
            Serial.write(c);

            if(c != '\r')
              LocalHttpBuffer[HttpBufferIndex++] = c;
            if(!currentLineIsBlank && c == '\n')
            {
              LocalHttpBuffer[HttpBufferIndex++] = '\0';
              // Process HTTP contents
              //Serial.println("new http line - processing");
              //Serial.print(LocalHttpBuffer);
              HttpBufferIndex = 0;
              if(!strncmp(LocalHttpBuffer, "GET / ", 6))
                PageToDisplay = CONFIG_WEB_PAGE;

              else if(!strncmp(LocalHttpBuffer, "GET /params", 11))
              { // Apply settings request - parsing all parameters, stores, update, reboot
                char *pHtml = LocalHttpBuffer;
                // position the pointer on the first param
                while(*pHtml != '?' && *pHtml != '\0')  pHtml++;
                pHtml++;  // skips the ?
                while(*pHtml != '\0')
                {
                  pHtml += GrabNextParam(pHtml, StringBuffer);
                  //Serial.print("Param found: ");
                  //Serial.println(StringBuffer);

                  // Parsing params withing the submitted URL
                  if(!strncmp("ssid", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    strcpy(ssid, &(StringBuffer[Index]));
                    //Serial.print("Updated SSID: ");
                    //Serial.println(ssid);
                  }

                  if(!strncmp("pass", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);

                    strcpy(password, &(StringBuffer[Index]));
                    Serial.print("Updated password: ");
                    Serial.println(password);
                  }

                  if(!strncmp("security", StringBuffer, 8))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "WPA2", 4))
                      UseSecurity = true;
                    else
                      UseSecurity = false;

                    Serial.print("Updated Security: ");
                    Serial.println(UseSecurity);
                  }

                  if(!strncmp("mode", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "station", 7))
                      APorStation = STATION_MODE;
                    else
                      APorStation = AP_MODE;

                    Serial.print("Updated Mode: ");
                    Serial.println(APorStation);
                  }

                  if(!strncmp("type", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "static", 6))
                      UseDHCP = false;
                    else
                      UseDHCP = true;

                    Serial.print("Updated DHCP: ");
                    Serial.println(UseDHCP);
                  }

                  if(!strncmp("ip", StringBuffer, 2))
                  {
                    Rank = atoi(&(StringBuffer[2]));
                    Index = SkipToValue(StringBuffer);
                    LocalIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("dip", StringBuffer, 3))
                  {
                    Rank = atoi(&(StringBuffer[3]));
                    Index = SkipToValue(StringBuffer);
                    DestIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("gw", StringBuffer, 2))
                  {
                    Rank = atoi(&(StringBuffer[1]));
                    Index = SkipToValue(StringBuffer);
                    GatewayIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("msk", StringBuffer, 3))
                  {
                    Rank = atoi(&(StringBuffer[1]));
                    Index = SkipToValue(StringBuffer);
                    SubnetMask[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("port", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    DestPort = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("id", StringBuffer, 2))
                  {
                    Index = SkipToValue(StringBuffer);
                    ModuleID = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("rate", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    SampleRate = atoi(&(StringBuffer[Index]));
                  }
                } // End of WHILE(PARSING PARAMETERS)
                // Save Params
                SaveFlashPrefs();
                Serial.println("Params updated and saved");
                PageToDisplay = PARAMS_WEB_PAGE;

              }
            }

            ////////////////////////////////////////////////////////////////////////
            // if you've gotten to the end of the line (received a newline
            // character) and the line is blank, the http request has ended,
            // so you can send a reply
            if (c == '\n' && currentLineIsBlank)
            {
              Serial.println("sending webpage");
              // send a standard http response header
              switch(PageToDisplay)
              {
              case CONFIG_WEB_PAGE:
                SendConfigWebPage();
                StayConnected = false;
                delay(100);
                break;

              case PARAMS_WEB_PAGE:
                SendParamsWebpage();
                StayConnected = false;
                delay(100);
                break;

              default:
                client.println(HTTP_RESPONSE_0);
                client.println(HTTP_RESPONSE_1);
                client.println(HTTP_RESPONSE_2);
                client.println();
                delay(100);
                StayConnected = false;
                break;
              }
            }
            if (c == '\n')
            {
              // you're starting a new line
              currentLineIsBlank = true;
            }
            else if (c != '\r')
            {
              // you've gotten a character on the current line
              currentLineIsBlank = false;
            }
          }
        }
        // give the web browser time to receive the data
        delay(100);

        // close the connection:
        client.stop();
        Serial.println("client disconnected");
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////
  // Incoming serial message (config, control)
  if(GrabSerialMessage())
  {
    //Serial.println("New serial message");
    ProcessSerial();
    FlagSerial = false;
  }
}


void Connect(void)
{
  if(!UseSecurity)
    WiFi.begin(ssid);
  else
    WiFi.begin(ssid, password); // If security is needed

  // if static IP - this still has problems with profiles and CC3200 API
  if(!UseDHCP)
    WiFi.config(LocalIP, GatewayIP, GatewayIP, SubnetMask);
}


void SendConfigWebPage(void)
{
  client.println(HTTP_RESPONSE_0);
  client.println(HTTP_RESPONSE_1);
  client.println(HTTP_RESPONSE_2);
  client.println();
  for(int i = 0 ; i < HTML_HEADER_CSS_SIZE ; i++)
  {
    client.println(pHTML_HEADER_CSS[i]);
  }
  // IRCAM logo
  for(int i = 0 ; i < IRCAM_LOGO_SIZE ; i++)
  {
    client.print(pIRCAM_LOGO[i]);
    delay(1);
  }
  client.println();
  client.println("</span></h1>\n<br/><br/><hr>");
  client.println("<h1>R-IoT Configuration Page</h1>");
  client.println("<p><table><tr><td><strong>Module Information</strong></td></tr></table>");
  sprintf(StringBuffer, "<table><tr><td>MAC: %02x:%02x%:%02x:%02x:%02x:%02x</td></tr>\0",mac[0], mac[1], mac[2], mac[3], mac[5], mac[5]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<tr><td>ID: %u</td></tr>\0",ModuleID);
  client.println(StringBuffer);

  client.println("<tr><td>Beta = \0");
  client.println(beta);
  client.println("</td></tr><br/>");

  sprintf(StringBuffer, "<tr><td>Firmware: %s</td></tr></table><br/><br/>\0", VERSION_DATE);
  client.println(StringBuffer);
  client.println("<table><tr><td><strong>Network Configuration</strong></td></tr></table>");
  client.println("<form method=\"GET\" action=\"params\"><table><tr><td>");

  client.print("<tr><td>WIFI MODE:</td><td><select name=\"mode\">");
  delay(10);
  if(APorStation == STATION_MODE)
    client.println("<option selected=\"selected\">station</option><option>AP</option></select></td></tr>");
  else
    client.println("<option>station</option><option selected=\"selected\">AP</option></select></td></tr>");
  delay(10);

  client.print("<tr><td>IP TYPE:</td><td><select name=\"type\">");
  delay(10);
  if(!UseDHCP)
    client.println("<option selected=\"selected\">static</option><option>DHCP</option></select></td></tr>");
  else
    client.println("<option>static</option><option selected=\"selected\">DHCP</option></select></td></tr>");
  delay(10);

  sprintf(StringBuffer,"<tr><td>SSID:</td><td><input type=\"text\" size=\"32\" maxlength=\"32\" name=\"ssid\" value=\"%s\"></td></tr>\0", ssid);
  client.println(StringBuffer);

  client.println("<tr><td>SECURITY:</td><td><select name=\"security\">");
  delay(10);
  if(!UseSecurity)
    client.println("<option selected=\"selected\">None</option><option>WPA2</option></select></td></tr>");
  else
    client.println("<option>None</option><option selected=\"selected\">WPA2</option></select></td></tr>");

  delay(10);
  client.print("<tr><td>PASSWD:</td><td><input type=\"text\" size=\"32\" maxlength=\"32\" name=\"pass\"");
  delay(10);
  sprintf(StringBuffer, " value=\"%s\"></td></tr>\0", password);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>IP:</td><td><input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi1\" value=\"%u\">.\0", LocalIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi2\" value=\"%u\">.\0", LocalIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi3\" value=\"%u\">.\0", LocalIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi4\" value=\"%u\"></td></tr>\0", LocalIP[3]);
  client.println(StringBuffer);

  sprintf(StringBuffer, "<tr><td>DEST IP:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"dip1\" value=\"%u\">.\0", DestIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip2\" value=\"%u\">.\0", DestIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip3\" value=\"%u\">.\0", DestIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip4\" value=\"%u\"></td></tr>\0", DestIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>GATEWAY:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"gw1\" value=\"%u\">.\0", GatewayIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw2\" value=\"%u\">.\0", GatewayIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw3\" value=\"%u\">.\0", GatewayIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw4\" value=\"%u\"></td></tr>\0", GatewayIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>MASK:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"msk1\" value=\"%u\">.\0", SubnetMask[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk2\" value=\"%u\">.\0", SubnetMask[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk3\" value=\"%u\">.\0", SubnetMask[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk4\" value=\"%u\"></td><br/></tr>\0", SubnetMask[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PORT:</td><td><input type=\"text\" size=\"4\" maxlength=\"6\" name=\"port\" value=\"%u\"></td></tr>\0", DestPort);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>ID:</td><td><input type=\"text\" size=\"4\" maxlength=\"3\" name=\"id\" value=\"%u\"></td></tr/>\0", ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SAMPLERATE:</td><td><input type=\"text\" size=\"4\" maxlength=\"4\" name=\"rate\" value=\"%u\"></td></tr>\0", SampleRate);
  client.println(StringBuffer);

  client.println("<tr><td><br/></td></tr><tr><td><input type=\"submit\" value=\"Submit\"></td></tr></table></form>");

  // output the value of each analog input pin
  BatteryVoltage = analogRead(BATT_MONITORING);
  client.print("Battery=");
  client.print((float)BatteryVoltage / 801.5);
  client.println(" volts");
  client.println("<br /></body></html>");
  delay(1);
  client.println();
}


void SendParamsWebpage(void)
{
  client.println(HTTP_RESPONSE_0);
  client.println(HTTP_RESPONSE_1);
  client.println(HTTP_RESPONSE_2);
  client.println();
  for(int i = 0 ; i < HTML_HEADER_CSS_SIZE ; i++)
  {
    client.println(pHTML_HEADER_CSS[i]);
  }
  // IRCAM logo
  for(int i = 0 ; i < IRCAM_LOGO_SIZE ; i++)
  {
    client.print(pIRCAM_LOGO[i]);
    delay(1);
  }
  client.println();
  client.println("</span></h1>\n<br/><br/><hr>");
  client.println("<h1>R-IoT Configuration *SAVED* - OK</h1>");
  client.println("<p><table><tr><td><strong>Module Information</strong></td></tr></table>");
  sprintf(StringBuffer, "<table><tr><td>MAC: %02x:%02x%:%02x:%02x:%02x:%02x</td></tr>\0",mac[0], mac[1], mac[2], mac[3], mac[5], mac[5]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<tr><td>ID: %u</td></tr><tr><td>\0",ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "Firmware: %s</td></tr></table><br/><br/>\0", VERSION_DATE);
  client.println(StringBuffer);
  client.println("<table><tr><td><strong>Network Configuration</strong></td></tr></table>");
  client.println("<table><tr><td>");

  client.print("<tr><td>WIFI MODE:</td>");
  if(APorStation == STATION_MODE)
    client.println("<td> Station </td></tr>");
  else
    client.println("<td> Access Point </td></tr>");
  client.print("<tr><td>IP TYPE:</td>");
  if(!UseDHCP)
    client.println("<td> Static IP </td></tr>");
  else
    client.println("<td> DHCP </td></tr>");
  sprintf(StringBuffer,"<tr><td>SSID:</td><td>%s</td></tr>\0", ssid);
  client.println(StringBuffer);
  client.println("<tr><td>SECURITY:</td>");
  sprintf(StringBuffer, "<td> %d </td></tr>\0", UseSecurity);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PASSWD:</td><td> %s </td></tr>\0", password);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>IP:</td><td>%u.%u.%u.%u</td></tr>\0", LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>DEST IP:</td><td>%u.%u.%u.%u</td></tr>\0", DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>GATEWAY:</td><td>%u.%u.%u.%u</td></tr>\0", GatewayIP[0], GatewayIP[1], GatewayIP[2], GatewayIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SUBNET MASK:</td><td>%u.%u.%u.%u</td></tr>\0", SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PORT:</td><td>%u</td></tr>\0", DestPort);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>MODULE ID:</td><td>%u</td></tr>\0", ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SAMPLE RATE:</td><td>%u</td></tr>\0", SampleRate);
  client.println(StringBuffer);

  client.println("<tr><td><br/></td></tr></table>");

  // output the value of each analog input pin
  BatteryVoltage = analogRead(BATT_MONITORING);
  client.print("Battery=");
  client.print((float)BatteryVoltage / 801.5);
  client.println(" volts");
  client.println("<br /></body></html>");
  delay(1);
  client.println();

}

unsigned int GrabNextParam(char *pBuffer, char *ParamString)
{
  unsigned int Len = 0;

  // URL / param string looks like below
  //  /params?type=static+IP&ssid=RIOT-36b9&type=None&pass=&ipi1=192&ipi2=168&ipi3=1&ipi4=1&dip
  // Looks up the & that split between the parameters
  // uses a fair search limit of 50 chars to avoid crashing in case something goes wrong
  while(Len < 50 && *pBuffer != '&' && *pBuffer != ' ')
  {
    ParamString[Len] = *pBuffer;
    pBuffer++;
    Len++;
  }
  pBuffer++; // skips the &
  ParamString[Len] = '\0';  // terminates the string
  Len++;
  return(Len);
}


void printWifiData() {
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address:
  printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print("Dest. IP Address: ");
  Serial.println(DestIP);

  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("NetMask: ");
  Serial.println(subnet);

  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("Gateway: ");
  Serial.println(gateway);

  printf("UDP/OSC Port=%u\n", DestPort);
  printf("Module ID=%u\n", ModuleID);
  printf("Sample Period (ms)=%u\n", SampleRate);
}


void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI) in dB:");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
}



//=====================================================================================================
// function gyroOffsetCalibration
//=====================================================================================================
//
// calibrate the zero of the gyroscope
//
void gyroOffsetCalibration(void)
{
    //gyro offset autocalibration
    // update min, max, sum and counter
    gyroOffsetAutocalMax[0] = max(gyroOffsetAutocalMax[0], GyroscopeX.Value);
    gyroOffsetAutocalMax[1] = max(gyroOffsetAutocalMax[1], GyroscopeY.Value);
    gyroOffsetAutocalMax[2] = max(gyroOffsetAutocalMax[2], GyroscopeY.Value);

    gyroOffsetAutocalMin[0] = min(gyroOffsetAutocalMin[0], GyroscopeX.Value);
    gyroOffsetAutocalMin[1] = min(gyroOffsetAutocalMin[1], GyroscopeY.Value);
    gyroOffsetAutocalMin[2] = min(gyroOffsetAutocalMin[2], GyroscopeZ.Value);

    gyroOffsetAutocalSum[0] += GyroscopeX.Value;
    gyroOffsetAutocalSum[1] += GyroscopeY.Value;
    gyroOffsetAutocalSum[2] += GyroscopeZ.Value;
    gyroOffsetAutocalCounter++;

    // if the max-min differences are above the threshold, reset the counter and values
    if((gyroOffsetAutocalMax[0]-gyroOffsetAutocalMin[0] > gyroOffsetAutocalThreshold)
       ||(gyroOffsetAutocalMax[1]-gyroOffsetAutocalMin[1] > gyroOffsetAutocalThreshold)
       ||(gyroOffsetAutocalMax[2]-gyroOffsetAutocalMin[2] > gyroOffsetAutocalThreshold))
    {
        resetGyroOffsetCalibration();
        resetAccOffsetCalibration();
    }

    // check if there are enough stable samples. If yes, update the offsets and the "calibrate" flag
    if(gyroOffsetAutocalCounter >= (gyroOffsetAutocalTime / 1000. * SampleRate) )
    { // update bias
      Serial.println("Gyro AutoCal");
      Serial.print("offsets: ");
      for(int i = 0 ; i < 3 ; i++)
      {
        gyro_bias[i] = (gyroOffsetAutocalSum[i] * 1.0)/ gyroOffsetAutocalCounter;
        gbias[i] = gRes * (float)gyro_bias[i];
        Serial.print(gyro_bias[i]);
        Serial.print(" ; ");
      }
      Serial.println();
      gyroOffsetCalDone = true;
      gyroOffsetCalElapsed = millis();
    }
}


// Reset gyro offset auto calibration / min / max
void resetGyroOffsetCalibration(void) {
  gyroOffsetAutocalCounter = 0;

  for(int i = 0 ; i<3 ; i++)
  {
    gyroOffsetAutocalMin[i] = 40000;
    gyroOffsetAutocalMax[i] = -40000;
    gyroOffsetAutocalSum[i] = 0;
    gyro_bias[i] = 0;
    gbias[i] = 0.0;
   }
}

// Reset gyro offset auto calibration / min / max
void resetMagOffsetCalibration(void) {

  for(int i = 0 ; i<3 ; i++)
  {
    magOffsetAutocalMin[i] = 40000;
    magOffsetAutocalMax[i] = -40000;
    mag_bias[i] = 0;
    mbias[i] = 0.0;
  }
}

// Reset gyro offset auto calibration / min / max
void resetAccOffsetCalibration(void) {
  for(int i = 0 ; i<3 ; i++)
  {
    accel_bias[i] = 0;
    abias[i] = 0.0;
    accOffsetAutocalSum[i] = 0;
  }
}


void CalibrateAccGyroMag(void)
{
  boolean QuitLoop = false;
  boolean LedState = 0;

  Serial.println("ACC+GYRO+MAG Calibration Started");
  Serial.println("Please place the module on a flat and stable surface");
  while(!digitalRead(SWITCH_INPUT))
    delay(20);

  resetAccOffsetCalibration();
  resetGyroOffsetCalibration();
  resetMagOffsetCalibration();

  while(!QuitLoop)
  {
    delay(10);
    ReadAccel();
    ReadGyro();

    magOffsetAutocalMax[0] = max(magOffsetAutocalMax[0], MagnetometerX.Value);
    magOffsetAutocalMax[1] = max(magOffsetAutocalMax[1], MagnetometerY.Value);
    magOffsetAutocalMax[2] = max(magOffsetAutocalMax[2], MagnetometerZ.Value);
    magOffsetAutocalMin[0] = min(magOffsetAutocalMin[0], MagnetometerX.Value);
    magOffsetAutocalMin[1] = min(magOffsetAutocalMin[1], MagnetometerY.Value);
    magOffsetAutocalMin[2] = min(magOffsetAutocalMin[2], MagnetometerZ.Value);

    LedState = 1 - LedState;
    digitalWrite(POWER_LED, LedState);
    if(!digitalRead(SWITCH_INPUT))
    {
      digitalWrite(POWER_LED, HIGH);
      Serial.println("Calibration interrupted");
      return;
    }

    //gyro offset autocalibration
    // update min, max, sum and counter
    gyroOffsetAutocalMax[0] = max(gyroOffsetAutocalMax[0], GyroscopeX.Value);
    gyroOffsetAutocalMax[1] = max(gyroOffsetAutocalMax[1], GyroscopeY.Value);
    gyroOffsetAutocalMax[2] = max(gyroOffsetAutocalMax[2], GyroscopeZ.Value);

    gyroOffsetAutocalMin[0] = min(gyroOffsetAutocalMin[0], GyroscopeX.Value);
    gyroOffsetAutocalMin[1] = min(gyroOffsetAutocalMin[1], GyroscopeY.Value);
    gyroOffsetAutocalMin[2] = min(gyroOffsetAutocalMin[2], GyroscopeZ.Value);

    gyroOffsetAutocalSum[0] += GyroscopeX.Value;
    gyroOffsetAutocalSum[1] += GyroscopeY.Value;
    gyroOffsetAutocalSum[2] += GyroscopeZ.Value;

    accOffsetAutocalSum[0] += AccelerationX.Value;
    accOffsetAutocalSum[1] += AccelerationY.Value;
    accOffsetAutocalSum[2] += AccelerationZ.Value - (int)(1./aRes);

    gyroOffsetAutocalCounter++;

    // if the max-min differences are above the threshold, reset the counter and values
    if((gyroOffsetAutocalMax[0]-gyroOffsetAutocalMin[0] > gyroOffsetAutocalThreshold)
       ||(gyroOffsetAutocalMax[1]-gyroOffsetAutocalMin[1] > gyroOffsetAutocalThreshold)
       ||(gyroOffsetAutocalMax[2]-gyroOffsetAutocalMin[2] > gyroOffsetAutocalThreshold))
    {
        resetGyroOffsetCalibration();
        resetAccOffsetCalibration();

    }

    // check if there are enough stable samples. If yes, update the offsets and the "calibrate" flag
    if(gyroOffsetAutocalCounter >= (gyroOffsetAutocalTime / 1000. * SampleRate) )
    { // update bias
      Serial.println("Gyro Stable");
      for(int i = 0 ; i < 3 ; i++)
      {
        gyro_bias[i] = (gyroOffsetAutocalSum[i] * 1.0)/ gyroOffsetAutocalCounter;
        gbias[i] = gRes * (float)gyro_bias[i];
        accel_bias[i] = (accOffsetAutocalSum[i] * 1.0)/ gyroOffsetAutocalCounter;
        abias[i] = aRes * (float)accel_bias[i];
      }
      QuitLoop = true;
    }
  }

  sprintf(StringBuffer, "*** FOUND Bias acc= %d %d %d", accel_bias[0], accel_bias[1], accel_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  sprintf(StringBuffer,"*** FOUND Bias gyro= %d %d %d", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);

  digitalWrite(POWER_LED, HIGH);
  int WinkCounter = 0;
  while(digitalRead(SWITCH_INPUT))  // Waits for the GP switch to proceed to mag calibration
  {
    delay(20);
    WinkCounter++;
    if(WinkCounter > 50)
    {
      digitalWrite(POWER_LED, LOW);
      delay(100);
      digitalWrite(POWER_LED, HIGH);
      WinkCounter = 0;
    }
  }

  resetMagOffsetCalibration();
  while(!digitalRead(SWITCH_INPUT))
    delay(20);
  delay(200);
  sprintf(StringBuffer, "*** Proceeding to MAG calibration - Max out all axis **** ");
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);

  QuitLoop = false;
  while(!QuitLoop)
  {
    delay(50);
    ReadMagneto();
    magOffsetAutocalMax[0] = max(magOffsetAutocalMax[0], MagnetometerX.Value);
    magOffsetAutocalMax[1] = max(magOffsetAutocalMax[1], MagnetometerY.Value);
    magOffsetAutocalMax[2] = max(magOffsetAutocalMax[2], MagnetometerZ.Value);
    magOffsetAutocalMin[0] = min(magOffsetAutocalMin[0], MagnetometerX.Value);
    magOffsetAutocalMin[1] = min(magOffsetAutocalMin[1], MagnetometerY.Value);
    magOffsetAutocalMin[2] = min(magOffsetAutocalMin[2], MagnetometerZ.Value);

    LedState = 1 - LedState;
    digitalWrite(POWER_LED, LedState);

    if(!digitalRead(SWITCH_INPUT))
    {
      digitalWrite(POWER_LED, HIGH);
      QuitLoop = true;
    }
  }
  while(!digitalRead(SWITCH_INPUT))
    delay(20);
  for(int i = 0 ; i < 3 ; i++)
  {
     mag_bias[i] = (magOffsetAutocalMax[i] + magOffsetAutocalMin[i]) / 2;
     mbias[i] = mRes * (float)mag_bias[i];
  }
  sprintf(StringBuffer,"*** FOUND Bias mag= %d %d %d", mag_bias[0], mag_bias[1], mag_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  SaveFlashPrefs();
}

void CalibrateMag(float *CalibratedArray, float *UnCalibratedArray)
{
  //calculation (bias was removed right after data acquisition)
  float result[3] = {
    0.,  0., 0.    };
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += rotation_matrix[i][j] * UnCalibratedArray[j];
  for (int i=0; i<3; ++i)
    CalibratedArray[i] = result[i];
}

boolean GrabSerialMessage(void)
{
  char TheChar;
  if (Serial.available())
  {
    while(Serial.available() && (SerialIndex < MAX_SERIAL))
    {
      TheChar = Serial.read();
      //Serial.print(TheChar);
      if((TheChar == '\n') || (TheChar == '\r'))
      {
        SerialBuffer[SerialIndex] = '\0';
        SerialIndex = 0;
        strcpy(StringBuffer, SerialBuffer);
        FlagSerial = true;
        return(true);
      }
      else
      {
        SerialBuffer[SerialIndex++] = TheChar;
      }
    }
    if(SerialIndex >= MAX_SERIAL)
      SerialIndex = 0;
  }
  return(false);
}

void ProcessSerial(void)
{
  unsigned char i, j;
  unsigned char Index;

  byte temp_ip[4];

  // Debug
  //printf("Cmd: %s - OK\n",StringBuffer);

  // Send current config to the configuration app
  if(!strncmp("cfgrequest",StringBuffer,10))
  {
    // Outputs all the configuration
    printf("%s %d\n", TEXT_DHCP, UseDHCP);
    printf("%s %s\n", TEXT_SSID, ssid);
    printf("%s %d\n", TEXT_WIFI_MODE, APorStation);
    printf("%s %d\n", TEXT_SECURITY, UseSecurity);
    printf("%s %s\n", TEXT_PASSWORD, password);
    printf("%s %u.%u.%u.%u\n", TEXT_OWNIP, LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3] );
    printf("%s %u.%u.%u.%u\n", TEXT_DESTIP, DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
    printf("%s %u.%u.%u.%u\n", TEXT_GATEWAY, GatewayIP[0], GatewayIP[1],GatewayIP[2],GatewayIP[3]);
    printf("%s %u.%u.%u.%u\n", TEXT_MASK, SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3] );
    printf("%s %u\n", TEXT_PORT, DestPort);
    printf("%s %u\n", TEXT_MASTER_ID, ModuleID);
    printf("%s %u\n", TEXT_SAMPLE_RATE, SampleRate);

    // All offsets as lists + rotation matrix
    printf("%s %d\n", TEXT_ACC_OFFSETX, accel_bias[0]);
    printf("%s %d\n", TEXT_ACC_OFFSETY, accel_bias[1]);
    printf("%s %d\n", TEXT_ACC_OFFSETZ, accel_bias[2]);

    printf("%s %d\n", TEXT_GYRO_OFFSETX, gyro_bias[0]);
    printf("%s %d\n", TEXT_GYRO_OFFSETY, gyro_bias[1]);
    printf("%s %d\n", TEXT_GYRO_OFFSETZ, gyro_bias[2]);

    printf("%s %d\n", TEXT_MAG_OFFSETX, mag_bias[0]);
    printf("%s %d\n", TEXT_MAG_OFFSETY, mag_bias[1]);
    printf("%s %d\n", TEXT_MAG_OFFSETZ, mag_bias[2]);

    printf("%s %f\n", TEXT_BETA, beta);

    printf("%s %f %f %f %f %f %f %f %f %f\n", TEXT_MATRIX, rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]);

    printf("refresh\n");
  }

  // Ping / Echo question/answer from the GUI
  else if(!strncmp("ping",StringBuffer,4))
  {
    printf("echo\n");	// a simple ASCII echo answer to let the GUI know the COM port is the right one
    return;
  }

  else if(!strncmp("savecfg",StringBuffer,7))	// Saves config to FLASH
  {
    SaveFlashPrefs();
    // Reboot is needed to use new settings
    //Reboot();
  }

  else if(!strncmp(TEXT_WIFI_MODE,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    sscanf(&StringBuffer[Index],"%d", &APorStation);
    return;
  }

  else if(!strncmp(TEXT_DHCP,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    sscanf(&StringBuffer[Index],"%d", &UseDHCP);
    return;
  }
  else if(!strncmp(TEXT_SSID,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    memset(ssid, '\0', sizeof(ssid));
    sscanf(&StringBuffer[Index],"%s", ssid);
    return;
  }
  else if(!strncmp(TEXT_OWNIP,StringBuffer,5))
  {
    Index = SkipToValue(StringBuffer);
    //printf("%s\n",&StringBuffer[Index]);
    ParseIP(&StringBuffer[Index], &LocalIP);
    //printf("own ip update %u.%u.%u.%u\n",pucIP_Addr[0], pucIP_Addr[1], pucIP_Addr[2], pucIP_Addr[3]);
    return;
  }
  else if(!strncmp(TEXT_DESTIP,StringBuffer,6))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &DestIP);
    return;
  }
  else if(!strncmp(TEXT_GATEWAY,StringBuffer,7))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &GatewayIP);
    return;
  }
  else if(!strncmp(TEXT_DNS,StringBuffer,3))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &GatewayIP);
    return;
  }
  else if(!strncmp(TEXT_MASK,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &SubnetMask);
    return;
  }
  else if(!strncmp(TEXT_PORT,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    DestPort = atoi(&StringBuffer[Index]);
    return;
  }
  else if(!strncmp(TEXT_MASTER_ID,StringBuffer,8))
  {
    Index = SkipToValue(StringBuffer);
    ModuleID = atoi(&StringBuffer[Index]);
    return;
  }
  else if(!strncmp(TEXT_SAMPLE_RATE,StringBuffer,10))
  {
    Index = SkipToValue(StringBuffer);
    SampleRate = atoi(&StringBuffer[Index]);
    if(SampleRate < MIN_SAMPLE_RATE)
      SampleRate = MIN_SAMPLE_RATE;

    if(SampleRate > MAX_SAMPLE_RATE)
      SampleRate = MAX_SAMPLE_RATE;

    return;
  }

  else if(!strncmp(TEXT_ACC_OFFSETX,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    accel_bias[0] = atoi(&StringBuffer[Index]);
    abias[0] = (float)accel_bias[0]*aRes;
    return;
  }
  else if(!strncmp(TEXT_ACC_OFFSETY,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    accel_bias[1] = atoi(&StringBuffer[Index]);
    abias[1] = (float)accel_bias[1]*aRes;
    return;
  }
  else if(!strncmp(TEXT_ACC_OFFSETZ,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    accel_bias[2] = atoi(&StringBuffer[Index]);
    abias[2] = (float)accel_bias[2]*aRes;
    return;
  }

  else if(!strncmp(TEXT_GYRO_OFFSETX,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[0] = atoi(&StringBuffer[Index]);
    gbias[0] = (float)gyro_bias[0]*gRes;
    return;
  }
  else if(!strncmp(TEXT_GYRO_OFFSETY,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[1] = atoi(&StringBuffer[Index]);
    gbias[1] = (float)gyro_bias[1]*gRes;
    return;
  }
  else if(!strncmp(TEXT_GYRO_OFFSETZ,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[2] = atoi(&StringBuffer[Index]);
    gbias[2] = (float)gyro_bias[2]*gRes;
    return;
  }

  else if(!strncmp(TEXT_MAG_OFFSETX,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    mag_bias[0] = atoi(&StringBuffer[Index]);
    mbias[0] = (float)mag_bias[0]*mRes;
    return;
  }
  else if(!strncmp(TEXT_MAG_OFFSETY,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    mag_bias[1] = atoi(&StringBuffer[Index]);
    mbias[1] = (float)mag_bias[1]*mRes;
    return;
  }
  else if(!strncmp(TEXT_MAG_OFFSETZ,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    mag_bias[2] = atoi(&StringBuffer[Index]);
    mbias[2] = (float)mag_bias[2]*mRes;
    return;
  }

  else if(!strncmp(TEXT_MATRIX,StringBuffer,5))
  {
    Index = SkipToValue(StringBuffer);
    for(int i = 0 ; i < 3 ; i++)
    {
      for(int j = 0 ; j < 3 ; j++)
      {
        rotation_matrix[i][j] = atof(&StringBuffer[Index]);
        Index = SkipToNextValue(StringBuffer, Index);
      }
    }
    //printf("Updated Matrix :\n");
    //printf("%s %f %f %f %f %f %f %f %f %f\n", TEXT_MATRIX, rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]);

  }

  else if(!strncmp(TEXT_BETA,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    beta = atof(&StringBuffer[Index]);
    return;
  }

  else if(!strncmp("defaults",StringBuffer,8))
  {
    // Re open in write mode
    SerFlash.close();
    if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE))
    {
      Serial.println("Restoring defaults");
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Please Reboot");
      while(1);
    }
    //Reboot();
  }
  else if(!strncmp("reboot",StringBuffer,6))
  {
    // Still needs work to find out how to reboot the chip
    // see hibernate modes)
    //Reboot();
  }

  else if(!strncmp("calibrate",StringBuffer,9))
  {
    Calibrate();
  }
}

void LoadParams(void)
{
  // Check if file exists
  if(!SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_READ))
  {
    SerFlash.close();
    // Creates the file
    Serial.println("Param File not found");
    if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_CREATE(512, _FS_FILE_OPEN_FLAG_COMMIT)))
    {
      // Re open in write mode
      Serial.println("Param File created and opened for writing");
      Serial.println("Restoring defaults");
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Please Reboot");
      while(1);
      // REBOOT NEEDED
    }
  }
  else
  {
    Serial.println("Found Param file, parsing");
    int FormatToken;
    GrabLine(StringBuffer);
    // checks if the file is properly formatted with the 0x55 header token
    if(strncmp(StringBuffer, "0x55", 4))
    {
      Serial.println("Restoring defaults");
      SerFlash.close();
      SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE);
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Reboot Device");
    }
    else
    {
      // Parses the preferences
      GrabLine(StringBuffer);
      strcpy(ssid, StringBuffer);
      APorStation = atoi(StringBuffer);
      printf("Wifi Mode = ");
      if(APorStation == STATION_MODE)
        printf("Station\n");
      else
        printf("Access Point\n");

      GrabLine(StringBuffer);
      strcpy(ssid, StringBuffer);
      GrabLine(StringBuffer);
      UseSecurity = atoi(StringBuffer);
      printf("WiFi Encryption = %d\n", UseSecurity);
      GrabLine(StringBuffer);
      strcpy(password, StringBuffer);
      printf("WiFi Password = %s\n", password);
      GrabLine(StringBuffer);
      UseDHCP = atoi(StringBuffer);
      printf("Use DHCP = %d\n", UseDHCP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &LocalIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &DestIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &GatewayIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &SubnetMask);
      GrabLine(StringBuffer);
      DestPort = atoi(StringBuffer);
      GrabLine(StringBuffer);
      ModuleID = atoi(StringBuffer);
      GrabLine(StringBuffer);
      SampleRate = atoi(StringBuffer);

      if(!SampleRate)
      {
        Serial.println("Min Sample Rate is 3 ms");
        SampleRate = MIN_SAMPLE_RATE;
      }
      deltat = (float)SampleRate / 1000.0f;

      // Loading calibration data
      GrabLine(StringBuffer);
      accel_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      accel_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      accel_bias[2] = atoi(StringBuffer);
      printf("Loaded Accel Offsets: %d %d %d\n", accel_bias[0], accel_bias[1], accel_bias[2]);

      GrabLine(StringBuffer);
      gyro_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      gyro_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      gyro_bias[2] = atoi(StringBuffer);
      printf("Loaded Gyro Offsets: %d %d %d\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

      GrabLine(StringBuffer);
      mag_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      mag_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      mag_bias[2] = atoi(StringBuffer);
      printf("Loaded Mag Offsets: %d %d %d\n", mag_bias[0], mag_bias[1], mag_bias[2]);

      // Calibration matrix
      for(int i = 0 ; i < 3 ; i++)
      {
        for(int j = 0 ; j < 3 ; j++)
        {
          GrabLine(StringBuffer);
          //printf("debug string = %s\n", StringBuffer);
          rotation_matrix[i][j] = atof(StringBuffer);
        }
      }
      printf("Loaded Calibration Matrix \n");
      printf("rotation_matrix[3][3] = \n");
      printf("[\n");
      for(int i=0 ; i<3 ; i++)
      {
        Serial.print("\t[");
        for(int j=0 ; j <3 ; j++)
        {
          Serial.print(rotation_matrix[i][j]);
          Serial.print("   ");
        }
        Serial.println("]");
      }
      printf("]\n\n");
      Serial.print("Madgwick Specifics: Beta =");
      Serial.println(beta);
      SerFlash.close();
    }
  }
}

void SaveFlashPrefs(void)
{
  int writeStatus;

  // File was opened for Read so far, re open in write mode
  SerFlash.close();
  if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE))
  {
    Serial.println("Saving prefs in FLASH");

    // Format token 0x55
    writeStatus = SerFlash.write((uint8_t*)("0x55\n"),5);

    // Station or AP mode
    sprintf(StringBuffer, "%d\n", APorStation);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // ssid
    sprintf(StringBuffer, "%s\n", ssid);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // use security ?
    sprintf(StringBuffer, "%d\n", UseSecurity);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%s\n", password);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%d\n", UseDHCP);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", GatewayIP[0], GatewayIP[1],GatewayIP[2],GatewayIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", SubnetMask[0], SubnetMask[1],SubnetMask[2],SubnetMask[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", DestPort);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", ModuleID);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", SampleRate);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Calibration data
    // Accel offset
    sprintf(StringBuffer, "%d\n", accel_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", accel_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", accel_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Gyro offset
    sprintf(StringBuffer, "%d\n", gyro_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", gyro_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", gyro_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Mag offset
    sprintf(StringBuffer, "%d\n", mag_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", mag_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", mag_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Calibration matrix
    for(int i = 0 ; i < 3 ; i++)
    {
      for(int j = 0 ; j < 3 ; j++)
      {
        ftoa(rotation_matrix[i][j], StringBuffer, 5); // No float support for sprintf, using a simple ftoa
        sprintf(StringBuffer, "%s\n", StringBuffer);
        //printf("debug string = %s\n", StringBuffer);
        writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
      }
    }

    SerFlash.close();
    Serial.println("Done");
  }
  else
    Serial.println("Error saving params");
}


// Assumes the file is already opened for writing
void RestoreDefaults(void)
{
  int writeStatus;

  // Clears the "file"
  //for(int i = 0 ; i < 500 ; i++)
  //  writeStatus = SerFlash.write((uint8_t*)0x00,1);

  // Format token 0x55
  writeStatus = SerFlash.write((uint8_t*)("0x55\n"),5);

  // Use Station mode
  sprintf(StringBuffer, "%d\n", STATION_MODE);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // ssid
  sprintf(StringBuffer, "%s\n", TheSSID);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Use Security = false
  sprintf(StringBuffer, "%d\n", false);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // default password = 12345678
  sprintf(StringBuffer, "12345678\n");
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Use DHCP = true
  sprintf(StringBuffer, "%d\n", true);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Fixed / static IP
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheLocalIP[0], TheLocalIP[1], TheLocalIP[2], TheLocalIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Destination computer IP
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheDestIP[0], TheDestIP[1], TheDestIP[2], TheDestIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheGatewayIP[0], TheGatewayIP[1],TheGatewayIP[2],TheGatewayIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheSubnetMask[0], TheSubnetMask[1],TheSubnetMask[2],TheSubnetMask[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheDestPort);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheID);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheSampleRate);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Calibration data
  // Accel offset
  sprintf(StringBuffer, "0\n");
  // reseting all offsets to zero
  for(int i=0 ; i < 9 ; i++)
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Calibration matrix (set to identity as default)
  for(int i = 0 ; i < 3 ; i++)
  {
    for(int j = 0 ; j < 3 ; j++)
    {
      ftoa(identity_matrix[i][j], StringBuffer, 5);
      sprintf(StringBuffer, "%s\n", StringBuffer);
      //printf("debug string = %s\n", StringBuffer);
      writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    }
  }
}

void EmptyString(char* TheString, int size)
{
  for(int i = 0 ; i < size ; i++)
    TheString[i] = '\0';
}

void PrintToOSC(char *StringMessage)
{
  // char LocalString[50];

  // if(WiFi.status() != WL_CONNECTED)
  //   return;
  // UdpPacket.beginPacket(DestIP, DestPort);
  // sprintf(LocalString, "/%d/message", ModuleID);
  // StringToOsc(&Message, LocalString, StringMessage);
  // UdpPacket.write((uint8_t*)Message.buf, Message.PacketSize);
  // UdpPacket.endPacket();
  // delay(20);
}


void Calibrate(void)
{

  unsigned char NextFlag = FALSE;
  unsigned char i;

  // TODO : do a long calibration and an estimate of the gyro drift based
  // on temperature.

  digitalWrite(POWER_LED, LOW);
  Serial.println(TEXT_CALIBRATION_00);
  Serial.println(TEXT_CALIBRATION_01);
  Serial.println(TEXT_CALIBRATION_02);
  Serial.println(TEXT_CALIBRATION_03);

  PrintToOSC(TEXT_CALIBRATION_00);
  PrintToOSC(TEXT_CALIBRATION_01);
  PrintToOSC(TEXT_CALIBRATION_02);
  PrintToOSC(TEXT_CALIBRATION_03);

  // Wait for the switch to be depressed or the next command to be sent
  // via serial
  while(digitalRead(SWITCH_INPUT) && !NextFlag)
  {
    NextFlag = WaitSerialNext();
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);
    delay(100);
  }

  NextFlag = false;
  // Wait for the switch to be released
  digitalWrite(POWER_LED, HIGH);
  while(!digitalRead(SWITCH_INPUT))
  {
    delay(20);
  }

  Serial.println(TEXT_CALIBRATION_20);
  PrintToOSC(TEXT_CALIBRATION_20);

  for(i=0 ; i < 3 ; i++)
  {
    gyro_bias[i] = 0;
    accel_bias[i] = 0;
  }

  // Average the accel and gyro offsets
  for(i=0;i<bias_samples;i++)    // We take some readings...
  {
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);
    ReadGyro();
    ReadAccel();
    gyro_bias[0] += GyroscopeX.Value;
    gyro_bias[1] += GyroscopeY.Value;
    gyro_bias[2] += GyroscopeZ.Value;
    accel_bias[0] += AccelerationX.Value;
    accel_bias[1] += AccelerationY.Value;
    accel_bias[2] += AccelerationZ.Value - (int)(1./aRes);
    delay(20);
  }

  accel_bias[0] /= bias_samples; // average the data
  accel_bias[1] /= bias_samples;
  accel_bias[2] /= bias_samples;

  gyro_bias[0] /= bias_samples; // average the data
  gyro_bias[1] /= bias_samples;
  gyro_bias[2] /= bias_samples;

  sprintf(StringBuffer, "*** FOUND Bias acc= %d %d %d", accel_bias[0], accel_bias[1], accel_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  sprintf(StringBuffer,"*** FOUND Bias gyro= %d %d %d", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);

  Serial.println(TEXT_CALIBRATION_21);
  Serial.println(TEXT_CALIBRATION_22);
  Serial.println(TEXT_CALIBRATION_03);

  PrintToOSC(TEXT_CALIBRATION_21);
  PrintToOSC(TEXT_CALIBRATION_22);
  PrintToOSC(TEXT_CALIBRATION_03);

  i = 0;
  while(digitalRead(SWITCH_INPUT) && !NextFlag)
  {
    NextFlag = WaitSerialNext();
    delay(200);
    Serial.print(".");
    i++;
    if(i > 30)
    {
      Serial.println();
      i = 0;
    }
  }
  Serial.println();

  // Wait for the switch to be released
  digitalWrite(POWER_LED, HIGH);
  while(!digitalRead(SWITCH_INPUT))
  {
    delay(20);
  }

  boolean QuitLoop = false;
  while(!QuitLoop)
  {
    // 200 ms sample rate, we don't need much there
    if((millis() - ElapsedTime) >= 200)
    {
      ElapsedTime = millis();
      digitalWrite(POWER_LED, HIGH);
      ReadMagneto();

      if(!digitalRead(SWITCH_INPUT) || NextFlag)
      {
        while(!digitalRead(SWITCH_INPUT))
          delay(20);

        NextFlag = false;

        switch(CalibrationStage)
        {
        case START :
          PrintSensorOrientation(TEXT_X_PLUS_0);
          PrintToOSC("Place the module in the cardinal positions:");
          PrintToOSC(TEXT_X_PLUS_0);
          break;

          // X axis
        case X_PLUS_0:
          StoreMag(Xplus0);
          sprintf(StringBuffer,"X+ 0deg : %d %d %d", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_X_PLUS_180);
          PrintToOSC(TEXT_X_PLUS_180);
          break;

        case X_PLUS_180:
          StoreMag(Xplus180);
          sprintf(StringBuffer,"X+ 180deg : %d %d %d", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_X_MINUS_0);
          PrintToOSC(TEXT_X_MINUS_0);
          break;

        case X_MINUS_0:
          StoreMag(Xminus0);
          sprintf(StringBuffer,"X- 0deg : %d %d %d", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_X_MINUS_180);
          PrintToOSC(TEXT_X_MINUS_180);
          break;

        case X_MINUS_180:
          StoreMag(Xminus180);
          sprintf(StringBuffer,"X- 180deg : %d %d %d", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Y_PLUS_0);
          PrintToOSC(TEXT_Y_PLUS_0);
          break;

          // Y axis
        case Y_PLUS_0:
          StoreMag(Yplus0);
          sprintf(StringBuffer,"Y+ 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Y_PLUS_180);
          PrintToOSC(TEXT_Y_PLUS_180);
          break;

        case Y_PLUS_180:
          StoreMag(Yplus180);
          sprintf(StringBuffer,"Y+ 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Y_MINUS_0);
          PrintToOSC(TEXT_Y_MINUS_0);
          break;

        case Y_MINUS_0:
          StoreMag(Yminus0);
          sprintf(StringBuffer,"Y- 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Y_MINUS_180);
          PrintToOSC(TEXT_Y_MINUS_180);
          break;

        case Y_MINUS_180:
          StoreMag(Yminus180);
          sprintf(StringBuffer,"Y- 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Z_PLUS_0);
          PrintToOSC(TEXT_Z_PLUS_0);
          break;

          // Z axis
        case Z_PLUS_0:
          StoreMag(Zplus0);
          sprintf(StringBuffer,"Z+ 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Z_PLUS_180);
          PrintToOSC(TEXT_Z_PLUS_180);
          break;

        case Z_PLUS_180:
          StoreMag(Zplus180);
          sprintf(StringBuffer,"Z+ 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Z_MINUS_0);
          PrintToOSC(TEXT_Z_MINUS_0);
          break;

        case Z_MINUS_0:
          StoreMag(Zminus0);
          sprintf(StringBuffer,"Z- 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          PrintSensorOrientation(TEXT_Z_MINUS_180);
          PrintToOSC(TEXT_Z_MINUS_180);
          break;

        case Z_MINUS_180:
          StoreMag(Zminus180);
          sprintf(StringBuffer,"Z- 180deg: %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          printf("%s\n");
          PrintToOSC(StringBuffer);
          Serial.println(TEXT_CALIB_END);
          PrintToOSC(TEXT_CALIB_END);
          break;

        default:
          break;
        }

        CalibrationStage++;
        if(CalibrationStage >= END)
        {
          sprintf(StringBuffer,"Computing Matrix and Bias");
          Serial.println(StringBuffer);
          PrintToOSC(StringBuffer);
          CalculateTransformationMatrix();

          //Display results
          //Transformation matrix
          printf("Matrix \n");
          printf("float rotation_matrix[3][3] = \n");
          printf("[\n");
          for(int i=0 ; i<3 ; i++)
          {
            Serial.print("\t[");
            for(int j=0 ; j <3 ; j++)
            {
              Serial.print(rotation_matrix[i][j]);
              Serial.print("   ");
            }
            Serial.println("]");
          }
          printf("]\n\n");
          // Bias

          sprintf(StringBuffer,"Bias");
          Serial.println(StringBuffer);
          PrintToOSC(StringBuffer);
          sprintf(StringBuffer, "mag_bias[3] = {%d, %d, %d}\n", mag_bias[0], mag_bias[1], mag_bias[2]);
          PrintToOSC(StringBuffer);
          digitalWrite(POWER_LED, HIGH);
          SaveFlashPrefs();
          sprintf(StringBuffer,"** CALIBRATION FINISHED - PREFS SAVED - PLEASE REBOOT **");
          printf("\n%s\n", StringBuffer);
          PrintToOSC(StringBuffer);

          QuitLoop = true;
        }
      }
      digitalWrite(POWER_LED, LOW);
      NextFlag = WaitSerialNext();
    }
  } // End of while(!QUIT CALIBRATION LOOP)
  while(1)
  {
    delay(250);
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);
  }
}

/////////////////////////////////////////////////////////////////////////////
// Linear algebra routines to solve the mag sensor matrix inversion

void PrintSensorOrientation(const char *msg)
{
  Serial.print("Place the sensor in the ");
  Serial.print(msg);
  Serial.println(" orientation");
  Serial.println("and press the switch or send the \"next\" command via serial");
}

void StoreMag(float *StorageVector)
{
  StorageVector[0] = (float)MagnetometerX.Value;
  StorageVector[1] = (float)MagnetometerY.Value;
  StorageVector[2] = (float)MagnetometerZ.Value;
}

void CalculateTransformationMatrix(void)
{
  //Axis X--------------------------------------------------------------------------------------------------
  float Xplus_center[3];
  //Centers of the circles
  Xplus_center[X] = (Xplus0[X] + Xplus180[X]) / 2;
  Xplus_center[Y] = (Xplus0[Y] + Xplus180[Y]) / 2;
  Xplus_center[Z] = (Xplus0[Z] + Xplus180[Z]) / 2;

  //Centers of the circles
  float Xminus_center[3];
  Xminus_center[X] = (Xminus0[X] + Xminus180[X]) / 2;
  Xminus_center[Y] = (Xminus0[Y] + Xminus180[Y]) / 2;
  Xminus_center[Z] = (Xminus0[Z] + Xminus180[Z]) / 2;

  //Vector from the center of minus circle to the center of plus circle
  float Xvector[3];
  Xvector[X] = Xplus_center[X] - Xminus_center[X];
  Xvector[Y] = Xplus_center[Y] - Xminus_center[Y];
  Xvector[Z] = Xplus_center[Z] - Xminus_center[Z];

  //Axis Y--------------------------------------------------------------------------------------------------
  float Yplus_center[3];
  //Centers of the circles
  Yplus_center[X] = (Yplus0[X] + Yplus180[X]) / 2;
  Yplus_center[Y] = (Yplus0[Y] + Yplus180[Y]) / 2;
  Yplus_center[Z] = (Yplus0[Z] + Yplus180[Z]) / 2;
  //Centers of the circles
  float Yminus_center[3];
  Yminus_center[X] = (Yminus0[X] + Yminus180[X]) / 2;
  Yminus_center[Y] = (Yminus0[Y] + Yminus180[Y]) / 2;
  Yminus_center[Z] = (Yminus0[Z] + Yminus180[Z]) / 2;
  //Vector from the center of minus circle to the center of plus circle
  float Yvector[3];
  Yvector[X] = Yplus_center[X] - Yminus_center[X];
  Yvector[Y] = Yplus_center[Y] - Yminus_center[Y];
  Yvector[Z] = Yplus_center[Z] - Yminus_center[Z];

  //Axis Z--------------------------------------------------------------------------------------------------
  float Zplus_center[3];
  //Centers of the circles
  Zplus_center[X] = (Zplus0[X] + Zplus180[X]) / 2;
  Zplus_center[Y] = (Zplus0[Y] + Zplus180[Y]) / 2;
  Zplus_center[Z] = (Zplus0[Z] + Zplus180[Z]) / 2;
  //Centers of the circles
  float Zminus_center[3];
  Zminus_center[X] = (Zminus0[X] + Zminus180[X]) / 2;
  Zminus_center[Y] = (Zminus0[Y] + Zminus180[Y]) / 2;
  Zminus_center[Z] = (Zminus0[Z] + Zminus180[Z]) / 2;
  //Vector from the center of minus circle to the center of plus circle
  float Zvector[3];
  Zvector[X] = Zplus_center[X] - Zminus_center[X];
  Zvector[Y] = Zplus_center[Y] - Zminus_center[Y];
  Zvector[Z] = Zplus_center[Z] - Zminus_center[Z];

  //Dividing by main value, for example for X axis - dividing by X coordinate, for Y axis by Y coordinate, for Z axis by Z coordinate
  rotation_matrix[0][0] = Xvector[0] / Xvector[0];
  rotation_matrix[0][1] = Yvector[0] / Yvector[1];
  rotation_matrix[0][2] = Zvector[0] / Zvector[2];
  rotation_matrix[1][0] = Xvector[1] / Xvector[0];
  rotation_matrix[1][1] = Yvector[1] / Yvector[1];
  rotation_matrix[1][2] = Zvector[1] / Zvector[2];
  rotation_matrix[2][0] = Xvector[2] / Xvector[0];
  rotation_matrix[2][1] = Yvector[2] / Yvector[1];
  rotation_matrix[2][2] = Zvector[2] / Zvector[2];

  //Matrix inversion
  Serial.println("Inverting Matrix");
  InvertMatrix(rotation_matrix, rotation_matrix_result);
  Serial.println("Invertion done");

  //Determinating the corrected by rotation matrix centers of the circles
  float CenterResultXplus[3], CenterResultXminus[3];
  float CenterResultYplus[3], CenterResultYminus[3];
  float CenterResultZplus[3], CenterResultZminus[3];

  MatrixVectorMultiply(rotation_matrix, Xplus_center, CenterResultXplus);
  MatrixVectorMultiply(rotation_matrix, Xminus_center, CenterResultXminus);
  MatrixVectorMultiply(rotation_matrix, Yplus_center, CenterResultYplus);
  MatrixVectorMultiply(rotation_matrix, Yminus_center, CenterResultYminus);
  MatrixVectorMultiply(rotation_matrix, Zplus_center, CenterResultZplus);
  MatrixVectorMultiply(rotation_matrix, Zminus_center, CenterResultZminus);

  //Determinating of the elipsoid center---------------------------------------------------------------------------
  float center[3];
  center[X] = (Xplus_center[0] + Xminus_center[0] + Yplus_center[0] + Yminus_center[0] + Zplus_center[0] + Zminus_center[0]) / 6;
  center[Y] = (Xplus_center[1] + Xminus_center[1] + Yplus_center[1] + Yminus_center[1] + Zplus_center[1] + Zminus_center[1]) / 6;
  center[Z] = (Xplus_center[2] + Xminus_center[2] + Yplus_center[2] + Yminus_center[2] + Zplus_center[2] + Zminus_center[2]) / 6;

  //Determinating of the radius of the future sphere-----------------------------------------------------------------------
  float x_length = fabs(Xplus_center[0] - Xminus_center[0])/2;
  float y_length = fabs(Yplus_center[1] - Yminus_center[1])/2;
  float z_length = fabs(Zplus_center[2] - Zminus_center[2])/2;
  float Xplus_0[3];
  Xplus_0[0] =  Xplus0[X];
  Xplus_0[1] =  Xplus0[Y];
  Xplus_0[2] =  Xplus0[Z];
  MatrixVectorMultiply(rotation_matrix, Xplus0, Xplus_0);
  float Yplus_0[3];
  Yplus_0[0] =  Yplus0[X];
  Yplus_0[1] =  Yplus0[Y];
  Yplus_0[2] =  Yplus0[Z];
  MatrixVectorMultiply(rotation_matrix, Yplus0, Yplus_0);
  float Zplus_0[3];
  Zplus_0[0] =  Zplus0[X];
  Zplus_0[1] =  Zplus0[Y];
  Zplus_0[2] =  Zplus0[Z];
  MatrixVectorMultiply(rotation_matrix, Zplus0, Zplus_0);
  float x_abs = sqrt(x_length * x_length + Xplus0[Y] * Xplus0[Y] + Xplus0[Z] * Xplus0[Z]);
  float y_abs = sqrt(Yplus0[X] * Yplus0[X] + y_length * y_length + Yplus0[Z] * Yplus0[Z]);
  float z_abs = sqrt(Zplus0[X] * Zplus0[X] + Zplus0[Y] * Zplus0[Y] + z_length * z_length);
  //sphere radius
  float sphere_radius = (x_abs + y_abs + z_abs) / 3;

  //Scales for the each axis------------------------------------------------
  //Diameter of the sphere
  float diameter = sphere_radius * 2;
  float kx = fabs(diameter / (Xplus_center[X] - Xminus_center[X]));
  float ky = fabs(diameter / (Yplus_center[Y] - Yminus_center[Y]));
  float kz = fabs(diameter / (Zplus_center[Z] - Zminus_center[Z]));

  //Multiplying elements of matrix by scales
  rotation_matrix[0][0] = rotation_matrix[0][0] * kx;
  rotation_matrix[0][1] = rotation_matrix[0][1] * ky;
  rotation_matrix[0][2] = rotation_matrix[0][2] * kz;
  rotation_matrix[1][0] = rotation_matrix[1][0] * kx;
  rotation_matrix[1][1] = rotation_matrix[1][1] * ky;
  rotation_matrix[1][2] = rotation_matrix[1][2] * kz;
  rotation_matrix[2][0] = rotation_matrix[2][0] * kx;
  rotation_matrix[2][1] = rotation_matrix[2][1] * ky;
  rotation_matrix[2][2] = rotation_matrix[2][2] * kz;

  //Bias
  mag_bias[X] = (int)center[X];
  mag_bias[Y] = (int)center[Y];
  mag_bias[Z] = (int)center[Z];

}

///////////////////////////////////////////////////////////////////////////
// Cheap and dirty algebra function designed for 3x3

void MatrixVectorMultiply(float matrixA[][3], float *vectorB, float *result)
{
  int aRows = 3;
  int aCols = 3;
  int bRows = 3;

  for (int i = 0; i < aRows; ++i) // each row of A
    for (int k = 0; k < aCols; ++k)
      result[i] += matrixA[i][k] * vectorB[k];
}

// A = matric to invert, x = resulting matrix
void InvertMatrix(float A[][3], float x[][3])
{
  int n = 3;
  //e will represent each column in the identity matrix
  float e[3];

  //x will hold the inverse matrix to be returned

  /*
  * solve will contain the vector solution for the LUP decomposition as we solve
   * for each vector of x.  We will combine the solutions into the float[][] array x.
   * */
  float solve[3];

  //Get the LU matrix and P matrix (as an array)
  float LU[3][3];
  int P[3];

  //Serial.println("Before LUP Decomposition");
  LUPDecomposition(A, LU, P);

  /*
  * Solve AX = e for each column e[i] of the identity matrix using LUP decomposition
   * */
  for (int i = 0; i < n; i++)
  {
    // Make an identity matrix column based on the index i
    // clears first
    for(int t = 0 ; t < 3 ; t++)
      e[t] = 0.0;
    e[i] = 1.0;

    LUPSolve(LU, P, e, solve);
    for (int j = 0; j < 3; j++)
    {
      x[j][i] = solve[j];
    }
  }
}


void LUPSolve(float LU[][3], int *pi, float *b, float *solution)
{
  int n = 2; // size-1
  float x[3];
  float y[3];
  float suml = 0;
  float sumu = 0;
  float lij = 0;

  /*
  * Solve for y using formward substitution
   * */
  for (int i = 0; i <= n; i++)
  {
    suml = 0;
    for (int j = 0; j <= i - 1; j++)
    {
      /*
      * Since we've taken L and U as a singular matrix as an input
       * the value for L at index i and j will be 1 when i equals j, not LU[i][j], since
       * the diagonal values are all 1 for L.
       * */
      if (i == j)
      {
        lij = 1;
      }
      else
      {
        lij = LU[i][j];
      }
      suml = suml + (lij * y[j]);
    }
    y[i] = b[pi[i]] - suml;
  }
  //Solve for x by using back substitution
  for (int i = n; i >= 0; i--)
  {
    sumu = 0;
    for (int j = i + 1; j <= n; j++)
    {
      sumu = sumu + (LU[i][j] * x[j]);
    }
    x[i] = (y[i] - sumu) / LU[i][i];
  }

}

void LUPDecomposition(float A[][3], float LU[][3], int  *pi)
{
  int n = 2;
  /*
   * pi represents the permutation matrix.  We implement it as an array
   * whose value indicates which column the 1 would appear.  We use it to avoid
   * dividing by zero or small numbers.
   * */
  float p = 0;
  int kp = 0;
  int pik = 0;
  int pikp = 0;
  float aki = 0;
  float akpi = 0;

  //Initialize the permutation matrix, will be the identity matrix
  for (int j = 0; j <= n; j++)
  {
    pi[j] = j;
  }

  // We make a copy of the matrix A into LU and work directly on LU for permutations
  // and decomposition.
  for(int i = 0 ; i <= n ; i++)
  {
    for(int j = 0 ; j <= n ; j++)
    {
      LU[i][j] = A[i][j];
    }
  }

  for (int k = 0; k <= n; k++)
  {
    /*
    * In finding the permutation matrix p that avoids dividing by zero
     * we take a slightly different approach.  For numerical stability
     * We find the element with the largest
     * absolute value of those in the current first column (column k).  If all elements in
     * the current first column are zero then the matrix is singluar and throw an
     * error.
     * */
    p = 0;
    for (int i = k; i <= n; i++)
    {
      if (fabs(LU[i][k]) > p)
      {
        p = fabs(LU[i][k]);
        kp = i;
      }
    }
    if (p == 0)
    {
      printf("Error : singular matrix");
    }
    /*
    * These lines update the pivot array (which represents the pivot matrix)
     * by exchanging pi[k] and pi[kp].
     * */
    pik = pi[k];
    pikp = pi[kp];
    pi[k] = pikp;
    pi[kp] = pik;

    /*
    * Exchange rows k and kpi as determined by the pivot
     * */
    for (int i = 0; i <= n; i++)
    {
      aki = LU[k][i];
      akpi = LU[kp][i];
      LU[k][i] = akpi;
      LU[kp][i] = aki;
    }

    /*
    * Compute the Schur complement
     * */
    for (int i = k + 1; i <= n; i++)
    {
      LU[i][k] = LU[i][k] / LU[k][k];
      for (int j = k + 1; j <= n; j++)
      {
        LU[i][j] = LU[i][j] - (LU[i][k] * LU[k][j]);
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////////////
// Awaits ASCII string "next" to keep going with the calibration process
boolean WaitSerialNext(void)
{
  if(GrabSerialMessage())
  {
    if(!strncmp("next", StringBuffer, 4))
      return(true);
  }

  // Also check the next command via UDP / OSC to have a fully wireless system
  if(WiFi.status() == WL_CONNECTED)
  {
    int packetSize = ConfigPacket.parsePacket();
    if (packetSize)
    {
        // Debug Info
         //Serial.print("Received packet of size ");
         //Serial.println(packetSize);
         //Serial.print("From ");
         //IPAddress remoteIp = ConfigPacket.remoteIP();
         //Serial.print(remoteIp);
         //Serial.print(", port ");
         //Serial.println(ConfigPacket.remotePort());

        // read the packet into packetBufffer
        int Index = 0;
        int len = ConfigPacket.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;          // Add a terminator in the buffer
        //Serial.println("Contents:");
        //Serial.println(packetBuffer);
        //Serial.println("Packet Len:");
        //Serial.println(len);
        // Actual parsing
        // Checks that's for the proper ID / module
        sprintf(StringBuffer, "/%u/\0",ModuleID);
        if(!strncmp(packetBuffer, StringBuffer, strlen(StringBuffer)))
        {  // that's for us
          char *pUDP = packetBuffer + strlen(StringBuffer);
          if(!strncmp(pUDP, "next", 4))
          {
            //Serial.println("received UDP next");
            return(true);
          }
        }
    }
  }
  return(false);
}







