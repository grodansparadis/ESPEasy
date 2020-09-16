/*
  This example connect to the network, log in to a remote VSCP host 
  and send a wake-up, on and a temperature and battery voltage event. 
  It shows how to send measurements in level I and Level II format.

  For temperature readings to work the example need a 1-wire temperature 
  sensor connected to GPIO 5. Need a 4k7 pullup.

  Ake Hedman, Grodans Paradis AB <akhe@grodansparadis.com>
*/

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>

#include <vscptcpclient.h>

// Tempsensor
#include <DallasTemperature.h>
#include <OneWire.h>

static const char VSCP_ETH_PREFIX[] PROGMEM = "FF:FF:FF:FF:FF:FF:FF:FE:";

char vscp_guid[50];

const char *ssid = ""; // Enter the SSID of your WiFi Network.
const char *password =
    ""; // Enter the Password of your WiFi Network.

// VSCP connection credentials
const char *vscp_server = "192.168.1.7";
const int16_t vscp_port = 9598;
const char *vscp_user = "admin";
const char *vscp_password = "secret";

WiFiClient espClient;
// WiFiClientSecure espClient;

// Define VSCP client object
vscpTcpClient vscp(vscp_server, vscp_port, espClient);

// GPIO where the DS18B20 is connected to
const int oneWireBus = 5;  //(D1 on NodeMCU)

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

ADC_MODE(ADC_VCC); // ADC connected to VCC

///////////////////////////////////////////////////////////////////////////////
// setup
//

void setup() {

  // Construct GUID based on MAC address
  strcpy(vscp_guid,"FF:FF:FF:FF:FF:FF:FF:FE:");
  strcat(vscp_guid,WiFi.macAddress().c_str());
  strcat(vscp_guid,":00:00");

  Serial.begin(115200);
  //gdbstub_init();     // Uncomment for debug
  delay(200); // Needs some time to stabilize

  // Set WiFi to station mode and disconnect from an AP if it was Previously
  // connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println();
  Serial.println();
  Serial.print("Connecting To: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("*");
  }

  Serial.println();
  Serial.println("WiFi Connected.");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("ESP Board GUID:  ");
  Serial.println(vscp_guid);

  Serial.print("RSSI:  ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

}

///////////////////////////////////////////////////////////////////////////////
// loop
//

void loop() 
{ 
  int rv;

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  Serial.print(tempC);
  Serial.println(" C");

  // Get VCC (battery) voltage.
  uint_fast16_t vcc = ESP.getVcc();
  Serial.print(vcc);
  Serial.println(" mV");

  // Give the system a chance to work.
  yield();

  // Connect to the VSCP remote host
  if ( VSCP_ERROR_SUCCESS == vscp.connect("admin","secret") ) {
    Serial.println("Connected to VSCP remote host");
  }
  else {
    Serial.println("Failed to connect to VSCP remote host");
  }

  char ver[32];
  vscp.getRemoteVersion(ver);
  Serial.print("Remote server version: ");
  Serial.println(ver);

  vscpEventEx ex;

  // Construct woken up event to tell the world we are on-line
  // https://docs.vscp.org/spec/latest/#/./class1.information?id=type29
  // To indicate that we came up from deep sleep
  memset(&ex,0,sizeof(ex));
  ex.timestamp = millis()*1000;
  ex.head = VSCP_PRIORITY_NORMAL |  // No rush
            VSCP_HEADER16_DUMB |    // We have no registers/dm etc
            VSCP_HEADER16_GUID_TYPE_STANDARD; // Normal GUID
  ex.obid = 0;
  ex.timestamp = millis()*1000; // Leave blank to let server fill in
  ex.vscp_class = VSCP_CLASS1_INFORMATION; 
  ex.vscp_type = VSCP_TYPE_INFORMATION_WOKEN_UP;
  vscp.readGuidFromStr(ex.GUID,vscp_guid);
  ex.sizeData = 3;
  ex.data[0] = 0;          // Index
  ex.data[1] = 0;          // Zone
  ex.data[2] = 0;          // Subzone
  
  if ( VSCP_ERROR_SUCCESS == (rv = vscp.sendEventToRemote(ex)) ) {
    Serial.println("Sent woken up event");  
  }
  else {
    Serial.print("Failed to send event, ");
    Serial.print("Error=");
    Serial.println(rv);
  }

  // Construct temperature event as a VSCP Level I event
  // Level I events are compact and intended for lower end 
  // devices
  // https://docs.vscp.org/spec/latest/#/./vscp_measurements?id=class2measurement_str
  memset(&ex,0,sizeof(ex));
  ex.timestamp = millis()*1000;
  ex.head = VSCP_PRIORITY_NORMAL |  // No rush
            VSCP_HEADER16_DUMB |    // We have no registers/dm etc
            VSCP_HEADER16_GUID_TYPE_STANDARD; // Normal GUID
  ex.obid = 0;
  ex.timestamp = millis()*1000; // Leave blank to let server fill in
  ex.vscp_class = VSCP_CLASS1_MEASUREMENT; 
  ex.vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
  vscp.readGuidFromStr(ex.GUID,vscp_guid);

  ex.sizeData = 5;
  // Datacoding is explained here https://docs.vscp.org/spec/latest/#/./vscp_measurements
  ex.data[0] = 0xA8; // Floating point, Sensor=0, Unit = Degrees Celsius
  // Floating point values (as all numbers) are stored MSB first in VSCP
  byte *p = (byte *)&tempC;  
  ex.data[1] = *(p + 3);  // MSB of 32-bit floating point number (Big-endian)
  ex.data[2] = *(p + 2);     
  ex.data[3] = *(p + 3);
  ex.data[4] = *(p + 0);  // MSB of 32-bit floating point number (Big-endian)   
  
  
  if ( VSCP_ERROR_SUCCESS == (rv = vscp.sendEventToRemote(ex)) ) {
    Serial.println("Sent temperature event");  
  }
  else {
    Serial.print("Failed to send event, ");
    Serial.print("Error=");
    Serial.println(rv);
  }

  // Construct temperature event as a VSCP Level II event
  // Level II events are intended for higher end devices &
  // software applications
  memset(&ex,0,sizeof(ex));
  ex.timestamp = millis()*1000;
  ex.head = VSCP_PRIORITY_NORMAL |  // No rush
            VSCP_HEADER16_DUMB |    // We have no registers/dm etc
            VSCP_HEADER16_GUID_TYPE_STANDARD; // Normal GUID
  ex.obid = 0;
  ex.timestamp = millis()*1000; // Leave blank to let server fill in
  ex.vscp_class = VSCP_CLASS2_MEASUREMENT_STR; 
  ex.vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
  vscp.readGuidFromStr(ex.GUID,vscp_guid);

  ex.data[0] = 0; // Sensor index
  ex.data[1] = 0; // Zone
  ex.data[2] = 0; // Sub zone
  ex.data[3] = 1; // Unit = 1 = Degrees Celsius for a temperature
  // Print the floating point value
  sprintf((char *)(ex.data+4),"%f",tempC);
  ex.sizeData = 4 + strlen((char *)ex.data+4) + 1; // We include the terminating zero  
  
  if ( VSCP_ERROR_SUCCESS == (rv = vscp.sendEventToRemote(ex)) ) {
    Serial.println("Sent level I temperature event");  
  }
  else {
    Serial.print("Failed to send event, ");
    Serial.print("Error=");
    Serial.println(rv);
  }


  // Construct event to report battery voltage
  // We use a compact level I measurement event
  // https://docs.vscp.org/spec/latest/#/./class1.measurement?id=type16
  memset(&ex,0,sizeof(ex));
  ex.timestamp = millis()*1000;
  ex.head = VSCP_PRIORITY_NORMAL |  // No rush
            VSCP_HEADER16_DUMB |    // We have no registers/dm etc
            VSCP_HEADER16_GUID_TYPE_STANDARD; // Normal GUID
  ex.obid = 0;
  ex.timestamp = millis()*1000; // Leave blank to let server fill in
  ex.vscp_class = VSCP_CLASS1_MEASUREMENT; 
  ex.vscp_type = VSCP_TYPE_MEASUREMENT_ELECTRICAL_POTENTIAL;
  vscp.readGuidFromStr(ex.GUID,vscp_guid);

  ex.data[0] = 0x89; // Normalized Integer, Sensor=0, Unit = 0 = Volt
  ex.data[1] = 0x83; // 0x83 - Move decimal point left three steps
                     //        Divide with 1000.sizeData = 4 + strlen((char *)ex.data+4) + 1; // We include the terminating zero  
  p = (byte *)&vcc;
  ex.data[2] = *(p + 1);  // MSB 
  ex.data[3] = *(p + 0);  // LSB
  ex.sizeData = 4;

  if ( VSCP_ERROR_SUCCESS == (rv = vscp.sendEventToRemote(ex)) ) {
    Serial.println("Sent battery voltage event");  
  }
  else {
    Serial.print("Failed to send event, ");
    Serial.print("Error=");
    Serial.println(rv);
  }

  // Disconnect from VSCP host
  vscp.disconnect();

  delay(30000);
}


/////////////////////////////////////////////////////////////////////
// dumpVscpEvent
//

void dumpVscpEvent(vscpEventEx *pex)
{
  char buf[80];

  Serial.print("VSCP head=");
  Serial.println(pex->head);
  
  Serial.print("VSCP Class=");
  Serial.println(pex->vscp_class);
  
  Serial.print("VSCP Type=");
  Serial.println(pex->vscp_type);
  
  Serial.print("VSCP OBID=");
  Serial.println(pex->obid);

  vscp.writeDateTimeToStr(buf, *pex);
  Serial.print("VSCP DateTime=");
  Serial.println(buf);

  Serial.print("VSCP Timestamp=");
  Serial.println(pex->timestamp);

  Serial.print("VSCP GUID=");
  vscp.writeGuidToStr(buf,pex->GUID);
  Serial.println(buf);

  Serial.print("VSCP sizeData=");
  Serial.println(pex->sizeData);

  Serial.print("VSCP Data: ");
  for ( int j=0; j<pex->sizeData; j++ ) {
    Serial.print("0x");
    itoa(pex->data[j],buf,16);
    Serial.print(buf);
    Serial.print(" ");
  }

  Serial.println();
}

/////////////////////////////////////////////////////////////////////
// callback
//

void callback(vscpEventEx *pex) 
{
  dumpVscpEvent(pex);
  Serial.println("-------------------------------------------------");
  delete pex;
}
