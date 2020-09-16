/*
 This example have demos of most calls
*/


//#include <GDBStub.h>    // Uncomment for debugger

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>

#include <vscptcpclient.h>

static const char VSCP_ETH_PREFIX[] PROGMEM = "FF:FF:FF:FF:FF:FF:FF:FE:";

char vscp_guid[50];

const char *ssid = ""; // Enter the SSID of your WiFi Network.
const char *password =
    ""; // Enter the Password of your WiFi Network.

// MQTT connection credentials
const char *mqtt_server = "192.168.1.7";
const int16_t mqtt_port = 1883;
const char *mqtt_user = "vscp";
const char *mqtt_password = "secret";
const char *mqtt_topic = "vscp"; // vscp/guid/class/type

// VSCP connection credentials
const char *vscp_server = "192.168.1.7";
const int16_t vscp_port = 9598;
const char *vscp_user = "admin";
const char *vscp_password = "secret";

WiFiClient espClient;
// WiFiClientSecure espClient;

// Prototypes
void dumpVscpEvent(vscpEventEx *pex);
void callback(vscpEventEx *pex);

vscpTcpClient vscp(vscp_server, vscp_port, callback, espClient);


///////////////////////////////////////////////////////////////////////////////
// setup
//

void setup() {

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

  Serial.print("ESP Board MAC Address:  ");
  Serial.println("FF:FF:FF:FF:FF:FF:FF:FE:" + WiFi.macAddress() + ":00:00");
  Serial.println(vscp_guid);

  // Give system a chance to work.
  yield();

  // Connect to remote VSCP host
  if ( VSCP_ERROR_SUCCESS == vscp.connect("admin","secret") ) {
    Serial.printf("Connected to remote VSCP host");
  }
  else {
    Serial.printf("Failed to connect to remote VSCP host");
  }

  vscpEventEx ex;
  float tempC = 12.765;
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
  ex.sizeData = 4;
  ex.data[0] = 0xa8;          // Floating point, Sensor=0, Degrees Celsius
  byte *p1 = (byte *)&tempC;
  ex.data[1] = *(p1 + 3); // MSB first in VSCP
  ex.data[2] = *(p1 + 2);
  ex.data[3] = *(p1 + 1);
  ex.data[4] = *(p1 + 0);
  if ( VSCP_ERROR_SUCCESS == vscp.sendEventToRemote(ex) ) {
    Serial.println("Sent event");  
  }
  else {
    Serial.println("Failed to send  event");
  }

  ex.timestamp = millis()*1000;
  if ( VSCP_ERROR_SUCCESS == vscp.sendEventToRemote(ex) ) {
    Serial.println("Sent event");  
  }
  else {
    Serial.println("Failed to send  event");
  }

  char buf[80];

  // Noop
  if ( VSCP_ERROR_SUCCESS == vscp.doNoop() ) {
    Serial.println("Noop OK");  
  }
  else {
    Serial.println("Noop Failed");
  }

  // Remote server version
  if ( VSCP_ERROR_SUCCESS == vscp.getRemoteVersion(buf) ) {
    Serial.print("Version: ");  
    Serial.println(buf);
  }
  else {
    Serial.println("Version failed");
  }

  // Remote server version
  uint32_t chid;
  if ( VSCP_ERROR_SUCCESS == vscp.getRemoteChannelId(&chid) ) {
    Serial.print("Channel id: ");  
    Serial.println(chid);
  }
  else {
    Serial.println("getRemoteChannelId failed");
  }

  if ( VSCP_ERROR_SUCCESS == vscp.getRemoteGUID(ex.GUID) ) {
    Serial.print("Channel GUID: ");
    vscp.writeGuidToStr(buf, ex.GUID);
    Serial.println(buf);
  }
  else {
    Serial.println("getRemoteGUID failed");
  }

  ex.GUID[0] = 0x55;
  ex.GUID[1] = 0xAA;
  if ( VSCP_ERROR_SUCCESS == vscp.setRemoteGUID(ex.GUID) ) {
    Serial.println("setRemoteGUID success");
  }
  else {
    Serial.println("setRemoteGUID failed");
  }

  if ( VSCP_ERROR_SUCCESS == vscp.getRemoteGUID(ex.GUID) ) {
    Serial.print("Channel GUID: ");
    vscp.writeGuidToStr(buf, ex.GUID);
    Serial.println(buf);
  }
  else {
    Serial.println("getRemoteGUID failed");
  }

  // Read events with polling

  // for ( int i=0; i<10; i++) {

  //   uint16_t cnt;

    // while(true) {
    //   yield();
    //   if ( VSCP_ERROR_SUCCESS == vscp.checkRemoteBufferCount(&cnt) ) {
        
    //     Serial.print("Frames in remote buffer: ");
    //     Serial.println(cnt);

    //     if (cnt) {
    //       while ( VSCP_ERROR_SUCCESS == vscp.fetchRemoteEvent(ex) ) {            
            
    //         Serial.print("VSCP head=");
    //         Serial.println(ex.head);
            
    //         Serial.print("VSCP Class=");
    //         Serial.println(ex.vscp_class);
            
    //         Serial.print("VSCP Type=");
    //         Serial.println(ex.vscp_type);
            
    //         Serial.print("VSCP OBID=");
    //         Serial.println(ex.obid);

    //         vscp.writeDateTimeToStr(buf,ex);
    //         Serial.print("VSCP DateTime=");
    //         Serial.println(buf);

    //         Serial.print("VSCP Timestamp=");
    //         Serial.println(ex.timestamp);

    //         Serial.print("VSCP GUID=");
    //         vscp.writeGuidToStr(buf,ex.GUID);
    //         Serial.println(buf);

    //         Serial.print("VSCP sizeData=");
    //         Serial.println(ex.sizeData);

    //         Serial.print("VSCP Data: ");
    //         for ( int j=0; j<ex.sizeData; j++ ) {
    //           Serial.print("0x");
    //           itoa(ex.data[j],buf,16);
    //           Serial.print(buf);
    //           Serial.print(" ");
    //         }
    //         Serial.println();
    //         Serial.println("--------------------------------------");

    //         yield();
    //       }
    //     }
    //     else {
    //       break;
    //     }
  
    //   }

    // }

    //delay(5000);
  //}

  if (vscp.isConnected()) {
    Serial.println("Connected");  
  }
  else {
    Serial.println("Not connected"); 
  }

  // Serial.println("Disconnecting");
  // vscp.disconnect();

  // if (vscp.isConnected()) {
  //   Serial.println("Connected");  
  // }
  // else {
  //   Serial.println("Not connected"); 
  // }

  
}

///////////////////////////////////////////////////////////////////////////////
// loop
//

void loop() 
{ 
  vscp.loop(); 
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