/*
  This example connect to a remote VSCP host and display received events using 
  polling
*/

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>

#include <vscptcpclient.h>

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

// Prototypes
void dumpVscpEvent(vscpEventEx *pex);

// Polling version
vscpTcpClient vscp(vscp_server, vscp_port, espClient);


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

  // Give the system a chance to work.
  yield();

  // Connect to the VSCP remote host
  if ( VSCP_ERROR_SUCCESS == vscp.connect("admin","secret") ) {
    Serial.println("Connected to remote VSCP host");
  }
  else {
    Serial.println("Failed to connect to remote VSCP host");
  }

  // If just interested in specific events set filter here with
  // setRemoteFilter(vscpEventFilter &filter);
  // This sample filter below only recives events of type
  // CLASS1.MEASUREMENT, Type=6, Temperature

  /*
  vscpEventFilter filter;
  filter.mask_priority = 0;       // No filtering on priority
  filter.mask_class = 0xffff;     // Filtering on class
  filter.mask_type = 0xffff;      // Filtering on type
  memset(filter.mask_GUID,0,16);  // No filtering on GUID  

  filter.filter_priority = 0;     // No filtering (set to any value)
  filter.filter_class = 10;       // Class should be eual to 10
  filter.filter_type = 6;          // Type should be equal to 6
  memset(filter.filter_GUID,0,16);// No filtering on GUID (set to any value)

  if ( VSCP_ERROR_SUCCESS == vscp.setRemoteFilter(filter) ) {
    Serial.println("Filter set successfully");
    Serial.println("We now filter on CLASS=10m Type=6");
  }
  else {
    Serial.println("Failed to set filter");
  }
  */
}

///////////////////////////////////////////////////////////////////////////////
// loop
//

void loop() 
{ 
  vscpEventEx ex;
  uint16_t cnt;

  // Read event

  while(true) {
    
    if ( VSCP_ERROR_SUCCESS == vscp.checkRemoteBufferCount(&cnt) ) {
      
      Serial.print("Frames in remote buffer: ");
      Serial.println(cnt);

      if (cnt) {
        while ( VSCP_ERROR_SUCCESS == vscp.fetchRemoteEvent(ex) ) {            
          
          dumpVscpEvent(&ex);
          Serial.println("--------------------------------------------");
          yield();

        }
      }
      else {
        break;  // leavle while loop if not events to fetch
      }

    }

  }

  if (vscp.isConnected()) {
    Serial.println("Connected");  
  }
  else {
    Serial.println("Not connected"); 
  }

  delay(5000);
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


