#include "_CPlugin_Helper.h"
#ifdef USES_C020

// #######################################################################################################
// ########################### Controller Plugin 020: VSCP tcp/ip ######################################
// #######################################################################################################

#define CPLUGIN_020
#define CPLUGIN_ID_020         20
#define CPLUGIN_NAME_020       "VSCP MQTT"

#define C020_HEARTBEAT_LABEL          "lblhb"
#define C020_VCC_LABEL                "lblvcc"
#define C020_RSSI_LABEL               "lblrssi"
#define C020_ZONE_LABEL               "lblmainzone"
#define C020_ONOFF_SUBZONE_LABEL      "lblonoffsz"
#define C020_HEARTBEAT_SUBZONE_LABEL  "lblhbsz"
#define C020_VCC_SUBZONE_LABEL        "lblvccsz"
#define C020_RSSI_SUBZONE_LABEL       "lblrssisz"
#define C020_SUBZONE_COUNT            16

// Flags for received events and subzone coding
#define C020_SUBZONE_EVENT_OPERATION_ONOFF     1
#define C020_SUBZONE_EVENT_OPERATION_PULSE     2
#define C020_SUBZONE_EVENT_OPERATION_PWM       3
#define C020_SUBZONE_EVENT_OPERATION_SERVO     4
#define C020_SUBZONE_EVENT_OPERATION_MONITOR   5
#define C020_SUBZONE_EVENT_OPERATION_TONE      6
#define C020_SUBZONE_EVENT_OPERATION_ENABLE    7

#include "src/Commands/InternalCommands.h"
#include <ArduinoJson.h>
#include <vscp.h>

struct C020_SubzoneStruct {
  byte gpio;
  byte subzone;
  int op;
};
struct C020_ConfigStruct
{
  C020_ConfigStruct() {
    reset();
  }

  void validate() {
    // ZERO_TERMINATE(DeviceEUI);
    // ZERO_TERMINATE(DeviceAddr);
    // ZERO_TERMINATE(NetworkSessionKey);
    // ZERO_TERMINATE(AppSessionKey);

    if (module_zone > 255) {
       reset();
    }

    for ( int i=0;i<C020_SUBZONE_COUNT;i++) {
      if (module_subzone[i].subzone > 254) {
         reset();
      }
    }
    
  }

  void reset() {
    // ZERO_FILL(DeviceEUI);
    bHeartbeat = true;
  #if FEATURE_ADC_VCC  
    bVCC = false;
  #endif  
    bRSSI = false;
    module_zone   = 0;
    subzone_onoff = 0;
    subzone_heartbeat = 0;
    subzone_vcc = 0;
    for ( int i=0;i<C020_SUBZONE_COUNT;i++) {
      module_subzone[i].gpio = i;
      module_subzone[i].subzone = 0;
      module_subzone[i].op = 0;
    }
  }

  bool bHeartbeat;
  #if FEATURE_ADC_VCC 
  bool bVCC;
  #endif
  bool bRSSI;
  byte module_zone;
  byte subzone_onoff;
  byte subzone_heartbeat;
  byte subzone_vcc;
  struct C020_SubzoneStruct module_subzone[C020_SUBZONE_COUNT];  
};

String CPlugin_020_prefix;                    // MQTT publish prefix
String CPlugin_020_guid;                      // VSCP guid
byte CPlugin_020_stream_idx;                  // Current stream index
String CPlugin_020_stream;                    // Stream data
bool CPlugin_020_mqtt_retainFlag = false;     
unsigned long CPlugin_020_last_heartbeat = 0; // Timestamp last VSCP heartbeat event
#if FEATURE_ADC_VCC 
unsigned long CPlugin_020_last_vcc = 0;       // Timestamp last VSCP VCC event
#endif
unsigned long CPlugin_020_last_rssi = 0;      // Timestamp last VSCP RSSI event
byte CPlugin_020_stream_index = 0;            // Used for string/data streams

// Forward declarations
bool CPlugin_020_send_vscp_event(struct EventStruct *event, uint16_t vscp_class, uint16_t vscp_type, DynamicJsonDocument &vscp_event );
void CPlugin_020_set_single_val( DynamicJsonDocument &vscp_event, struct EventStruct *event, byte nVar, byte offset);
bool CPlugin_020_SendStringAsStream(struct EventStruct *event,String &str);
String CPlugin_020_formatUserVar(struct EventStruct *event, byte rel_index);


bool CPlugin_020(CPlugin::Function function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPlugin::Function::CPLUGIN_PROTOCOL_ADD:
    {
      Protocol[++protocolCount].Number     = CPLUGIN_ID_020;
      Protocol[protocolCount].usesMQTT     = true;
      Protocol[protocolCount].usesTemplate = true;
      Protocol[protocolCount].usesAccount  = true;
      Protocol[protocolCount].usesPassword = true;
      Protocol[protocolCount].usesExtCreds = true;
      Protocol[protocolCount].defaultPort  = 1883;
      Protocol[protocolCount].usesID       = true;

      CPlugin_020_last_heartbeat = 0;
  #if FEATURE_ADC_VCC     
      CPlugin_020_last_vcc = 0;
  #endif    
      CPlugin_020_last_rssi = 0;
      break;
    }

    case CPlugin::Function::CPLUGIN_GET_DEVICENAME:
    {
      string = F(CPLUGIN_NAME_020);
      break;
    }

    case CPlugin::Function::CPLUGIN_INIT:
    {
      std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

      if (!customConfig) {
        return false;
      }

      LoadCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
      customConfig->validate();

      success = init_mqtt_delay_queue(event->ControllerIndex, CPlugin_020_prefix, CPlugin_020_mqtt_retainFlag);
      CPlugin_020_guid = F("FF:FF:FF:FF:FF:FF:FF:FE:%mac%:00:00");
      parseControllerVariables(CPlugin_020_guid, event, false);
      CPlugin_020_last_heartbeat = 0;
  #if FEATURE_ADC_VCC     
      CPlugin_020_last_vcc = 0;
  #endif    
      CPlugin_020_last_rssi = 0;
      // for (int i=0; i<C020_SUBZONE_COUNT;i++) {
      //   CPlugin_020_module_subzone[i].subzone = 0;
      // }
      break;
    }

    case CPlugin::Function::CPLUGIN_EXIT:
    {
      exit_mqtt_delay_queue();
      break;
    }

    case CPlugin::Function::CPLUGIN_PROTOCOL_TEMPLATE:
    {
      parseControllerVariables(CPlugin_020_guid, event, false);

      // Publish
      event->String1 =  F("vscp/");
      event->String1 += CPlugin_020_guid;
      event->String1 += F("/miso");

      // Subscribe
      event->String2 =  F("vscp/");
      event->String2 += CPlugin_020_guid;
      event->String2 += F("/mosi");
      break;
    }

    case CPlugin::Function::CPLUGIN_WEBFORM_LOAD:
    {
      bool bHeartbeat;
    #if FEATURE_ADC_VCC  
      bool bVCC;
    #endif  
      bool bRSSI;
      byte module_zone;
      byte subzone_onoff;
      byte subzone_heartbeat;
  #if FEATURE_ADC_VCC     
      byte subzone_vcc;
  #endif    
      struct C020_SubzoneStruct module_subzone[C020_SUBZONE_COUNT];

      {
        // Keep this object in a small scope so we can destruct it as soon as possible again.
        std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

        if (!customConfig) {
          break;
        }
        
        LoadCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
        customConfig->validate();
        bHeartbeat = customConfig->bHeartbeat;
  #if FEATURE_ADC_VCC       
        bVCC = customConfig->bVCC;
  #endif      
        bRSSI = customConfig->bRSSI;
        module_zone = customConfig->module_zone;
        subzone_onoff = customConfig->subzone_onoff;
        subzone_heartbeat = customConfig->subzone_heartbeat;
  #if FEATURE_ADC_VCC       
        subzone_vcc = customConfig->subzone_vcc;
  #endif      
        for ( int i=0; i<C020_SUBZONE_COUNT;i++) {
          module_subzone[i].gpio = customConfig->module_subzone[i].gpio;
          module_subzone[i].subzone = customConfig->module_subzone[i].subzone;
          module_subzone[i].op = customConfig->module_subzone[i].op;
        }
      }

      addTableSeparator(F("VSCP module configuration"), 2, 3);
      {
        addFormCheckBox(F("Enable heartbeat event"), F(C020_HEARTBEAT_LABEL), bHeartbeat, false);
  #if FEATURE_ADC_VCC 
        addFormCheckBox(F("Enable VCC event"), F(C020_VCC_LABEL), bVCC, false);
  #endif      

        addFormCheckBox(F("Enable RSSI event"), F(C020_RSSI_LABEL), bRSSI, false);

        addFormNumericBox(F("Module zone"), F(C020_ZONE_LABEL), module_zone, 0, 255);
        //addUnit(F("---"));
        addFormNote(F("Set to '255' for don't care on receive"));

        addFormNumericBox(F("On/Off/Monitor Subzone"), F(C020_ONOFF_SUBZONE_LABEL), subzone_onoff, 0, 255);
        addFormNumericBox(F("Heartbeat Subzone"), F(C020_HEARTBEAT_SUBZONE_LABEL), subzone_heartbeat, 0, 255);
      #if FEATURE_ADC_VCC   
        addFormNumericBox(F("VCC Subzone"), F(C020_VCC_SUBZONE_LABEL), subzone_vcc, 0, 255);
      #endif  
        //addFormSeparator(2);

        for ( int i=0; i<16; i++) {
          char lblbuf[50];         

          sprintf(lblbuf,(char *)F("VSCP incoming event handling for gpio%i"),i);
          addTableSeparator(lblbuf, 2, 3);

          sprintf(lblbuf,(char *)F("lblsz%i"),i);
          addFormNumericBox(F("subzone"), lblbuf, module_subzone[0].subzone, 0, 255);
          //addUnit(F("---"));
          //addFormNote(F("255, for all subzones is not allowed."));

          {
            String options[7] = { F("disabled"),F("on/off/toggle"),F("monitor"),F("pulse"),F("pwm"),F("servo"),F("tone") };
            int    values[7]  = { 0, C020_SUBZONE_EVENT_OPERATION_ONOFF, C020_SUBZONE_EVENT_OPERATION_MONITOR,C020_SUBZONE_EVENT_OPERATION_PULSE,C020_SUBZONE_EVENT_OPERATION_PWM,C020_SUBZONE_EVENT_OPERATION_SERVO,C020_SUBZONE_EVENT_OPERATION_TONE};
            sprintf(lblbuf,(char *)F("lblop%i"),i);
            addFormSelector_script(F("Operation"), lblbuf, i<12?7:6,
                               options, values, NULL, module_subzone[i].op,
                               F("joinChanged(this)")); 

          }
          html_add_script(F("document.getElementById('joinmethod').onchange();"), false);

          /*sprintf(lblbuf,(char *)F("lbloot%i"),i);
          addFormCheckBox(F("on/off/toggle"), lblbuf, false, false);
          
          sprintf(lblbuf,(char *)F("lbpulse%i"),i);
          addFormCheckBox(F("pulse"), lblbuf, false, false);
          
          if ( i < 15 ) { // Only available for 0-15
            sprintf(lblbuf,(char *)F("lbpwm%i"),i);
            addFormCheckBox(F("pwm"), lblbuf, false, false);
            
            sprintf(lblbuf,"lbservo%i",i);
            addFormCheckBox(F("servo"), lblbuf, false, false);
          }
          
          sprintf(lblbuf,(char *)F("lblmon%i"),i);
          addFormCheckBox(F("monitor"), lblbuf, false, false);

          if ( i>=12) {
            sprintf(lblbuf,(char *)F("lbltone%i"),i);
            addFormCheckBox(F("tone"), lblbuf, false, false);
          }*/

        }

        addFormSeparator(2);
/*
        addFormNumericBox(F("subzone GPIO-1 (D10) TX0"), F(C020_SUBZONE1_LABEL), CPlugin_020_module_subzone[1].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-2 (D4) ⚠"), F(C020_SUBZONE2_LABEL), CPlugin_020_module_subzone[2].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-3 (D9) RX0"), F(C020_SUBZONE3_LABEL), CPlugin_020_module_subzone[3].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-4 (D2)"), F(C020_SUBZONE4_LABEL), CPlugin_020_module_subzone[4].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-5 (D1)"), F(C020_SUBZONE5_LABEL), CPlugin_020_module_subzone[5].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-9 (D11) ⚠"), F(C020_SUBZONE9_LABEL), CPlugin_020_module_subzone[6].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-10 (D12) ⚠"), F(C020_SUBZONE10_LABEL), CPlugin_020_module_subzone[7].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-12 (D6)"), F(C020_SUBZONE12_LABEL), CPlugin_020_module_subzone[8].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-13 (D7)"), F(C020_SUBZONE13_LABEL), CPlugin_020_module_subzone[9].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-14 (D5)"), F(C020_SUBZONE14_LABEL), CPlugin_020_module_subzone[10].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-15 (D8)"), F(C020_SUBZONE15_LABEL), CPlugin_020_module_subzone[11].subzone, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));
*/        
      }
      break;
    }

    case CPlugin::Function::CPLUGIN_WEBFORM_SAVE:
    {
      std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

      if (customConfig) {
        customConfig->reset();
        customConfig->bHeartbeat = isFormItemChecked(F(C020_HEARTBEAT_LABEL));
      #if FEATURE_ADC_VCC   
        customConfig->bVCC = isFormItemChecked(F(C020_VCC_LABEL));
      #endif  
        customConfig->bRSSI = isFormItemChecked(F(C020_RSSI_LABEL));
        customConfig->module_zone = getFormItemInt(F(C020_ZONE_LABEL), customConfig->module_zone);
        customConfig->subzone_onoff = getFormItemInt(F(C020_ZONE_LABEL), customConfig->subzone_onoff);        
        customConfig->subzone_heartbeat = getFormItemInt(F(C020_ZONE_LABEL), customConfig->subzone_heartbeat);
        customConfig->subzone_vcc = getFormItemInt(F(C020_ZONE_LABEL), customConfig->subzone_vcc);        
        
        for (int i=0;i<C020_SUBZONE_COUNT;i++) {
          char lblbuf[20];
          sprintf(lblbuf,"lblsz%i",i);
          customConfig->module_subzone[i].subzone = /*CPlugin_020_module_subzone[i].subzone =*/ getFormItemInt(lblbuf, customConfig->module_subzone[i].subzone);
          
          sprintf(lblbuf,"lblop%i",i);
          customConfig->module_subzone[i].op = getFormItemInt(lblbuf, customConfig->module_subzone[i].op);

          /*
          sprintf(lblbuf,(char *)F("lbloot%i"),i);          
          if (isFormItemChecked(lblbuf)) {
            CPlugin_020_module_subzone[i].op |= C020_SUBZONE_EVENT_OPERATION_ONOFF;
          }
          
          sprintf(lblbuf,(char *)F("lbpulse%i"),i);
          if (isFormItemChecked(lblbuf)) {
            CPlugin_020_module_subzone[i].op |= C020_SUBZONE_EVENT_OPERATION_PULSE;
          }

          if ( i < 15 ) { // Only available for 0-15
            sprintf(lblbuf,(char *)F("lbpwm%i"),i);
            if (isFormItemChecked(lblbuf)) {
              CPlugin_020_module_subzone[i].op |= C020_SUBZONE_EVENT_OPERATION_PWM;
            }

            sprintf(lblbuf,"lbservo%i",i);
            if (isFormItemChecked(lblbuf)) {
              CPlugin_020_module_subzone[i].op |= C020_SUBZONE_EVENT_OPERATION_SERVO;
            }
          }

          sprintf(lblbuf,(char *)F("lblmon%i"),i);
          if (isFormItemChecked(lblbuf)) {
            CPlugin_020_module_subzone[i].op |= C020_SUBZONE_EVENT_OPERATION_MONITOR;
          }

          if ( i>=12) {
            sprintf(lblbuf,(char *)F("lbltone%i"),i);
            if (isFormItemChecked(lblbuf)) {
              CPlugin_020_module_subzone[i].op |= C020_SUBZONE_EVENT_OPERATION_TONE;
            }
          }*/
        }        
        
        SaveCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
      }
      break;
    }

    case CPlugin::Function::CPLUGIN_PROTOCOL_RECV:
    {
      // zone = ControllerID
      // subzone = gpio

      // Find first enabled controller index with this protocol  
      controllerIndex_t ControllerID = findFirstEnabledControllerWithId(CPLUGIN_ID_020);
      
      Serial.print("Incoming VSCP event ");
      Serial.println(ControllerID);

      if (validControllerIndex(ControllerID)) {

        DynamicJsonDocument vscp_event(512);
        deserializeJson(vscp_event, event->String2.c_str());

        if (!vscp_event.isNull())
        {

          uint16_t vscp_class = vscp_event[F("vscpClass")];
          uint16_t vscp_type = vscp_event[F("vscpType")];
          JsonArray vscp_data = vscp_event[F("vscpData")];

          // We handle 
          // VSCP_CLASS1.CONTROL/VSCP_TYPE_CONTROL_TURNON
          // VSCP_CLASS1.CONTROL/VSCP_TYPE_CONTROL_TURNOFF
          // VSCP_CLASS1.CONTROL/VSCP_TYPE_CONTROL_DIM_LAMPS
          // Set to input???
          // Pulse
          // PWM
          // Servo
          // Tone
          // Stream -> ESPEasy events 
          // Zone is id for C020 controller
          // Subzone is gpio 
          // https://www.letscontrolit.com/wiki/index.php/GPIO
          // https://espeasy.readthedocs.io/en/latest/Reference/Command.html

          for (taskIndex_t x = 0; x < TASKS_MAX; x++) {

            // We need the index of the controller we are: 0...CONTROLLER_MAX
            if (Settings.TaskDeviceEnabled[x] && (Settings.TaskDeviceID[ControllerID][x] == vscp_data[1])) // get idx for our controller index
            {
              String action = "";

              byte module_zone;
              struct C020_SubzoneStruct module_subzone[C020_SUBZONE_COUNT];

              {
                // Keep this object in a small scope so we can destruct it as soon as possible again.
                std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

                if (!customConfig) {
                  break;
                }
                
                LoadCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
                customConfig->validate();
                module_zone = customConfig->module_zone;
                for ( int i=0; i<C020_SUBZONE_COUNT;i++) {
                  module_subzone[i].gpio = customConfig->module_subzone[i].gpio;
                  module_subzone[i].subzone = customConfig->module_subzone[i].subzone;
                  module_subzone[i].op = customConfig->module_subzone[i].op;
                }
              }

              switch (Settings.TaskDeviceNumber[x]) {

                // https://espeasy.readthedocs.io/en/latest/Plugin/P001.html
                case 1: 
                {
                  if ( VSCP_CLASS1_CONTROL == vscp_class ) {
                    switch(vscp_type) {

                      // Check data, correct zone                      
                      if (( 3 > vscp_data.size() ) ||
                          (module_zone != (byte)vscp_data[2]) ) {
                        //String log = F("VSCP : Data count is wrong!");
                        //addLog(LOG_LEVEL_ERROR, log);    
                        break;
                      }

                      case VSCP_TYPE_CONTROL_TURNON:
                        Serial.println("VSCP_TYPE_CONTROL_TURNON");
                        for (int i=0;i<C020_SUBZONE_COUNT;i++) {
                          if (vscp_data[2] == module_subzone[i].subzone) {
                            // GPIO,<gpio>,<state>
                            action  = F("gpio,");
                            action += module_subzone[i].gpio;  // GPIO
                            action += F(",1");
                          }
                        }
                        break;

                      case VSCP_TYPE_CONTROL_TURNOFF:
                        for (int i=0;i<C020_SUBZONE_COUNT;i++) {
                          if (vscp_data[2] == module_subzone[i].subzone) {
                            // GPIO,<gpio>,<state>
                            action  = F("gpio,");
                            action += module_subzone[i].gpio;  // GPIO
                            action += F(",0");
                          }
                        }
                        break;

                      case VSCP_TYPE_CONTROL_TOGGLE_STATE:
                        for (int i=0;i<C020_SUBZONE_COUNT;i++) {
                          if (vscp_data[2] == module_subzone[i].subzone) {
                            // GPIOtoggle,<gpio>,<state>
                            action  = F("gpiotoggle,");
                            action += module_subzone[i].gpio;  // GPIO
                          }
                        }
                        break;

                      // milliseconds/seconds/minutes
                      case VSCP_TYPE_CONTROL_TIMED_PULSE_ON:
                        {
                          long int duration;
                          byte *p = (byte *)&duration;
                          p[0] = vscp_event[F("vscpData")][7];
                          p[1] = vscp_event[F("vscpData")][6];
                          p[2] = vscp_event[F("vscpData")][5];
                          p[3] = vscp_event[F("vscpData")][4];
                          
                          switch( (byte)vscp_event[F("vscpData")][3] & 0x07) {
                            
                            case 0: // microseconds
                              duration = duration/1000;
                              action  = F("pulse,");
                              break;

                            case 1: // milliseconds
                              if (duration <= 1000 ) {
                                action  = F("pulse,");
                              }
                              else if (duration <= 15000 ) {
                                action  = F("longpulsems,");
                              }
                              else {
                                duration /= 1000; // Max 999s
                                action  = F("longpulse,");
                              }
                              break;

                            case 2: // seconds
                              action  = F("longpulse,");
                              break;  
                          } // timing
                          for (int i=0;i<C020_SUBZONE_COUNT;i++) {
                            if (vscp_data[2] == module_subzone[i].subzone) {
                              action += module_subzone[i].gpio;  // GPIO
                              action += ",1,";
                              action += duration;
                            }
                          }                         
                        }
                        break;

                      case VSCP_TYPE_CONTROL_TIMED_PULSE_OFF: 
                        {
                          long int duration;
                          byte *p = (byte *)&duration;
                          p[0] = vscp_event[F("vscpData")][7];
                          p[1] = vscp_event[F("vscpData")][6];
                          p[2] = vscp_event[F("vscpData")][5];
                          p[3] = vscp_event[F("vscpData")][4];
                        
                          switch((byte)vscp_event[F("vscpData")][3] & 0x07) {
                          
                            case 0: // microseconds
                              duration = duration/1000;
                              action  = F("pulse,");
                              break;

                            case 1: // milliseconds
                              if (duration <= 1000 ) {
                                action  = F("pulse,");
                              }
                              else if (duration <= 15000 ) {
                                action  = F("longpulsems,");
                              }
                              else {
                                duration /= 1000; // Max 999s
                                action  = F("longpulse,");
                              }
                              break;

                            case 2: // seconds
                              action  = F("longpulse,");
                              break;  
                          } // timing
                          for (int i=0;i<C020_SUBZONE_COUNT;i++) {
                            if (vscp_data[2] == module_subzone[i].subzone) {
                              action += module_subzone[i].gpio;  // GPIO
                              action += ",0,";
                              action += duration;
                            }
                          }
                        }
                        break;

                      case VSCP_TYPE_CONTROL_PWM:
                        {
                          // PWM,<GPIO>,<state>
                          // PWM,<GPIO>,<state>,<duration> 

                          uint16_t time_on;
                          uint16_t time_off;
                          byte *p = (byte *)&time_on;
                          p[0] = vscp_event[F("vscpData")][5];
                          p[1] = vscp_event[F("vscpData")][4];
                          p = (byte *)&time_off;
                          p[0] = vscp_event[F("vscpData")][7];
                          p[1] = vscp_event[F("vscpData")][6];

                          action = "pwm,";
                          action += x;  // GPIO
                          action += ",";
                          if (0 == time_on) {
                            action += 0;    // leave off if no on time  
                          }
                          else if (0 == time_off) {
                            action += 1023; // leave on if no off time
                          }
                          else {
                            // Calculate duty cycle
                            action += ((float)time_on/(time_on + time_off)) * 1023;
                          }
                        }
                        break;

                      default:
                        // Ask for switch state TODO remove/change
                        action  = F("inputSwitchState,");
                        action += x;  // GPIO
                        action += ',';
                        action += 1;
                        break;
                    }
                  }
                  // Servo == position
                  // signed 16-bit integer -180 - +180
                  // float
                  else if ( (VSCP_CLASS1_SETVALUEZONE == vscp_class) &&
                            (VSCP_TYPE_MEASUREMENT_ANGLE == vscp_type)) {                    
                      action  = F("servo,");
                      action += 0;  // sensorindex
                      action += ',';
                      action += x;  // GPIO
                      action += ',';
                      action += 1; // TODO
                  }

                  
                }
                
#if defined(USES_P088) || defined(USES_P115)
                case 88: // Send heatpump IR (P088) if IDX matches
                case 115: // Send heatpump IR (P115) if IDX matches
                {
                  action = F("heatpumpir,");
                  action += svalue1; // svalue1 is like 'gree,1,1,0,22,0,0'
                  break;
                }
#endif // USES_P088 || USES_P115
                default:
                  break;
              }

              if (action.length() > 0) {
                ExecuteCommand_plugin(x, EventValueSource::Enum::VALUE_SOURCE_MQTT, action.c_str());

                // trigger rulesprocessing
                if (Settings.UseRules) {
                  struct EventStruct TempEvent;
                  TempEvent.TaskIndex = x;
                  parseCommandString(&TempEvent, action);
                  createRuleEvents(&TempEvent);
                }
              }
            }
          }
          LoadTaskSettings(event->TaskIndex);
        } // !null
      }
      break;
    }

    case CPlugin::Function::CPLUGIN_PROTOCOL_SEND:
    {
      byte module_zone;
      byte subzone_onoff;

      {
        // Keep this object in a small scope so we can destruct it as soon as possible again.
        std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

        if (!customConfig) {
          break;
        }
        
        LoadCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
        customConfig->validate();
        module_zone = customConfig->module_zone;
        subzone_onoff = customConfig->subzone_onoff;
      }

      if (event->idx != 0)
      {
        String pubname;
        uint16_t vscp_class;
        uint16_t vscp_type;
        DynamicJsonDocument vscp_event(512);

        vscp_event[F("vscpHead")]  = VSCP_HEADER16_DUMB;
        vscp_event[F("vscpObid")]  = 0;
        vscp_event[F("vscpDateTime")]  = "";  // GMT datetime - Let first interface fill in
        // DateTime alternative: Time set in node
        // NTP server should be enable and offset set to zero (GMT)
        //String dt = "%lcltime%";
        //parseControllerVariables(dt, event, false);
        // The middle space should be replaced with a 'T' to be correct
        // but parsing works anyway so step can be skiped
        //vscp_event[F("vscpDateTime")]  = dt;
        vscp_event[F("vscpTimeStamp")]  = millis()*1000;
        vscp_event[F("vscpGuid")]  = CPlugin_020_guid;

        switch (event->sensorType)
        {
          case SENSOR_TYPE_SWITCH:

            vscp_class  = VSCP_CLASS1_INFORMATION;
            
            vscp_event[F("vscpData")][0] = 0;             // Index
            vscp_event[F("vscpData")][1] = module_zone;   // Zone
            vscp_event[F("vscpData")][2] = subzone_onoff; // Subzone

            if (UserVar[event->BaseVarIndex] == 0) {
              vscp_type  = VSCP_TYPE_INFORMATION_ON;
            }
            else {
              vscp_type = VSCP_TYPE_INFORMATION_OFF;
            }
    
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_DIMMER:

            if (UserVar[event->BaseVarIndex] == 0) {
              
              vscp_event[F("vscpData")][0] = 0;             // Index
              vscp_event[F("vscpData")][1] = module_zone;   // Zone
              vscp_event[F("vscpData")][2] = subzone_onoff; // Subzone              
              
              vscp_class = VSCP_CLASS1_INFORMATION;
              vscp_type = VSCP_TYPE_INFORMATION_OFF;

              return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

            }
            else {

              // Level changed (possible alternative)
              /*
              vscp_class = VSCP_CLASS1_INFORMATION;
              vscp_type = VSCP_TYPE_INFORMATION_LEVEL_CHANGED;
              vscp_event[F("vscpData")][1] = 0;   // Zone 
              vscp_event[F("vscpData")][2] = 0;   // Subzone
              uint8_t val = (uint8_t)UserVar[event->BaseVarIndex];
              p = (byte *)&UserVar[event->BaseVarIndex];
              vscp_event[F("vscpData")][0] = *p;
              */

              // Big level changed (possible alternative)
              /*
              vscp_class = VSCP_CLASS1_INFORMATION;
              vscp_type = VSCP_TYPE_INFORMATION_BIG_LEVEL_CHANGED;
              vscp_event[F("vscpData")][0] = 0;             // Index 
              vscp_event[F("vscpData")][1] = module_zone;   // Zone 
              vscp_event[F("vscpData")][2] = 0;             // Subzone
              CPlugin_020_set_single_val( vscp_event, event, 0, 3);
              */

              // Relative change
              vscp_class = VSCP_CLASS1_MEASUREMENT;
              vscp_type = VSCP_TYPE_MEASUREMENT_RELATIVE_LEVEL;
              // Data is single precision floating point value, 
              // sensor = 0, unit = 0
              vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
              CPlugin_020_set_single_val( vscp_event, event, 0, 1);
              return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );
            }
            break;

          case SENSOR_TYPE_TEMP_HUM:

            // 1.) Temp (K), 2.) humidity (%)

            // Send temp

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
            
            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Kelvin) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: sensor = 0, unit = 1 (Degrees Celsius)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: sensor = 0, unit = 2 (Degrees Fahrenheit)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 0, 1);

            if (!CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event ) ) {
              return false;
            }

            // Prepare humidity value            
            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_HUMIDITY;

            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Relative percentage 0-100%) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            CPlugin_020_set_single_val( vscp_event, event, 1, 1);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );


          case SENSOR_TYPE_TEMP_BARO:
            // 1.) Temp (K), 2.) pressure (Pa)

            // Send temp

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
            
            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Kelvin) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: sensor = 0, unit = 1 (Degrees Celsius)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: sensor = 0, unit = 2 (Degrees Fahrenheit)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 0, 1);

            if (!CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event ) ) {
              return false;
            }

            // Send pressure

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_PRESSURE;

            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Pascal) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: unit = 1 (bar)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: unit = 2 (psi)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 1, 1);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_TEMP_EMPTY_BARO:
            
            // 1.) Temp (K), 2.) empty (no send) 3.) pressure (Pa)

            // Send temp

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
            
            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Kelvin) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: sensor = 0, unit = 1 (Degrees Celsius)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: sensor = 0, unit = 2 (Degrees Fahrenheit)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 0, 1);

            if (!CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event ) ) {
              return false;
            }

            // Send pressure

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_PRESSURE;

            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Pascal) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: unit = 1 (bar)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: unit = 2 (psi)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 2, 1);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_TEMP_HUM_BARO:

            // 1.) Temp (K), 2.) humidity (%) 3.) pressure (Pa)
            // Temp, humidity, pressure

            // Send temp

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
            
            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Kelvin) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: sensor = 0, unit = 1 (Degrees Celsius)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: sensor = 0, unit = 2 (Degrees Fahrenheit)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 0, 1);

            if (!CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event ) ) {
              return false;
            }

            // Prepare humidity value            
            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_HUMIDITY;

            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Relative percentage 0-100%) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            CPlugin_020_set_single_val( vscp_event, event, 1, 1);
            
            if (!CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event ) ) {
              return false;
            }

            // Send pressure

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_PRESSURE;

            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (Pascal) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: unit = 1 (bar)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: unit = 2 (psi)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 2, 1);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_WIND:

            // send wind speed

            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_SPEED;
            
            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (m/s) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            // Alternative: sensor = 0, unit = 1 (Kilometers per hour)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(1);
            // Alternative: sensor = 0, unit = 2 (Miles per hour)
            //vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE + VSCP_DATACODING_UNIT(2);

            CPlugin_020_set_single_val( vscp_event, event, 0, 1);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_SINGLE:
            
            // single value sensor, used for Dallas, BH1750, etc

            // This one is hard to map to VSCP. Send as general type that
            // must be interpreted on receiving side
            vscp_class = VSCP_CLASS1_MEASUREMENT;
            vscp_type = VSCP_TYPE_MEASUREMENT_GENERAL;
            
            // Data is single precision floating point value, 
            // sensor = 0, unit = 0 (whatever it is) 
            vscp_event[F("vscpData")][0] = VSCP_DATACODING_SINGLE;
            CPlugin_020_set_single_val( vscp_event, event, 0, 1);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_LONG:

            // RFID 
            vscp_class = VSCP_CLASS1_INFORMATION;
            vscp_type = VSCP_TYPE_INFORMATION_TOKEN_ACTIVITY;
            
            vscp_event[F("vscpData")][0] = 1;  // toke touched
            vscp_event[F("vscpData")][1] = 0;  // zone
            vscp_event[F("vscpData")][2] = 0;  // subzone
            vscp_event[F("vscpData")][3] = 0;  // frame index
       
            CPlugin_020_set_single_val( vscp_event, event, 0, 4);
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_DUAL:
          {
            // Don't know what we get here so send as stream and
            // let the receiving end interpret at will
            String strval  = CPlugin_020_formatUserVar(event, 0);
            strval += CPlugin_020_formatUserVar(event, 1);
            strval += CPlugin_020_formatUserVar(event, 2); 
            return CPlugin_020_SendStringAsStream(event,strval);            
          }
          break;

          case SENSOR_TYPE_TRIPLE:
          {
            // Don't know what we get here so send as stream and
            // let the receiving end interpret at will
            String strval  = CPlugin_020_formatUserVar(event, 0);
            strval += CPlugin_020_formatUserVar(event, 1);
            strval += CPlugin_020_formatUserVar(event, 2);
            return CPlugin_020_SendStringAsStream(event,strval);           
          }
          break;

          case SENSOR_TYPE_QUAD:
          {
            // Don't know what we get here so send as stream and
            // let the receiving end interpret at will
            String strval  = CPlugin_020_formatUserVar(event, 0);
            strval += CPlugin_020_formatUserVar(event, 1);
            strval += CPlugin_020_formatUserVar(event, 2);
            strval += CPlugin_020_formatUserVar(event, 3);
            return CPlugin_020_SendStringAsStream(event,strval);          
          }

          case SENSOR_TYPE_STRING:
          // String type data stored in the event->String2
          // We send as stream data
          CPlugin_020_SendStringAsStream(event,event->String2);
          default:
            // Do nothing
            break;
        }

      } // if ixd !=0
      else
      {
        String log = F("MQTT : IDX cannot be zero!");
        addLog(LOG_LEVEL_ERROR, log);
      }
      break;
    }

    case CPlugin::Function::CPLUGIN_TEN_PER_SECOND:
    {
      bool bHeartbeat = true;
    #if FEATURE_ADC_VCC  
      bool bVCC = false;
    #endif  
      bool bRSSI = false;
      byte module_zone = 0;
      byte subzone_heartbeat = 0;
  #if FEATURE_ADC_VCC     
      byte subzone_vcc = 0;
  #endif    

      {
        // Keep this object in a small scope so we can destruct it as soon as possible again.
        std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

        if (!customConfig) {
          break;
        }
        
        LoadCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
        customConfig->validate();
        bHeartbeat = customConfig->bHeartbeat;
  #if FEATURE_ADC_VCC      
        bVCC = customConfig->bVCC;
  #endif      
        bRSSI = customConfig->bRSSI;
        module_zone = customConfig->module_zone;
        subzone_heartbeat = customConfig->subzone_heartbeat;
  #if FEATURE_ADC_VCC      
        subzone_vcc = customConfig->subzone_vcc; 
  #endif       
      }

      // Send heartbeat every minute and one initially
      if ( bHeartbeat &&
           ((!CPlugin_020_last_heartbeat) || 
           ((millis() - CPlugin_020_last_heartbeat) > 60000 ))) {
        
        CPlugin_020_last_heartbeat = millis();
        Serial.println("Heartbeat"); // TODO remove

        uint16_t vscp_class;
        uint16_t vscp_type;
        DynamicJsonDocument vscp_event(512);

        vscp_event[F("vscpHead")]  = VSCP_HEADER16_DUMB;
        vscp_event[F("vscpObid")]  = 0;
        vscp_event[F("vscpDateTime")]  = "";  // Let interface set
        vscp_event[F("vscpTimeStamp")]  = millis()*1000;
        vscp_event[F("vscpGuid")]  = CPlugin_020_guid;

        vscp_class = VSCP_CLASS1_INFORMATION;
        vscp_type = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
        vscp_event[F("vscpData")][0] = (byte)WiFi.RSSI(); // RSSI
        vscp_event[F("vscpData")][1] = module_zone;       // Zone
        vscp_event[F("vscpData")][2] = subzone_heartbeat; // Subzone 

        if ( !CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event )) {
          return false;
        }
      }

      // rssi
      if ( bRSSI && 
           ((millis() - CPlugin_020_last_rssi) > 60000 )) {
        
        CPlugin_020_last_rssi = millis();
        Serial.println("RSSI"); // TODO remove

        uint16_t vscp_class;
        uint16_t vscp_type;
        DynamicJsonDocument vscp_event(512);

        vscp_event[F("vscpHead")]  = VSCP_HEADER16_DUMB;
        vscp_event[F("vscpObid")]  = 0;
        vscp_event[F("vscpDateTime")]  = "";  // Let interface set
        vscp_event[F("vscpTimeStamp")]  = millis()*1000;
        vscp_event[F("vscpGuid")]  = CPlugin_020_guid;

        vscp_class = VSCP_CLASS1_DATA;
        vscp_type = VSCP_TYPE_DATA_SIGNAL_QUALITY;
        vscp_event[F("vscpData")][0] = 2; // Unit is dBm
        vscp_event[F("vscpData")][1] = VSCP_DATACODING_INTEGER; // Integer,sensorindex=0,unit=0
        long rssi = WiFi.RSSI();
        byte *p = (byte *)rssi;
        vscp_event[F("vscpData")][2] = *(p+3); // MSB first
        vscp_event[F("vscpData")][3] = *(p+2);
        vscp_event[F("vscpData")][4] = *(p+1); 
        vscp_event[F("vscpData")][5] = *(p+0);

        if ( !CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event )) {
          return false;
        }
      }

  #if FEATURE_ADC_VCC
      if ( bVCC && 
           ((millis() - CPlugin_020_last_vcc) > 60000 )) {
        
        float vcc = ESP.getVcc() / 1000.0;
        byte *p = (byte *)&vcc;

        CPlugin_020_last_vcc = millis();

        uint16_t vscp_class;
        uint16_t vscp_type;
        DynamicJsonDocument vscp_event(512);

        vscp_event[F("vscpHead")]  = VSCP_HEADER16_DUMB;
        vscp_event[F("vscpObid")]  = 0;
        vscp_event[F("vscpDateTime")]  = "";  // Let interface set
        vscp_event[F("vscpTimeStamp")]  = millis()*1000;
        vscp_event[F("vscpGuid")]  = CPlugin_020_guid;

        vscp_class = VSCP_CLASS1_MEASUREMENT;
        vscp_type = VSCP_TYPE_DATA_SIGNAL_QUALITY;
        vscp_event[F("vscpData")][0] = VSCP_TYPE_MEASUREMENT_ELECTRICAL_POTENTIAL; // Integer,sensorindex=0,unit=0 (Volts)
        vscp_event[F("vscpData")][1] = *(p+3); // MSB first
        vscp_event[F("vscpData")][2] = *(p+2);
        vscp_event[F("vscpData")][3] = *(p+1); 
        vscp_event[F("vscpData")][4] = *(p+0);

        if ( !CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event )) {
          return false;
        }
      }
  #endif

      break;
    }

    case CPlugin::Function::CPLUGIN_FLUSH:
    {
      processMQTTdelayQueue();
      delay(0);
      break;
    }

    default:
      break;

  }
  return success;
}

void CPlugin_020_set_single_val( DynamicJsonDocument &vscp_event, struct EventStruct *event, byte nVar, byte offset) {
  byte *p = (byte *)&UserVar[event->BaseVarIndex + nVar];
  vscp_event[F("vscpData")][offset + 0] = *(p+3); // VSCP send MSB first
  vscp_event[F("vscpData")][offset + 1] = *(p+2);
  vscp_event[F("vscpData")][offset + 2] = *(p+1);
  vscp_event[F("vscpData")][offset + 3] = *(p+0);
}

bool CPlugin_020_send_vscp_event(struct EventStruct *event, uint16_t vscp_class, uint16_t vscp_type, DynamicJsonDocument &vscp_event )
{
  String json;
  serializeJson(vscp_event, json);

  vscp_event[F("vscpClass")] = vscp_class;
  vscp_event[F("vscpType")] = vscp_type;

  String topic = CPlugin_020_prefix + "/" + 
                          CPlugin_020_guid + "/" + 
                          vscp_class + "/" + vscp_type;
  topic = CPlugin_020_prefix + "/" + 
            vscp_class + "/" + vscp_type;                    
  parseControllerVariables(topic, event, false);

  return MQTTpublish(event->ControllerIndex, topic.c_str(), json.c_str(), CPlugin_020_mqtt_retainFlag);
}

// Send a string as a stream of events
bool CPlugin_020_SendStringAsStream(struct EventStruct *event,String &str)
{
  uint16_t len = str.length();

  uint16_t vscp_class;
  uint16_t vscp_type;
  DynamicJsonDocument vscp_event(512);

  vscp_event[F("vscpHead")]  = VSCP_HEADER16_DUMB;
  vscp_event[F("vscpObid")]  = 0;
  vscp_event[F("vscpDateTime")]  = "";  // Let interface set
  vscp_event[F("vscpTimeStamp")]  = millis()*1000;
  vscp_event[F("vscpGuid")]  = CPlugin_020_guid;

  vscp_class = VSCP_CLASS1_INFORMATION;
  vscp_type = VSCP_TYPE_INFORMATION_STREAM_DATA;
        
  while (len) {
    JsonArray array;
    
    array.add(CPlugin_020_stream_index);
    for ( byte i=0;i<=7;i++) {
      array.add(str.charAt(str.length()-len));
      len--;
      if ( !len ) break;
    }

    vscp_event[F("vscpData")] = array;
    if ( !CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event ) ) {
      return false;
    }

    CPlugin_020_stream_index++;
  }

  return true;
}


String CPlugin_020_formatUserVar(struct EventStruct *event, byte rel_index) {
  String text = formatUserVarNoCheck(event, rel_index);

  text += ';';
  return text;
}

#endif // ifdef USES_C020
