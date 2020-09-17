#include "_CPlugin_Helper.h"
#ifdef USES_C020

// #######################################################################################################
// ########################### Controller Plugin 020: VSCP tcp/ip ######################################
// #######################################################################################################

#define CPLUGIN_020
#define CPLUGIN_ID_020         20
#define CPLUGIN_NAME_020       "VSCP MQTT"

#define C020_HEARTBEAT_LABEL   "heartbeat"
#define C020_ZONE_LABEL        "zone"
#define C020_SUBZONE0_LABEL    "subzone0"
#define C020_SUBZONE1_LABEL    "subzone1"
#define C020_SUBZONE2_LABEL    "subzone2"
#define C020_SUBZONE3_LABEL    "subzone3"
#define C020_SUBZONE4_LABEL    "subzone4"
#define C020_SUBZONE5_LABEL    "subzone5"
#define C020_SUBZONE9_LABEL    "subzone9"
#define C020_SUBZONE10_LABEL   "subzone10"
#define C020_SUBZONE12_LABEL   "subzone12"
#define C020_SUBZONE13_LABEL   "subzone13"
#define C020_SUBZONE14_LABEL   "subzone14"
#define C020_SUBZONE15_LABEL   "subzone15"

#include "src/Commands/InternalCommands.h"
#include <ArduinoJson.h>
#include <vscp.h>

String CPlugin_020_prefix;                    // MQTT publish prefix
String CPlugin_020_guid;                      // VSCP guid
byte CPlugin_020_stream_idx;                  // Current stream index
String CPlugin_020_stream;                    // Stream data
bool CPlugin_020_mqtt_retainFlag = false;     
unsigned long CPlugin_020_last_heartbeat = 0; // Timestamp last VSCP heartbeat
byte CPlugin_020_stream_index = 0;            // Used for string/data streams

// Forward declarations
bool CPlugin_020_send_vscp_event(struct EventStruct *event, uint16_t vscp_class, uint16_t vscp_type, DynamicJsonDocument &vscp_event );
void CPlugin_020_set_single_val( DynamicJsonDocument &vscp_event, struct EventStruct *event, byte nVar, byte offset);
bool CPlugin_020_SendStringAsStream(struct EventStruct *event,String &str);
String CPlugin_020_formatUserVar(struct EventStruct *event, byte rel_index);

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

    // if ((baudrate < 2400) || (baudrate > 115200)) {
    //   reset();
    // }
  }

  void reset() {
    // ZERO_FILL(DeviceEUI);
    // ZERO_FILL(DeviceAddr);
    // ZERO_FILL(NetworkSessionKey);
    // ZERO_FILL(AppSessionKey);
    bHeartbeat = true;
    module_zone   = 0;
    module_subzone0 = 0;
    module_subzone1 = 1;
    module_subzone2 = 2;
    module_subzone3 = 3;
    module_subzone4 = 4;
    module_subzone5 = 5;
    module_subzone9 = 9;
    module_subzone10 = 10;
    module_subzone12 = 12;
    module_subzone13 = 13;
    module_subzone14 = 14;
    module_subzone15 = 15;
  }

  bool bHeartbeat;
  byte module_zone;
  byte module_subzone0;
  byte module_subzone1;
  byte module_subzone2;
  byte module_subzone3;
  byte module_subzone4;
  byte module_subzone5;
  byte module_subzone9;
  byte module_subzone10;
  byte module_subzone12;
  byte module_subzone13;
  byte module_subzone14;
  byte module_subzone15;
};


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
      break;
    }

    case CPlugin::Function::CPLUGIN_GET_DEVICENAME:
    {
      string = F(CPLUGIN_NAME_020);
      break;
    }

    case CPlugin::Function::CPLUGIN_INIT:
    {
      success = init_mqtt_delay_queue(event->ControllerIndex, CPlugin_020_prefix, CPlugin_020_mqtt_retainFlag);
      CPlugin_020_guid = F("FF:FF:FF:FF:FF:FF:FF:FE:%mac%:00:00");
      parseControllerVariables(CPlugin_020_guid, event, false);
      CPlugin_020_last_heartbeat = 0;
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
      byte module_zone;
      byte module_subzone0;
      byte module_subzone1;
      byte module_subzone2;
      byte module_subzone3;
      byte module_subzone4;
      byte module_subzone5;
      byte module_subzone9;
      byte module_subzone10;
      byte module_subzone11;
      byte module_subzone12;
      byte module_subzone13;
      byte module_subzone14;
      byte module_subzone15;

      {
        // Keep this object in a small scope so we can destruct it as soon as possible again.
        std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

        if (!customConfig) {
          break;
        }
        LoadCustomControllerSettings(event->ControllerIndex, (byte *)customConfig.get(), sizeof(C020_ConfigStruct));
        customConfig->validate();
        bHeartbeat = customConfig->bHeartbeat;
        module_zone = customConfig->module_zone;
        module_subzone0 = customConfig->module_subzone0;
        module_subzone1 = customConfig->module_subzone1;
        module_subzone2 = customConfig->module_subzone2;
        module_subzone3 = customConfig->module_subzone3;
        module_subzone4 = customConfig->module_subzone4;
        module_subzone5 = customConfig->module_subzone5;
        module_subzone9 = customConfig->module_subzone9;
        module_subzone10 = customConfig->module_subzone10;
        module_subzone12 = customConfig->module_subzone12;
        module_subzone13 = customConfig->module_subzone13;
        module_subzone14 = customConfig->module_subzone14;
        module_subzone15 = customConfig->module_subzone15;
      }

      addTableSeparator(F("VSCP specific configuration"), 2, 3);
      {
        addFormCheckBox(F("Enable heartbeat event"), F(C020_HEARTBEAT_LABEL), true, false);

        addFormNumericBox(F("Module zone"), F(C020_ZONE_LABEL), module_zone, 0, 255);
        //addUnit(F("---"));
        //addFormNote(F("The zone is common for all operations"));

        addFormNumericBox(F("subzone GPIO-0 (D3) ⚠"), F(C020_SUBZONE0_LABEL), module_subzone0, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-1 (D10) TX0"), F(C020_SUBZONE1_LABEL), module_subzone1, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-2 (D4) ⚠"), F(C020_SUBZONE2_LABEL), module_subzone2, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-3 (D9) RX0"), F(C020_SUBZONE3_LABEL), module_subzone3, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-4 (D2)"), F(C020_SUBZONE4_LABEL), module_subzone4, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-5 (D1)"), F(C020_SUBZONE5_LABEL), module_subzone5, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-9 (D11) ⚠"), F(C020_SUBZONE9_LABEL), module_subzone9, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-10 (D12) ⚠"), F(C020_SUBZONE10_LABEL), module_subzone10, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-12 (D6)"), F(C020_SUBZONE12_LABEL), module_subzone12, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-13 (D7)"), F(C020_SUBZONE13_LABEL), module_subzone13, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-14 (D5)"), F(C020_SUBZONE14_LABEL), module_subzone14, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));

        addFormNumericBox(F("subzone GPIO-15 (D8)"), F(C020_SUBZONE15_LABEL), module_subzone15, 0, 254);
        //addUnit(F("---"));
        //addFormNote(F("255, for all subzones is not allowed."));
      }
      break;
    }

    case CPlugin::Function::CPLUGIN_WEBFORM_SAVE:
    {
      std::shared_ptr<C020_ConfigStruct> customConfig(new C020_ConfigStruct);

      if (customConfig) {
        customConfig->reset();
        //customConfig->bheartbeat = web_server.arg(F("bheartbeat"));
        customConfig->bHeartbeat = isFormItemChecked(F(C020_HEARTBEAT_LABEL));
        customConfig->module_zone = getFormItemInt(F(C020_ZONE_LABEL), customConfig->module_zone);
        customConfig->module_subzone0 = getFormItemInt(F(C020_SUBZONE0_LABEL), customConfig->module_subzone0);
        customConfig->module_subzone1 = getFormItemInt(F(C020_SUBZONE1_LABEL), customConfig->module_subzone1);
        customConfig->module_subzone2 = getFormItemInt(F(C020_SUBZONE2_LABEL), customConfig->module_subzone2);
        customConfig->module_subzone3 = getFormItemInt(F(C020_SUBZONE3_LABEL), customConfig->module_subzone3);
        customConfig->module_subzone4 = getFormItemInt(F(C020_SUBZONE4_LABEL), customConfig->module_subzone4);
        customConfig->module_subzone5 = getFormItemInt(F(C020_SUBZONE5_LABEL), customConfig->module_subzone5);
        customConfig->module_subzone9 = getFormItemInt(F(C020_SUBZONE9_LABEL), customConfig->module_subzone9);
        customConfig->module_subzone10 = getFormItemInt(F(C020_SUBZONE10_LABEL), customConfig->module_subzone10);
        customConfig->module_subzone12 = getFormItemInt(F(C020_SUBZONE12_LABEL), customConfig->module_subzone12);
        customConfig->module_subzone13 = getFormItemInt(F(C020_SUBZONE13_LABEL), customConfig->module_subzone13);
        customConfig->module_subzone14 = getFormItemInt(F(C020_SUBZONE14_LABEL), customConfig->module_subzone14);
        customConfig->module_subzone15 = getFormItemInt(F(C020_SUBZONE15_LABEL), customConfig->module_subzone15);        
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

              Serial.print("x  ");
              Serial.println(x);
              Serial.print("Settings.TaskDeviceNumber[x]");
              Serial.println(Settings.TaskDeviceNumber[x]);

              switch (Settings.TaskDeviceNumber[x]) {

                // https://espeasy.readthedocs.io/en/latest/Plugin/P001.html
                case 1: 
                {
                  if ( VSCP_CLASS1_CONTROL == vscp_class ) {
                    switch(vscp_type) {
                      
                      case VSCP_TYPE_CONTROL_TURNON:
                        Serial.println("VSCP_TYPE_CONTROL_TURNON");
                        if ( 3 == vscp_data.size() ) {
                          // GPIO,<gpio>,<state>
                          action  = F("gpio,");
                          action += (byte)vscp_data[2];  // GPIO
                          action += ',';
                          action += 1;
                        }
                        else {
                          String log = F("VSCP : Data count is wrong!");
                          addLog(LOG_LEVEL_ERROR, log);
                        }
                        break;

                      case VSCP_TYPE_CONTROL_TURNOFF:
                        // GPIO,<gpio>,<state>
                        action  = F("gpio,");
                        action += x;  // GPIO
                        action += ',';
                        action += 0;
                        break;

                      case VSCP_TYPE_CONTROL_TOGGLE_STATE:
                        // GPIOtoggle,<gpio>,<state>
                        action  = F("gpiotoggle,");
                        action += x;  // GPIO
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
                          action += x;  // GPIO
                          action += ",1,";
                          action += duration;
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
                          action += x;  // GPIO
                          action += ",0,";
                          action += duration;
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
            
            vscp_event[F("vscpData")][0] = 0; // Index
            vscp_event[F("vscpData")][1] = 0; // Zone
            vscp_event[F("vscpData")][2] = 0; // Subzone

            if (UserVar[event->BaseVarIndex] == 0) {
              vscp_type  = VSCP_TYPE_INFORMATION_ON;
            }
            else {
              vscp_type = VSCP_TYPE_INFORMATION_OFF;
            }
    
            return CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );

          case SENSOR_TYPE_DIMMER:

            if (UserVar[event->BaseVarIndex] == 0) {
              
              vscp_event[F("vscpData")][0] = 0; // Index
              vscp_event[F("vscpData")][1] = 0; // Zone
              vscp_event[F("vscpData")][2] = 0; // Subzone              
              
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
              vscp_event[F("vscpData")][0] = 0;   // Index 
              vscp_event[F("vscpData")][1] = 0;   // Zone 
              vscp_event[F("vscpData")][2] = 0;   // Subzone
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
     
      // Send heartbeat every minute and one initially
      if ( (!CPlugin_020_last_heartbeat) || 
           ((millis() - CPlugin_020_last_heartbeat) > 60000 )) {
        
        CPlugin_020_last_heartbeat = millis();
        Serial.println("Heartbeat");

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
        vscp_event[F("vscpData")][0] = 0; // Index
        vscp_event[F("vscpData")][1] = 0; // Zone
        vscp_event[F("vscpData")][2] = 0; // Subzone CPlugin_020_guid

        success = CPlugin_020_send_vscp_event(event, vscp_class, vscp_type, vscp_event );
      }

      // rssi

      // #if FEATURE_ADC_VCC
      //   vcc = ESP.getVcc() / 1000.0;
      // #endif
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
