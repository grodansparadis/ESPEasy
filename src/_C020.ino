#include "_CPlugin_Helper.h"
#ifdef USES_C020

// #######################################################################################################
// ########################### Controller Plugin 020: VSCP tcp/ip ######################################
// #######################################################################################################

#define CPLUGIN_020
#define CPLUGIN_ID_020         20
#define CPLUGIN_NAME_020       "VSCP MQTT"

#include "src/Commands/InternalCommands.h"
#include <ArduinoJson.h>
#include <vscp.h>

String CPlugin_020_prefix;                    // MQTT publish prefix
String CPlugin_020_guid;                      // VSCP guid
bool CPlugin_020_mqtt_retainFlag = false;     
unsigned long CPlugin_020_last_heartbeat = 0; // Timestamp last VSCP heartbeat
byte CPlugin_020_stream_index = 0;             // Used for string/data streams

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
      event->String1 += F("FF:FF:FF:FF:FF:FF:FF:FE:%mac%:00:00");
      event->String1 += F("/miso");

      // Subscribe
      event->String2 =  F("vscp/");
      event->String2 += F("FF:FF:FF:FF:FF:FF:FF:FE:%mac%:00:00");
      event->String2 += F("/mosi");
      break;
    }

    case CPlugin::Function::CPLUGIN_PROTOCOL_RECV:
    {
      // Find first enabled controller index with this protocol
      controllerIndex_t ControllerID = findFirstEnabledControllerWithId(CPLUGIN_ID_020);

      if (validControllerIndex(ControllerID)) {
        String pubname;
        DynamicJsonDocument root(512);
        deserializeJson(root, event->String2.c_str());

        if (!root.isNull())
        {
          unsigned int idx = root[F("idx")];
          float nvalue     = root[F("nvalue")];
          long  nvaluealt  = root[F("nvalue")];

          // const char* name = root["name"]; // Not used
          // const char* svalue = root["svalue"]; // Not used
          const char *svalue1 = root[F("svalue1")];

          // const char* svalue2 = root["svalue2"]; // Not used
          // const char* svalue3 = root["svalue3"]; // Not used
          const char *switchtype = root[F("switchType")]; // Expect "On/Off" or "dimmer"

          if (nvalue == 0) {
            nvalue = nvaluealt;
          }

          if ((int)switchtype == 0) {
            switchtype = "?";
          }

          for (taskIndex_t x = 0; x < TASKS_MAX; x++) {
            // We need the index of the controller we are: 0...CONTROLLER_MAX
            if (Settings.TaskDeviceEnabled[x] && (Settings.TaskDeviceID[ControllerID][x] == idx)) // get idx for our controller index
            {
              String action = "";

              switch (Settings.TaskDeviceNumber[x]) {
                case 1: // temp solution, if input switch, update state
                {
                  action  = F("inputSwitchState,");
                  action += x;
                  action += ',';
                  action += nvalue;
                  break;
                }
                case 29: // temp solution, if plugin 029, set gpio
                {
                  action = "";
                  int baseVar = x * VARS_PER_TASK;

                  if (strcasecmp_P(switchtype, PSTR("dimmer")) == 0)
                  {
                    int pwmValue = UserVar[baseVar];
                    action  = F("pwm,");
                    action += Settings.TaskDevicePin1[x];
                    action += ',';

                    switch ((int)nvalue)
                    {
                      case 0:  // Off
                        pwmValue         = 0;
                        UserVar[baseVar] = pwmValue;
                        break;
                      case 1: // On
                      case 2: // Update dimmer value
                        pwmValue         = 10 * atol(svalue1);
                        UserVar[baseVar] = pwmValue;
                        break;
                    }
                    action += pwmValue;
                  } else {
                    UserVar[baseVar] = nvalue;
                    action           = F("gpio,");
                    action          += Settings.TaskDevicePin1[x];
                    action          += ',';
                    action          += nvalue;
                  }
                  break;
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
        }
      }
      break;
    }

    case CPlugin::Function::CPLUGIN_PROTOCOL_SEND:
    {
      if (event->idx != 0)
      {
        String pubname;
        byte *p;
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

  String pubname = CPlugin_020_prefix + "/" + 
                          F("FF:FF:FF:FF:FF:FF:FF:FE:%mac%:00:00") + "/" + 
                          vscp_class + "/" + vscp_type;
  parseControllerVariables(pubname, event, false);

  return MQTTpublish(event->ControllerIndex, pubname.c_str(), json.c_str(), CPlugin_020_mqtt_retainFlag);
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
