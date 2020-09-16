/*
 FILE: vscptcpclient.h

 This file is part of the VSCP (https://www.vscp.org)

 The MIT License (MIT)

 Copyright © 2000-2020 Ake Hedman, Grodans Paradis AB
 <info@grodansparadis.com>

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

*/

#ifndef vscptcpclient_h
#define vscptcpclient_h


#include <Arduino.h>
#include "IPAddress.h"
#include "Client.h"
#include "vscp.h"

// Wait time if no events in queue
#define VSCP_CLIENT_TCP_LOOP_WAIT   1000

// Max size of response buffer. 
// Override with setResponseBufferSize()
#ifndef VSCP_MAX_RESPONSE_BUFFER
#define VSCP_MAX_RESPONSE_BUFFER 800
#endif

// Max time in milliseconds to wait for "+OK" response. 
// Override with setResponseTimeout()
#ifndef VSCP_RESPONSE_TIMEOUT
#define VSCP_RESPONSE_TIMEOUT    2000   // Two seconds
#endif

// Possible values for client.state()
#define VSCP_STATE_CONNECTED               0
#define VSCP_STATE_DISCONNECTED            1
#define VSCP_STATE_CONNECTION_LOST         2

#if defined(ESP8266) || defined(ESP32)
#include <functional>
#define VSCP_CALLBACK_PROTOTYPE std::function<void(PVSCPEVENTEX pex)> callback
#else
#define VSCP_CALLBACK_PROTOTYPE void (*callback)(PVSCPEVENTEX pex)
#endif

class vscpTcpClient /*: public Print*/ {
  
  public:

    // C-tor
    vscpTcpClient();
    vscpTcpClient(Client& client);
    vscpTcpClient(IPAddress ip, uint16_t port, Client& client);
    vscpTcpClient(IPAddress addr, uint16_t port, VSCP_CALLBACK_PROTOTYPE, Client& client);
    vscpTcpClient(uint8_t *ip, uint16_t port, Client& client);
    vscpTcpClient(uint8_t *ip, uint16_t port, VSCP_CALLBACK_PROTOTYPE, Client& client);
    vscpTcpClient(const char*, uint16_t, Client& client);
    vscpTcpClient(const char*, uint16_t, VSCP_CALLBACK_PROTOTYPE,Client& client);
    
    // D-tor
    ~vscpTcpClient();

    void setServer(IPAddress ip, uint16_t port);
    void setServer(uint8_t * ip, uint16_t port);
    void setServer(const char * domain, uint16_t port);
    
    void setCallback(VSCP_CALLBACK_PROTOTYPE);
    void setClient(Client& client);
    
    boolean setResponseBufferSize(uint16_t size);
    void setResponseTimeout(uint16_t timeout);

    /*!
      Returns the state of the client
      @return The connection state.
    */
    int state();

    /*!
      Read value from string

      @param strval Value to read. Can be decimal, octal, 
                    hexadecimal or binary
    */
    int32_t readStringValue(char *strval);

    /*!
      Read GUID on string and put as bytes in buffer
      
      @param buf 16 byte GUID buffer.
      @param strguid GUID on string form.
      @return Return VSCP_ERROR_SUCCESS on success or errocode if 
              failure.
    */
    int readGuidFromStr(uint8_t *buf, char *strguid);

    /*!
      Write GUID to buffer from string
      
      @param strguid GUID on string form.
      @param buf 16 byte GUID buffer.      
      @return Return VSCP_ERROR_SUCCESS on success or errocode if 
              failure.
    */
    int writeGuidToStr(char *strguid, const uint8_t *buf);

    /*!
      Get datetime from VSCP host 
      @param strdt VSCP datettime on ISOO format (2020-08-25T11:48:02Z)
      @param ex VSCP event.      
      @return Return VSCP_ERROR_SUCCESS on success or errocode if 
              failure.
    */
    int readDateTimeFromStr(vscpEventEx &ex,char *strdt);

    /*!
      Write VSCP datetime on string form to event.
      @param ex VSCP event.
      @param str String to write to
      @return Return VSCP_ERROR_SUCCESS on success or errocode if 
              failure.
    */
    int writeDateTimeToStr(char *str,vscpEventEx &ex);

    /*!
      Check if connected to remote host

      @return true if connected, false otherwise.
    */
    boolean isConnected(void); 

    /*!
      Connect to a remote VSCP host

      @param user Pointer to a string containing the username
      @param password Pointer to a string containing the password.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int connect(const char* user, const char* password);

    /*!
      Disconnect form remote VSCP host

      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int disconnect(void);

    /*!
        Check response from remote host. 

        @return VSCP_ERROR_SUCCESS is returned on success. The
                full respone is save and can be fetched with 
                getLastRespone()
    */
    int checkResponse(void);

    // boolean print(const char *pstr);
    
    // boolean println(const char *pstr);

    /*!
        Returns the last response from the remote host.

        @return Return last response as a pointer to a char string
    */
    uint8_t *getLastResponse(void) { return m_pbuf; };

    /*!
      Check if there are events available on the remote host

      @param cnt Reference to uint16_t that will get number of
                  events that are ready to be fetched form remote 
                  host.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int checkRemoteBufferCount(uint16_t *pcnt);
    
    /*!
      Fetch one event from remote host

      @param ex Reference to VSCP event that will get the remote event. 
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.

    */
    int fetchRemoteEvent(vscpEventEx &ex);

    /*!
      Fetch one event from remote host

      @param ex Reference to VSCP event that will get the remote event. 
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    //int fetchRemoteEvent(vscpEvent &ev);

    /*!
      Send event to remote door

      @param ex Reference to event that should be sent.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure. 
    */
    int sendEventToRemote(vscpEventEx &ex);

    /*!
      Send event to remote door

      @param ex Reference to event that should be sent.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    //int sendEventToRemote(vscpEvent &ev);

    /*!
      Do a NOOP (no operation)

      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int doNoop(void);

    /*!
      Set filter (filter+mask) on remote host

      @param filter Reference to a VSCP filter.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int setRemoteFilter(vscpEventFilter &filter);

    /*!
      Get the channel id from the remote host

      @param chid Pointer to an uint16_t that will get the channel id.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int getRemoteChannelId(uint32_t *chid);

    /*!
      Get the channel GUID from the remote host

      @param pGUID Pointer to a buffer that will get the GUID. 
                    The buffer must be 16-bytes (or more) in size. 
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int getRemoteGUID(uint8_t *pGUID);

    /*!
      Set the channel GUID on the remote host

      @param pGUID Pointer to a buffer that holds the new GUID
                    that should be set. 
                    The buffer must be 16-bytes (or more) in size. 
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.
    */
    int setRemoteGUID(const uint8_t *pGUID);

    /*!
      Get list of interfaces from the remote host remote host interface
      @param pcnt Pointer to a value that will get number of interfaces
                  on the remote server. Setting pinterfaces = NULL and
                  size = 0 just count the number of interfaces.
      @param pinterfaces Pointer to an array of strings to interface
                          descriptions. Each of the interface descriptions 
                          is in the receive buffer so any remote call 
                          or operation will destroy the information. It must
                          therefore be copied out if it is needed for later
                          use.
      @param size This is the maximum number of string pointers the caller
                  can accept in pinterfaces. Set to zero to get the available 
                  number of interfaces in the remote machine.
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.

      !!!! IMPORTANT !!!!
      The default buffer size is normally to small to support this
      method. Increase with setBufferSize before use.
    */
   int getRemoteInterfaces(uint8_t *pcnt, 
                            const char **pinterfaces, 
                            uint8_t size);

    /*!
      Get version for remote host interface

      @param pVersion Points to buffer that will receive
                      version information on the form 
                      major,minor,sub,build6
      @return Return VSCP_ERROR_SUCCESS on success or errocode in 
              failure.                      
    */
    int getRemoteVersion(char *pVersion);

    //void deleteEvent(PVSCPEVENT *pev);

    /*! 

    */
    boolean loop(void);

  private:

    Client* m_client;     // Communication client
    IPAddress m_ip;       // IP addres for VSCP remote host
    const char* m_domain; // Host domain for VSCP remote host
    uint16_t m_port;      // Port where VSCP host is located

    VSCP_CALLBACK_PROTOTYPE;

    uint16_t m_socketTimeout;
    uint16_t m_responseTimeout;

    uint32_t m_lastLoop;  // time stamp for last loop without event to fetch
                          // Set to zero when data is fetched

    // Holds the last response from the server
    // when a call has returned with a positive response. 
    uint8_t *m_pbuf;
    uint16_t m_size_buffer;

    int m_state;          // Communication state machine state
};

#endif
