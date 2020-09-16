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

#include "vscp.h"
#include "vscptcpclient.h"

vscpTcpClient::vscpTcpClient() 
{
    m_state = VSCP_STATE_DISCONNECTED;
    m_client = NULL;
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
    setCallback(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(Client& client) 
{
    m_state = VSCP_STATE_DISCONNECTED;
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(IPAddress addr, 
                              uint16_t port, 
                              Client& client) 
{
    m_state = VSCP_STATE_DISCONNECTED;
    setServer(addr, port);
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(IPAddress addr, 
                              uint16_t port, 
                              VSCP_CALLBACK_PROTOTYPE, 
                              Client& client) 
{
    m_state = VSCP_STATE_DISCONNECTED;
    setServer(addr, port);
    setCallback(callback);
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}


///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(uint8_t *ip, 
                              uint16_t port, 
                              Client& client) 
{
    m_state = VSCP_STATE_DISCONNECTED;
    setServer(ip, port);
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}


///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(uint8_t *ip, 
                              uint16_t port, 
                              VSCP_CALLBACK_PROTOTYPE, 
                              Client& client) 
{
    m_state = VSCP_STATE_DISCONNECTED;
    setServer(ip, port);
    setCallback(callback);
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(const char* domain, 
                              uint16_t port, 
                              Client& client) 
{
    m_state = VSCP_STATE_DISCONNECTED;
    setServer(domain,port);
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}


///////////////////////////////////////////////////////////////////////////////
// vscpTcpClient
//

vscpTcpClient::vscpTcpClient(const char* domain, 
                        uint16_t port, 
                        VSCP_CALLBACK_PROTOTYPE, 
                        Client& client)
{
    m_state = VSCP_STATE_DISCONNECTED;
    setServer(domain,port);
    setCallback(callback);
    setClient(client);
    m_pbuf = NULL;
    m_lastLoop = 0;
    setResponseBufferSize(VSCP_MAX_RESPONSE_BUFFER);
    setResponseTimeout(VSCP_RESPONSE_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////
// ~vscpTcpClient
//

vscpTcpClient::~vscpTcpClient() 
{
  ;
}

void vscpTcpClient::setServer(uint8_t *ip, uint16_t port) 
{
    IPAddress addr(ip[0],ip[1],ip[2],ip[3]);
    setServer(addr,port);
}

void vscpTcpClient::setServer(IPAddress ip, uint16_t port) 
{
  m_ip = ip;
  m_port = port;
  m_domain = NULL;
}

void vscpTcpClient::setServer(const char * domain, uint16_t port) 
{
  m_domain = domain;
  m_port = port;
}

///////////////////////////////////////////////////////////////////////////////
// setCallback
//

void vscpTcpClient::setCallback(VSCP_CALLBACK_PROTOTYPE) 
{
  this->callback = callback;
}

///////////////////////////////////////////////////////////////////////////////
// setClient
//

void vscpTcpClient::setClient(Client& client)
{
  m_client = &client;
}

///////////////////////////////////////////////////////////////////////////////
// state
//

int vscpTcpClient::state() 
{
  return m_state;
}

///////////////////////////////////////////////////////////////////////////////
// setResponseBufferSize
//

boolean vscpTcpClient::setResponseBufferSize(uint16_t size)
{
  if ( 0 == size) {
    return false;
  }

  if ( NULL == m_pbuf ) {
    m_pbuf = (uint8_t*)malloc(size);    
  }
  else {
    uint8_t* newBuffer = (uint8_t*)realloc(m_pbuf, size);
    if (newBuffer != NULL) {
      m_pbuf = newBuffer;
    } 
    else {
      return false;
    }
  }

  m_size_buffer = size;

  // remove old allocation
  // if ( NULL != m_pbuf ) {
  //   delete [] m_pbuf;
  //   m_pbuf = NULL;
  // }

  // m_pbuf = new char[size];
  // m_size_buffer = size;

  return (NULL != m_pbuf);
}


///////////////////////////////////////////////////////////////////////////////
// setResponseTimeout
//

void vscpTcpClient::setResponseTimeout(uint16_t timeout)
{
  m_responseTimeout = timeout;  
}

///////////////////////////////////////////////////////////////////////////////
// print
//

// boolean vscpTcpClient::print(const char *pstr)
// {
//   size_t len = strlen(pstr);

//   // Check pointer and string length
//   if ( (NULL == pstr) || !len ) return false;

//   uint8_t *p = new uint8_t[len];
//   if ( NULL == p ) return false;

//   memcpy(p,pstr,len);
//   size_t n = m_client->write(p, len);
//   delete [] p;

//   return (n==len);
// }

///////////////////////////////////////////////////////////////////////////////
// println
//

// boolean vscpTcpClient::println(const char *pstr)
// {
//   uint8_t buf[] = {'\r','\n'};

//   if ( print(pstr) ) {
//     return (2 == m_client->write(buf,2));
//   }

//   return false;
// }

///////////////////////////////////////////////////////////////////////////////
// isConnected
//

boolean vscpTcpClient::isConnected() 
{
  uint8_t rv = 0;

  if ( m_client != NULL ) {
    
    if (!(rv = m_client->connected())) {
        if (m_state == VSCP_STATE_CONNECTED) {
          m_state = VSCP_STATE_CONNECTION_LOST;
          m_client->flush();
          m_client->stop();
        }
    } 

  }

  return (rv?true:false);
}

///////////////////////////////////////////////////////////////////////////////
// connect
//

int vscpTcpClient::connect(const char* user, const char* pass)
{
  if ( !isConnected() ) {

    int result = 0;

    // Check if already connected?
    if ( m_client->connected() ) {
      result = 1;
    } 
    else {
      // Nope connect
      if (m_domain != NULL) {
        result = m_client->connect(m_domain, m_port);
      } 
      else {
        result = m_client->connect(m_ip, m_port);
      }
    }

    if ( 1 == result ) {

      // Verify remote host is VSCP host
      if (VSCP_ERROR_SUCCESS != checkResponse()) {
        m_client->stop();
        m_state = VSCP_STATE_DISCONNECTED;
        return VSCP_ERROR_CONNECTION;
      }

      // User
      m_client->println("user admin");
      m_client->flush();
      if (VSCP_ERROR_SUCCESS != checkResponse()) {
        m_client->stop();
        m_state = VSCP_STATE_DISCONNECTED;
        return VSCP_ERROR_USER;
      }

      // Password
      m_client->println("pass secret");
      m_client->flush();
      if (VSCP_ERROR_SUCCESS != checkResponse()) {
        m_client->stop();
        m_state = VSCP_STATE_DISCONNECTED;
        return VSCP_ERROR_PASSWORD;
      }

    }  
    else {
      m_client->stop();
      m_state = VSCP_STATE_DISCONNECTED;
      return VSCP_ERROR_CONNECTION;
    }

  }

  return VSCP_ERROR_SUCCESS;  
}

///////////////////////////////////////////////////////////////////////////////
// disconnect
//

int vscpTcpClient::disconnect() 
{
    m_client->println("quit");
    m_client->flush();
    m_state = VSCP_STATE_DISCONNECTED;
    m_client->flush();
    m_client->stop();
    return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// checkResponse
//

int vscpTcpClient::checkResponse() 
{
  int rv = VSCP_ERROR_TIMEOUT;
  uint16_t pos = 0;
  uint32_t start = millis();

  // Clear response buffer
  memset(m_pbuf, 0, m_size_buffer);

  while ( (millis()-start) < m_responseTimeout ) {
    
    if ( m_client->available() ) {

      m_pbuf[pos] = m_client->read();
      pos++;

      // Check for buffer overflow
      if ( pos >= m_size_buffer) { 
        return VSCP_ERROR_BUFFER_TO_SMALL;
      }
      
      // Positive response
      if ( (VSCP_ERROR_SUCCESS != rv) && 
          (NULL != strstr((char *)m_pbuf,"+OK"))) {     
        
        rv = VSCP_ERROR_SUCCESS;

        // Read up to end of line to empty input
        // buffer
        while ( m_client->available() ) {
          if ( '\n' == m_client->read() )  break; 
        }

        break;
      }

    }
    
    yield();
    
  }

  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// checkRemoteBufferCount
//

int vscpTcpClient::checkRemoteBufferCount(uint16_t *pcnt)
{
  m_client->println("chkdata");
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  char *p;
  if ( NULL == (p = strstr((char *)m_pbuf,"\r\n") ) ) {
    return VSCP_ERROR_ERROR;
  }

  *p = 0; // Isolate count
  *pcnt = atoi((char *)m_pbuf);

  return VSCP_ERROR_SUCCESS;
}


///////////////////////////////////////////////////////////////////////////////
// fetchRemoteEvent
//

int vscpTcpClient::fetchRemoteEvent(vscpEventEx &ex)
{
  char *p,*pFound;

  m_client->println("retr 1");
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  if ( NULL == (p = strstr((char *)m_pbuf,"\r\n") ) ) {
    return VSCP_ERROR_ERROR;
  }

  *p = 0;     // Isolate event
  p = (char *)m_pbuf;

  // * * * * Get header * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0;  
  ex.head = readStringValue(p);
  p = pFound + 1;    // Point beyond comma


  // * * * * Get class * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0; 
  ex.vscp_class = readStringValue(p);
  p = pFound + 1;    // Point beyond comma


  // * * * * Get type * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0; 
  ex.vscp_type = readStringValue(p);
  p = pFound + 1;    // Point beyond comma


  // * * * * Get obid * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0;  
  ex.obid = readStringValue(p);
  p = pFound + 1;    // Point beyond comma


  // * * * * Get datetime * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0;   
  readDateTimeFromStr(ex,p);
  p = pFound + 1;    // Point beyond comma


  // * * * * Get timestamp * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0;  
  ex.timestamp = (uint32_t)atol(p);
  p = pFound + 1;    // Point beyond comma


  // * * * * Get GUID * * * *
  if ( NULL == (pFound = strchr(p,',') ) ) {
    return VSCP_ERROR_ERROR;    
  }

  *pFound = 0;  
  if ( VSCP_ERROR_SUCCESS != readGuidFromStr(ex.GUID,p) ) {
    return VSCP_ERROR_PARAMETER;
  }
  p = pFound + 1;    // Point beyond comma


  // * * * * Get Data * * * *
  uint16_t count = 0;
  while (true) {
    
    if ( NULL == (pFound = strchr(p,',') ) ) {
      break;    
    }

    // Must be room
    if ( count >= VSCP_MAX_DATA) return VSCP_ERROR_BUFFER_TO_SMALL;

    *pFound = 0;

    ex.data[count] = (uint8_t)readStringValue(p);
    count++;
    p = pFound + 1; // Point beyond comma
  }

  ex.sizeData = count;

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// sendEventToRemote
//

int vscpTcpClient::sendEventToRemote(vscpEventEx &ex)
{
  char buf[80];

  m_client->print("send ");

  // Head
  memset(buf,0,sizeof(buf));  
  m_client->print(itoa(ex.head,buf,10));
  m_client->print(",");

  // Class
  memset(buf,0,sizeof(buf));;  
  m_client->print(itoa(ex.vscp_class,buf,10));
  m_client->print(",");

  // Type
  memset(buf,0,sizeof(buf));  
  m_client->print(itoa(ex.vscp_type,buf,10));
  m_client->print(",");

  // obid
  memset(buf,0,sizeof(buf));  
  m_client->print(ltoa(ex.obid,buf,10));
  m_client->print(",");

  // Datetime
  memset(buf,0,sizeof(buf));
  writeDateTimeToStr(buf,ex);
  m_client->print(buf);
  m_client->print(",");

  // timestamp
  memset(buf,0,sizeof(buf));  
  m_client->print(ltoa(ex.timestamp,buf,10));
  m_client->print(",");

  // GUID
  memset(buf,0,sizeof(buf));
  writeGuidToStr(buf,ex.GUID);
  m_client->print(buf);
  m_client->print(",");

  // Data
  for (int i=0;i<ex.sizeData;i++) {
    m_client->print(itoa(ex.data[i],buf,10));
    if (i<(ex.sizeData-1)) m_client->print(",");
  }

  m_client->println();
  m_client->flush();

  return checkResponse();
}

////////////////////////////////////////////////////////////////////////////////
// doNoop
//

int vscpTcpClient::doNoop(void)
{
  m_client->println("noop");
  m_client->flush();

  return checkResponse();
}

////////////////////////////////////////////////////////////////////////////////
// setRemoteFilter
//

int vscpTcpClient::setRemoteFilter(vscpEventFilter &filter)
{
  char buf[50];

  // * * * * Set Filter * * * * 

  // priority
  memset(buf,0,sizeof(buf));  
  sprintf(buf,
          "setfilter %d,%d,%d,",
          filter.filter_priority,
          filter.filter_class,
          filter.filter_type);
  m_client->print(buf);        

  // GUID
  memset(buf,0,sizeof(buf));
  writeGuidToStr(buf,filter.filter_GUID);
  m_client->println(buf);

  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  // * * * * Set Mask * * * * 

  // priority
  sprintf(buf,
          "setmask  %d,%d,%d,",
          filter.mask_priority,
          filter.mask_class,
          filter.mask_type);
  m_client->print(buf);

  // GUID
  memset(buf,0,sizeof(buf));
  writeGuidToStr(buf,filter.mask_GUID);
  m_client->println(buf);

  m_client->flush();

  return checkResponse();
}

////////////////////////////////////////////////////////////////////////////////
// getRemoteChannelId
//

int vscpTcpClient::getRemoteChannelId(uint32_t *chid)
{
  m_client->println("chid");
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  char *p;
  if ( NULL == (p = strstr((char *)m_pbuf,"\r\n") ) ) {
    return VSCP_ERROR_ERROR;
  }

  *p = 0; // Isolate count
  *chid = (uint32_t)atol((const char *)m_pbuf);

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// getRemoteGUID
//

int vscpTcpClient::getRemoteGUID(uint8_t *pGUID)
{
  m_client->println("getguid");
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  char *p;
  if ( NULL == (p = strstr((char *)m_pbuf,"\r\n") ) ) {
    return VSCP_ERROR_ERROR;
  }

  *p = 0; // Isolate count
  return readGuidFromStr(pGUID,(char *)m_pbuf);
}

////////////////////////////////////////////////////////////////////////////////
// setRemoteGUID
//

int vscpTcpClient::setRemoteGUID(const uint8_t *pGUID)
{
  char buf[50];
  
  m_client->print("setguid ");
  writeGuidToStr(buf,pGUID);
  m_client->print(buf);
  m_client->println();
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  return VSCP_ERROR_SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
// getRemoteVersion
//

int vscpTcpClient::getRemoteVersion(char *pVersion)
{
  // Check pointer 
  if ( NULL == pVersion ) return VSCP_ERROR_INVALID_POINTER;

  m_client->println("version");
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  char *p;
  if ( NULL == (p = strstr((char *)m_pbuf,"\r\n") ) ) {
    return VSCP_ERROR_ERROR;
  }

  *p = 0; 
  strcpy(pVersion,(const char *)m_pbuf);

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// getRemoteInterfaces
//

int vscpTcpClient::getRemoteInterfaces(uint8_t *pcnt, 
                                      const char **pinterfaces, 
                                      uint8_t size)
{
  // Check pointer 
  if ( NULL == pcnt ) return VSCP_ERROR_INVALID_POINTER;
  // pinterfaces allowed to be NULL

  m_client->println("interface list");
  m_client->flush();

  if ( VSCP_ERROR_SUCCESS != checkResponse() ) {
    return VSCP_ERROR_ERROR;
  }

  // Response look something like this
  // 65534,1,FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:FF:FE:00:00,Internal Server Client.|Started at 2020-08-23T21:56:23Z
  // 1,5,FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:00:01:00:00,Remote TCP/IP Server. [9598]|Started at 2020-08-23T21:56:24Z
  // 2,5,FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:00:02:00:00,Remote TCP/IP Server. [9598]|Started at 2020-08-23T21:56:31Z
  // 3,5,FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:00:03:00:00,Remote TCP/IP Server. [9598]|Started at 2020-08-25T10:37:26Z
  // +OK - Success.

  char *p = (char *)m_pbuf;
  while (*p) {
    
    char *pEnd;
    if ( NULL == (pEnd = strstr(p,"\r\n") ) ) {
      return VSCP_ERROR_ERROR;
    }

    *pEnd = 0;
    if (size && (*pcnt < size)) {
      pinterfaces[*pcnt] = p;
    } 

    pcnt++;
    p = pEnd + 2; // Point past CRLF == next i/f

  }

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// readGuidFromStr
//

int vscpTcpClient::readGuidFromStr(uint8_t *buf, char *strguid)
{
  char *p = strguid;

  for (int i=0; i<16; i++) {
    buf[i] = (uint8_t)strtoul(p, &p, 16);
    p++;
  }

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// writeGuidToStr
//

int vscpTcpClient::writeGuidToStr(char *strguid, const uint8_t *buf)
{
  char *p = strguid;
  for (int i=0; i<16; i++) {
    sprintf(p,"%02x",buf[i]);
    p += 2;
    if (15 != i) *p=':';
    p++;
  }

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// readDateTimeFromStr
//

int vscpTcpClient::readDateTimeFromStr(vscpEventEx &ex,char *strdt)
{
  // 2020-08-25T11:48:02Z
  char *p = strdt;

  // Get year
  ex.year = (uint16_t)strtoul(p, &p, 10);
  p++;
  ex.month = (uint8_t)strtoul(p, &p, 10);
  if (ex.month>12) return VSCP_ERROR_ERROR;
  p++;
  ex.day = (uint8_t)strtoul(p, &p, 10);
  if (ex.day>31) return VSCP_ERROR_ERROR;
  p++;
  ex.hour = (uint8_t)strtoul(p, &p, 10);
  if (ex.hour>24) return VSCP_ERROR_ERROR;
  p++;
  ex.minute = (uint8_t)strtoul(p, &p, 10);
  if (ex.minute>60) return VSCP_ERROR_ERROR;
  p++;
  ex.second = (uint8_t)strtoul(p, &p, 10);
  if (ex.minute>60) return VSCP_ERROR_ERROR;

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// writeDateTimeToStr
//

int vscpTcpClient::writeDateTimeToStr(char *pstr,vscpEventEx &ex)
{
  // Check pointer
  if ( NULL == pstr ) return VSCP_ERROR_INVALID_POINTER;

  *pstr = 0;

  // Only write out date if not nilled
  // Set by server if not given
  if ( ex.year ||
        ex.month ||
        ex.day ||
        ex.hour ||
        ex.minute ||
        ex.second ) {
    if ( sprintf(pstr,
                "%04d-%02d-%02dT%02d:%02d:%02d",
                ex.year,
                ex.month,
                ex.day,
                ex.hour,
                ex.minute,
                ex.second) < 0 ) {
      return VSCP_ERROR_ERROR;                  
    }
  }

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// readStringValue
//

int32_t vscpTcpClient::readStringValue(char *strval)
{
    int32_t val = 0;
    char *p = strval;
    char *pFound;

    // Make lower case
    while ((*p = tolower(*p))) p++;
    p = strval;

    // Trim leading space
    while (isspace(*p)) p++;

    if (NULL != (pFound = strstr(p,"0x"))) {
      p += 2;
      val = strtoul(p, &p, 16);
    }
    else if (NULL != (pFound = strstr(p,"0o"))) {
      p += 2;
      val = strtoul(p, &p, 8);
    }
    else if (NULL != (pFound = strstr(p,"0b"))) {
      p += 2;
      val = strtoul(p, &p, 2);
    }
    else {
      val = atol(p);
    }

    return val;
}

////////////////////////////////////////////////////////////////////////////////
// loop
//

boolean vscpTcpClient::loop() 
{
  if ( isConnected() ) {
    
    uint32_t now = millis();    // m_lastLoop   
    if (callback) {
      if (0 == m_lastLoop) {
        vscpEventEx *pex = new vscpEventEx;
        if ( VSCP_ERROR_SUCCESS == 
              fetchRemoteEvent(*pex) ) {
          callback(pex);
        }
        else {
          delete pex;
          m_lastLoop = millis();
        }
      }
      else {
        if ((now-m_lastLoop) > VSCP_CLIENT_TCP_LOOP_WAIT) {
          m_lastLoop = 0; // Read data on next run
        }
      }
    }

    return true;
  }

  return false;
}

// -----------------------------