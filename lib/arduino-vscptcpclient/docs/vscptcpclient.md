# vscpTcpClient

Creates a vscpTcpClient object instance. There are several possible constructors you can choose from.

## vscpTcpClient(void)

Creates an uninitialized client instance.

Before it can be used, it must be configured with the property setters:

```c++
EthernetClient ethClient;
vscpTcpClient client;

void setup() {
    client.setClient(ethClient);
    client.setServer("vscp.example.com",9598);
    // client is now configured for use
}
```

### Parameters
none

### Returns
void

----

## vscpTcpClient(Client& client)

Creates a partially initialized client instance.

Before it can be used, the server details must be configured:

```c++
EthernetClient ethClient;
vscpTcpClient client(ethClient);

void setup() {
    client.setServer("vscp.example.com",9598);
    // client is now ready for use
}
```

### Parameters

 * **client** - the network client to use, for example _WiFiClient_

### Returns
void

---
    
## vscpTcpClient(IPAddress ip, uint16_t port, Client& client)

Creates a fully configured client instance.

```c++
IPAddress ip(192, 168, 0, 2);

EthernetClient ethClient;
vscpTcpClient client(ip,9598,callback,ethClient);

void setup() 
{
    // client is now ready for use
}
```

### Parameters

 * **ip** - Ip address for remote VSCP host. The [IpAddress object](https://www.arduino.cc/en/Reference/EthernetIPAddress) is used for the IPAddress ().
 * **port** -  the port to connect to 
 * **client** - the network client to use, for example _WiFiClient_

### Returns
void

----
    
## vscpTcpClient(IPAddress addr, uint16_t port, VSCP_CALLBACK_PROTOTYPE, Client& client)

Creates a fully configured client instance.

```c++
void callback(vscpEventEx *pex) 
{
    // Do something with the received event
}

IPAddress ip(192, 168, 0, 2);

EthernetClient ethClient;
vscpTcpClient client(ip,9598,callback,ethClient);

void setup() 
{
    // client is now ready for use
}

void loop {
    client.loop();
}
```

### Parameters

 * **ip** - Ip address for remote VSCP host. The [IpAddress object](https://www.arduino.cc/en/Reference/EthernetIPAddress) is used for the IPAddress ().
 * **port** -  the port to connect to
 * **callback** function* - a pointer to a message callback function called when an event arrives from the remote VSCP host.
 * **client** - the network client to use, for example _WiFiClient_

### Returns
void

---

## vscpTcpClient(uint8_t *ip, uint16_t port, Client& client)

Creates a fully configured client instance.

```c++
EthernetClient ethClient;
vscpTcpClient client("192.168.1.7",9598,callback,ethClient);

void setup() 
{
    // client is now ready for use
}
```

### Parameters

**client** - the network client to use, for example WiFiClient

### Returns
void

---
    
## vscpTcpClient(uint8_t *ip, uint16_t port, VSCP_CALLBACK_PROTOTYPE, Client& client)

```c++
void callback(vscpEventEx *pex) 
{
    // Do something with the received event
}

EthernetClient ethClient;
vscpTcpClient client("192.168.1.7",9598,callback,ethClient);

void setup() 
{
    // client is now ready for use
}

void loop {
    client.loop();
}
```

Creates a fully configured client instance.

### Parameters

**client** - the network client to use, for example WiFiClient

### Returns
void

---

## vscpTcpClient(const char* host, uint16_t, Client& client)

Creates a fully configured client instance.

```c++
EthernetClient ethClient;
vscpTcpClient client("host.vscp.org:9598",callback,ethClient);

void setup() 
{
    // client is now ready for use
}
```

### Parameters

**host** - Host address on the form _domain:port_.
**client** - the network client to use, for example WiFiClient

### Returns
void

---
    
## vscpTcpClient(const char*, uint16_t, VSCP_CALLBACK_PROTOTYPE,Client& client)

Creates a fully configured client instance.

```c++
void callback(vscpEventEx *pex) 
{
    // Do something with the received event
}

EthernetClient ethClient;
vscpTcpClient client("host.vscp.org:9598",callback,ethClient);

void setup() 
{
    // client is now ready for use
}

void loop {
    client.loop();
}
```

### Parameters

 * **host** - Host address on the form _domain:port_.
 * **callback** function* - a pointer to a message callback function called when an event arrives from the remote VSCP host.
 * **client** - the network client to use, for example WiFiClient

### Returns
void

[filename](./bottom_copyright.md ':include')