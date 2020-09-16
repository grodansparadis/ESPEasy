# SetServer

## void setServer(IPAddress ip, uint16_t port)

Sets the server details.

### Params
* **ip** - Ip address for remote VSCP host. The [IpAddress object](https://www.arduino.cc/en/Reference/EthernetIPAddress) is used for the IPAddress ().
 * **port** -  the port to connect to 

### Returns
void

----
    
## void setServer(uint8_t *ip, uint16_t port)

Sets the server details.

### Params
* **ip** - Ip address for remote VSCP host ( ex "192.168.1.7"). 
 * **port** -  the port to connect to 

### Returns
void

## void setServer(const char *domain, uint16_t port)

Sets the server details.

### Parameters
* **domain** - Address for remote VSCP host (ex "host.vscp.org").
 * **port** -  the port to connect to 

### Returns
void

[filename](./bottom_copyright.md ':include')
