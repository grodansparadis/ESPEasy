# event callback function

If the client want to get events from the host it can either poll for them using [fetchRemoteEvent](./fetchremoteevent.md) or set a callback in the constructor. This function is called when new events arrive from the remote host.

The callback function has the following prototype:


## void callback(VscpEventEx *pex)

### Parameters

 * **pex** - Pointer to a VSCP event ([VscpEventEx](https://github.com/grodansparadis/vscp/blob/master/src/vscp/common/vscp.h) ![External Link](./images/Icon_External_Link.png)) received from the remote VSCP host. When you are done handling this event you should delete it (ex _delete pex_).

 ### Returns
 void

[filename](./bottom_copyright.md ':include')