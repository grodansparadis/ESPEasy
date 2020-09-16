# state

Returns the current state of the client. If a connection attempt fails, this can be used to get more information about the failure.

## int state(void)

### params
None

### returns

The possible states are defined as

 * VSCP_STATE_CONNECTED               0
 * VSCP_STATE_DISCONNECTED            1
 * VSCP_STATE_CONNECTION_LOST         2

[filename](./bottom_copyright.md ':include') 