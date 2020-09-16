# getRemoteInterfaces

Get interfaces available on the remote VSCP host. 

**Warning** If the remote host have many interfaces the assigned buffer will not be able to hold them and must be increase with [setResponseBufferSize](./setresponsebuffersize.md)

## int getRemoteInterfaces(uint8_t *pcnt, const char **pinterfaces, uint8_t size)

### Parameters
* ** pcnt** - A pointer to a byte that get the number of interfaces on the remote host.
* **pinterfaces** - This is an array of pointers which is filled with interface descriptions. The interface descriprions are held in the read buffer so no i&o operation scan be performed before needed data is copied out and saved. Can be set to NULL to just receive the number of interfaces on the remote host.
* **size** - This is the number of interface pointers that are available in the pinterfaces array.

### Returns
Return VSCP_ERROR_SUCCESS on success or an erro code in failure.

[filename](./bottom_copyright.md ':include')
