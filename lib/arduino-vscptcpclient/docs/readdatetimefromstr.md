# readDateTimeFromStr

Read a VSCP event date/time from a string and write it to the date time information in a VSCP event.

## int readDateTimeFromStr(vscpEventEx &ex,char *strdt)

### Parameters
 * **ex** - Reference to a VSCP event ([VscpEventEx](https://github.com/grodansparadis/vscp/blob/master/src/vscp/common/vscp.h) ![External Link](./images/Icon_External_Link.png)) that will receive the date/time data.
 * **strdt** - A string containing date/time info on ISO form (2020-08-27T14:54:05Z).

### Returns
VSCP_ERROR_SUCCESS is returned if all os OK.

[filename](./bottom_copyright.md ':include')
