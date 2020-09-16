# writeDateTimeToStr

Write the date/time information from a VSCP event into a string on ISO form (2020-08-27T14:59:05Z).

## int writeDateTimeToStr(char *str,vscpEventEx &ex)

### Parameters
* **str** String buffer that will receive the date/time. Must be large enough to hold the information.
* **ex** - Reference to a VSCP event ([VscpEventEx](https://github.com/grodansparadis/vscp/blob/master/src/vscp/common/vscp.h) ![External Link](./images/Icon_External_Link.png)) from which the date/time information is fetched.

### Returns
VSCP_ERROR_SUCCESS is returned if all os OK.

[filename](./bottom_copyright.md ':include')

