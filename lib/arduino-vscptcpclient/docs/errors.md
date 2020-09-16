# CANAL error codes

The following error codes are defined by CANAL

```c
/* Error Codes */
#define CANAL_ERROR_SUCCESS                 0           /* All is OK */
#define CANAL_ERROR_BAUDRATE                1           /* Baud rate error */
#define CANAL_ERROR_BUS_OFF                 2           /* Bus off error */
#define CANAL_ERROR_BUS_PASSIVE             3           /* Bus Passive error */
#define CANAL_ERROR_BUS_WARNING             4           /* Bus warning error */
#define CANAL_ERROR_CAN_ID                  5           /* Invalid CAN ID */
#define CANAL_ERROR_CAN_MESSAGE             6           /* Invalid CAN message */
#define CANAL_ERROR_CHANNEL                 7           /* Invalid channel */
#define CANAL_ERROR_FIFO_EMPTY              8           /* FIFO is empty */
#define CANAL_ERROR_FIFO_FULL               9           /* FIFI is full */
#define CANAL_ERROR_FIFO_SIZE               10          /* FIFO size error */
#define CANAL_ERROR_FIFO_WAIT               11
#define CANAL_ERROR_GENERIC                 12          /* Generic error */
#define CANAL_ERROR_HARDWARE                13          /* Hardware error */
#define CANAL_ERROR_INIT_FAIL               14          /* Initialization failed */
#define CANAL_ERROR_INIT_MISSING            15
#define CANAL_ERROR_INIT_READY              16
#define CANAL_ERROR_NOT_SUPPORTED           17          /* Not supported */
#define CANAL_ERROR_OVERRUN                 18          /* Overrun */
#define CANAL_ERROR_RCV_EMPTY               19          /* Receive buffer empty */
#define CANAL_ERROR_REGISTER                20          /* Register value error */
#define CANAL_ERROR_TRM_FULL                21
#define CANAL_ERROR_ERRFRM_STUFF            22          /* Error frame: stuff error detected */
#define CANAL_ERROR_ERRFRM_FORM             23          /* Error frame: form error detected */
#define CANAL_ERROR_ERRFRM_ACK              24          /* Error frame: acknowledge error */
#define CANAL_ERROR_ERRFRM_BIT1             25          /* Error frame: bit 1 error */
#define CANAL_ERROR_ERRFRM_BIT0             26          /* Error frame: bit 0 error */
#define CANAL_ERROR_ERRFRM_CRC              27          /* Error frame: CRC error */
#define CANAL_ERROR_LIBRARY                 28          /* Unable to load library */
#define CANAL_ERROR_PROCADDRESS             29          /* Unable get library proc. address */
#define CANAL_ERROR_ONLY_ONE_INSTANCE       30          /* Only one instance allowed */
#define CANAL_ERROR_SUB_DRIVER              31          /* Problem with sub driver call */
#define CANAL_ERROR_TIMEOUT                 32          /* Blocking call time-out */
#define CANAL_ERROR_NOT_OPEN                33          /* The device is not open. */
#define CANAL_ERROR_PARAMETER               34          /* A parameter is invalid. */
#define CANAL_ERROR_MEMORY                  35          /* Memory exhausted. */
#define CANAL_ERROR_INTERNAL                36          /* Some kind of internal program error */
#define CANAL_ERROR_COMMUNICATION           37          /* Some kind of communication error */
#define CANAL_ERROR_USER                    38          /* Login error */
```


[filename](./bottom_copyright.md ':include')