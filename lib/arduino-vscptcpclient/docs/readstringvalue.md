# readStringValue

Helper function that read a decimal, octal, hex or binary value from a string and return the value as a 32-bit signed value. 

* If the string value is just numbers it is interpreted as a decimal value.
* If the string value is preceded with "0o" it is interpreted as an octal number.
* If the string value is preceded with "0x" it is interpreted as an hexadecimal number.
* If the string value is preceded with "0b" it is interpreted as an binary number.

## int32_t readStringValue(char *strval)

[filename](./bottom_copyright.md ':include')
