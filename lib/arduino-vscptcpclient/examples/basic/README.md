# Example code - basic example

This example repeatable connect to the network, log in to a remote VSCP host 
and send a 'woken-up', 'on', 'temperature' and 'battery voltage' event. 
It shows how to send measurement events in level I and Level II format.

For the temperature readings to work the example need a 1-wire temperature 
sensor connected to GPIO 5 (D1 on NodeMcu). Need a 4k7 pullup.

This code is part of the VSCP project (https://www.vscp.org)