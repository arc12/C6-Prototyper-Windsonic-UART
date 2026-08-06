# Windsonic Sensor Guide
_Refer to the Windsonic User Manual from Gill Instruments; the equipment in Poole's Cavern is the "option 3" model._

This logger uses the RS232 UART interface, whereas the original Logbox AA used analogue outputs. It also uses the "polled" approach, where measurements are requested rather than transmitted at intervals.

In order to compute means over a series of readings, the Windsonic is also re-configured to return cartesian ("UV") speeds. A vector sum is computed and divided down to give mean speed and wind direction (where it is coming from).

There are no configuration settings which can be changed using the logger web interface.

## Connector Pin Assignment
Looking at rear of connector, pin 1 is top centre then proceeds clockwise to top left (8) and 9 in centre:  
1 = Blue = GND (for analogue and RS232)  
2 = Red = +ve supply  
3 = Black = -ve supply. This black is twisted pair with red.  
5 = Yellow = TX (from Windsonic)  
7 = White = RX  
8 = Green = analogue ch 1  
9 = Black = analogue ch 2

This works for Logbox AA and for RS232.

## Configuration
__Setup Prior to Change__  
_ie as used for the logbox AA_  
Device info (D1 and D2 commands): Y16460019, 2368-110-01  
Dump of Windsonic configuration (D3 command): M2,U1,O1,L1,P1,B3,H1,NQ,F1,E3,T3,S1,C2,G0,K50,

__Change Commands__  
M3 = UV polled. Send '?' to enable polling then 'Q' to take a reading (since Q is the node address) or just send '?Q' each time. Command mode is now '*Q'

S9 = disable analogue

O2 = fixed field output (vs O1 = CSV). Looks the same!

In polled mode, it appears you do not get the startup and self-test messages...*

__Typical Response to ?Q__  
[02]Q,+000.01,-000.01,M,00,[03]36  