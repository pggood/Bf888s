Adventures in using a BF888 removing its brain and connecting an arduino
I started out using a HAMShield however it uses a RDA1846 and there doesnt appear to be a way of getting to the discrminator
The BF888 uses an ABOV processor and a bk4811 that has a frequency range of 127 to 525Mhz <br>
20 VDD<br>
19 P00/AN0/DSDA tp3<br>
18 P01/AN1/DSCL tp8<br>
17 P02/AN2/AVREF/EINT0 CLOCK SPI to BK4811<br>
16 P03/AN3/EINT1 Enable SPI to BK4811<br>
15 P11/AN8/EINT6/EC1/BUZO Data SPI to BK4811<br>
14 P12/AN9/EINT11/T1O/PWM1O Inerupt from BK4811<br>
13 P13/AN10/EINT12/T2O/PWM2O led torch<br>
12 P14/AN11/MISO to transistor on recieve<br>
11 P15/AN12/MOSI RX LED<br>
10 P16/AN13/SCK MONI  (Monitor)  Button<br>
09 P24/SDA pin 10 U10(unpopulated)<br>
08 P25/SCL U10(unpopulated)<br>
7 P30/TXD/(SDA) RX data<br>
6 P31/RXD/(SCL) RX data and TX Switch<br>
5 P32/RESETB 16 Channel Switch E2-1<br>
4 P35/EINT10/T0O/PWM0O hannel Switch E2-3 via 2k resistor<br>
3 P36/XIN 16 Channel Switch E1-1<br>
2 P37/XOUT 16 Channel Switch E1-3<br>
1 VSS<br>
<br>
16 Channel Switch E1-2 and E2-2 to GND<br>
1	0	1	0<br>
0	0	1	0<br>
0	0	0	0<br>
0	1	0	0<br>
1	1	1	1<br>
1	0	1	1<br>
0	0	0	1<br>
0	1	1	1<br>
1	1	0	0<br>
1	0	0	0<br>

The BK4811B is a half duplex TDD FM transceiver operating from 127 MHz to 525 MHz band for worldwide personal radio service.
Key Features<br>
 World wide band: 127 ~ 525 MHz<br>
 12.5/25 kHz channel spacing<br>
 audio filter with four optional bandwidths<br>
 On chip 4 dBm RF PA<br>
 2.4 V to 3.6 V power supply<br>
 CTCSS tone receiver with up to parallel eight frequency detector<br>
 23/24 bit programmable DCS code<br>
 Standard DTMF and programmable in-band dual tone<br>
 SELCALL and programmable in-band single tone<br>
 1.2/2.4 kbps FSK data modem with either F2D or F1W modulation type<br>
 Frequency inversion scrambler<br>
 Voice activated switch (VOX) and time-out timer<br>
 RF Signal strength measurement and signal quality measurement<br>
 TX Audio signal strength indication and RX audio signal strength indication<br>
 3-wires interface with MCU with maximum 8 Mbps clock rate<br>
 QFN 4x4 24-Pin package<br>
