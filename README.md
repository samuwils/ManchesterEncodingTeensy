# ManchesterEncodingTeensy
Manchester Encoding Communication library

Teensy 3.2

This library sends out Manchester encoded char array on pin 2, and can read back the message on pin 3.
So to run the example just hook up pin 2 to pin 3 on your Teensy 3.2(haven't tested on other boards).

This library reads a 1 on a low to high pulse and 0 on high to low.

Works at 1MHz, but fails at 1.5MHz.

Library works by reading pulses using a FTM Timer and DMA. The sender class sends 8 ones at beginning and 8 zeros at end of message. 
Assuming the final pulse received is a zero we can go backwards through the pulses received and decode the message until we hit 7 ones
row. 

Unsolved issue is trying to receive during a transmission. Since DMA marker is moving and it looks like a message is there but there is
a possibility the the end char of 0's hasn't been sent and trying to read the pulses back would return junk.
