#include "MancEnc.h"


MancEncOutput meo;
MancEncInput mei;
int count=0;

const byte numChars = 40;
char receivedChars[numChars];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  meo.begin(2,800000);
  mei.begin(3);
  delay(1000);
  Serial.println("end setup");
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available() > 0)
  {
    static byte index = 0;
    char newL = '\n';
    char rc;
    
    while(Serial.available() > 0)// && newData == false)
    {
      rc = Serial.read();
  
      if(rc != newL)
      {
        receivedChars[index] = rc;
        index++;
        if(index >= numChars)
        {
          index = numChars -1;
        }        
      }
      else
      {
        receivedChars[index] = '\0';
        index = 0;
      }  
    }
    Serial.printf("Transmitting %s\n",receivedChars);
    meo.send(receivedChars);
  }

  delay(10);
  if(mei.available())
  {
    mei.read();
    Serial.printf("Manchester Encoding Read %s\n",mei.DATA);
  }

delay(500);
  
}
