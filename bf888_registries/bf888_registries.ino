void SetFrequency(float frequency,unsigned int IF,float crystal){
  int DIV;
  byte REG126_4;
  byte REG3_14_15;
  unsigned int REG127;
  if (frequency<127) return;
  if (frequency>525) return;
  if (frequency<525){
    DIV=8;
    REG126_4=0;
    REG3_14_15=0;
    REG127=43455;
  }   
  if (frequency<360){
    DIV=12;
    REG126_4=0;
    REG3_14_15=1;
    REG127=65182;
  }
  if (frequency<262){
    DIV=16;
    REG126_4=1;
    REG3_14_15=2;
    REG127=21373;
  }  
  if (frequency<210){
    DIV=20;
    REG126_4=1;
    REG3_14_15=3;
    REG127=43101;
  } 
  if (frequency<175){
    DIV=24;
    REG126_4=1;
    REG3_14_15=4;
    REG127=64828;
  }
  unsigned int REG16=19328/(121875/IF);
  unsigned long N_IF = 9.75*REG127;
  Serial.println(N_IF,DEC);
  unsigned long N =DIV*frequency*8388608/crystal-N_IF;
  Serial.println(N,DEC);
  unsigned int REG114= N&0xFFFF;
  unsigned int REG113=N>>16&0xFFFF;
 
  Serial.println(REG113,HEX);
  Serial.println(REG114,HEX);
  Serial.println(N_IF,HEX);
  Serial.println(DIV);
  Serial.println("_________________________________"); 
}

void setup() {
  Serial.begin(9600);
    SetFrequency(129,137000,21.7);
    SetFrequency(200,137000,21.7);
    SetFrequency(250,137000,21.7);
    SetFrequency(300,137000,21.7);        
    SetFrequency(462.5625,137000,21.7);

}

void loop() {
  // put your main code here, to run repeatedly:

}
