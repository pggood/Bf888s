// bitbang spi on standard spi pins
#define clk_pin 13
#define dat_pin 12
#define ncs_pin 10
static const byte start[][3] PROGMEM = {
     {0xea,0x0,0xa},
     {0xe0,0x80,0x0},
     {0xe,0xdf,0xfd},
     {0xe,0xdf,0xfc}
};
static const byte  initalise[] PROGMEM = {0x2,0x0,0x0,0xb5,0x25,0x8,0xcc,0x2,0x1d,0xb8,0x88,0x7,0xe7,0xd4,0x51,0xff,0x40,0xa1,0xcf,0x1c,0xc0,0x81,0x30,0x0,0x0,0x80,0x30,0x8,0xc0,0x0,0x0,0x37,0xf3,0x8,0x2,0x41,0x54,0x80,0x0};
static const byte finish[][3] PROGMEM = {
     {0xe0,0xa0,0x0},
     {0x50,0x90,0x20},
     {0x58,0x9a,0x3a},
     {0x5a,0x1e,0x80},
     {0x86,0xda,0x1d},
     {0x92,0x27,0xd},
     {0x8a,0x7,0xff},
     {0x8e,0x17,0x20},
     {0xb2,0xf8,0xa1},
     {0xb6,0x25,0x51},
     {0xd8,0x81,0xf2},
     {0xdc,0x0,0x22},
     {0xfc,0x0,0x49},
     {0xfe,0x6f,0xe6},
     {0x54,0x0,0x0},
     {0x54,0x4,0x20},
     {0x54,0x8,0x3f},
     {0x54,0xc,0x5e},
     {0x54,0x10,0x7c},
     {0x54,0x14,0x9a},
     {0x54,0x18,0xb7},
     {0x54,0x1c,0xd4},
     {0x54,0x20,0xf0},
     {0x54,0x25,0xb},
     {0x54,0x29,0x27},
     {0x54,0x2d,0x41},
     {0x54,0x31,0x5c},
     {0x54,0x35,0x75},
     {0x54,0x39,0x8f},
     {0x54,0x3d,0xa7},
     {0x54,0x41,0xc0},
     {0x54,0x45,0xd7},
     {0x54,0x49,0xef},
     {0x54,0x4e,0x5},
     {0x54,0x52,0x1b},
     {0x54,0x56,0x31},
     {0x54,0x5a,0x46},
     {0x54,0x5e,0x5b},
     {0x54,0x62,0x6f},
     {0x54,0x66,0x83},
     {0x54,0x6a,0x96},
     {0x54,0x6e,0xa9},
     {0x54,0x72,0xbb},
     {0x54,0x76,0xcd},
     {0x54,0x7a,0xde},
     {0x54,0x7e,0xef},
     {0x54,0x82,0xff},
     {0x54,0x87,0xf},
     {0x54,0x8b,0x1e},
     {0x54,0x8f,0x2d},
     {0x54,0x93,0x3b},
     {0x54,0x97,0x49},
     {0x54,0x9b,0x56},
     {0x54,0x9f,0x63},
     {0x54,0xa3,0x6f},
     {0x54,0xa7,0x7b},
     {0x54,0xab,0x86},
     {0x54,0xaf,0x91},
     {0x54,0xb3,0x9b},
     {0x54,0xb7,0xa5},
     {0x54,0xbb,0xae},
     {0x54,0xbf,0xb7},
     {0x54,0xc3,0xbf},
     {0x54,0xc7,0xc7},
     {0x54,0xcb,0xce},
     {0x54,0xcf,0xd5},
     {0x54,0xd3,0xdb},
     {0x54,0xd7,0xe1},
     {0x54,0xdb,0xe6},
     {0x54,0xdf,0xeb},
     {0x54,0xe3,0xef},
     {0x54,0xe7,0xf3},
     {0x54,0xeb,0xf6},
     {0x54,0xef,0xf9},
     {0x54,0xf3,0xfb},
     {0x54,0xf7,0xfd},
     {0x54,0xfb,0xfe},
     {0x54,0xff,0xff}
};
static const byte RX_reset[3] PROGMEM = {0xE0,0x80,0x00};
static const byte RX[3] PROGMEM = {0xE0,0xE0,0x00};
static const byte 12Khz[3] PROGMEM = {0x02,0x80,0x00};
bool Initalise(){
  pinMode(clk_pin, OUTPUT);
  pinMode(dat_pin, OUTPUT);
  pinMode(ncs_pin, OUTPUT);
  uint8_t temp;

  for (byte i = 0; i < sizeof(initalise) - 1; i++) { 
  digitalWrite(ncs_pin, 0);
    for (byte j = 0; j<8; j++) {
      temp = ((pgm_read_word_near(&initalise[i]) & (0x80 >> j)) != 0);
      digitalWrite(clk_pin, 0); 
      digitalWrite(dat_pin, temp);
      digitalWrite(clk_pin, 1); 
    }
  }
  digitalWrite(clk_pin, 0); 
  digitalWrite(ncs_pin, 1);

    for (byte h = 0; h < sizeof(finish)/3; h++) {
     
      digitalWrite(ncs_pin, 0);
      
      for (byte i = 0; i < 3; i++) {
        for (byte j = 0; j<8; j++) {
          temp = ((pgm_read_word_near(&finish[h][i]) & (0x80 >> j)) != 0);
          digitalWrite(clk_pin, 0); 
          digitalWrite(dat_pin, temp);
          digitalWrite(clk_pin, 1);
        } 
     }
     digitalWrite(clk_pin, 0);
     digitalWrite(ncs_pin, 1); 
  }
}
  
int8_t HSreadWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data)
{
  //return I2Cdev::readWord(devAddr, regAddr, data);
  
  uint8_t temp;
  uint16_t temp_dat;
  // bitbang for great justice!
  *data = 0;
  pinMode(dat_pin, OUTPUT);
  regAddr = regAddr | (1 << 7);
  
  digitalWrite(devAddr, 0); //PORTC &= ~(1<<1); //devAddr used as chip select
  for (int i = 0; i < 8; i++) {
    temp = ((regAddr & (0x80 >> i)) != 0);
    digitalWrite(clk_pin, 0); //PORTC &= ~(1<<5); //
    digitalWrite(dat_pin, temp);
    digitalWrite(clk_pin, 1); //PORTC |= (1<<5); //
  }
  // change direction of dat_pin
  pinMode(dat_pin, INPUT); // DDRC &= ~(1<<4); //
  for (int i = 15; i >= 0; i--) {
    digitalWrite(clk_pin, 0); //PORTC &= ~(1<<5); //
    digitalWrite(clk_pin, 1); //PORTC |= (1<<5); //
    temp_dat = digitalRead(dat_pin); //((PINC & (1<<4)) != 0);
    temp_dat = temp_dat << i;
    *data |= temp_dat;
  }
  digitalWrite(devAddr, 1); //PORTC |= (1<<1);// CS
  
  return 1;
}
bool HSwriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
  //return I2Cdev::writeWord(devAddr, regAddr, data);
  
  uint8_t temp_reg;
  uint16_t temp_dat;
  
  //digitalWrite(13, HIGH);
  
  // bitbang for great justice!
  pinMode(dat_pin, OUTPUT);
  regAddr = regAddr & ~(1 << 7);
  
  digitalWrite(devAddr, 0); // PORTC &= ~(1<<1); //CS
  for (int i = 0; i < 8; i++) {
    temp_reg = ((regAddr & (0x80 >> i)) != 0);
    digitalWrite(clk_pin, 0); //PORTC &= ~(1<<5); //
    digitalWrite(dat_pin, regAddr & (0x80 >> i));
    digitalWrite(clk_pin, 1); // PORTC |= (1<<5); //
  }
  for (int i = 0; i < 16; i++) {
    temp_dat = ((data & (0x8000 >> i)) != 0);
    digitalWrite(clk_pin, 0); //PORTC &= ~(1<<5); //
    digitalWrite(dat_pin, temp_dat);
    digitalWrite(clk_pin, 1); // PORTC |= (1<<5); //
  }
  
  digitalWrite(devAddr, 1); //PORTC |= (1<<1); //CS
  
  return true;
}
void setup() {
  
  uint8_t temp;
  Serial.begin(9600);
  digitalWrite(ncs_pin, 1);
  delay(100);
  digitalWrite(ncs_pin, 0);
    for (byte h = 0; h < sizeof(start)/3; h++) {
      digitalWrite(ncs_pin, 0);
      for (byte i = 0; i < 3; i++) {
        for (byte j = 0; j<8; j++) {
          temp = ((pgm_read_word_near(&start[h][i]) & (0x80 >> j)) != 0);
          digitalWrite(clk_pin, 0); 
          digitalWrite(dat_pin, temp);
          digitalWrite(clk_pin, 1);
        } 
     }
     digitalWrite(clk_pin, 0);
     digitalWrite(ncs_pin, 1);
  }
  
 delay(3);

  Initalise();
  delay(95);
  Initalise();

  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
