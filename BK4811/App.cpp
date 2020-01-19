#include "app.h"
#include "Spi_RW.h"
#include <Arduino.h>



/* fsk fifo */
#define FSK_FIFO_MAX_WRITE_SIZE     8   /* word */
#define FSK_THRESHOLD               4   /* word */

#define FSK_HEAD_LEN                2   /* word addr,type,size,CRCA */

/*max setting*/
#define MAX_CTCSS_NUM               255
#define MAX_CDCSS_NUM               104
#define MAX_CHN_NUM                 31
#define MAX_TOT                     8
#define MAX_VOX_THRESHOLD           8
#define MAX_MIC_THRESHOLD           255
#define MAX_SNR_THRESHOLD           63
#define MAX_RSSI_THRESHOLD          127
#define MAX_SOFT_MUTE               7
#define MAX_TX_VOLUME               31
#define MAX_RX_VOLUME               15
#define MAX_INVERSE_FREQ            7
#define MAX_ENCRYPTION              63




unsigned char EX0;
unsigned char *fsk_buffer ;
unsigned char *pInPayload ;
unsigned char pOutPayload[32];
int place = 0;
unsigned char outgoingnibble = 0;
unsigned char outgoingByte = 0;
unsigned char message_type = 0;
unsigned char pOutPayloadpos = 0;
FSK_CFG fsk_test_cfg;
unsigned char cur_test_item;
unsigned char fsk_test;
unsigned char *fsk_ptr ;
unsigned char RX_Parallel_Mode;
unsigned char OutUsbDataHit;
unsigned int  encryption;
BufferStructure OUT_BUFFER ;

unsigned char received_CTCSS_array[3];
unsigned char received_CTCSS_index = 0;

unsigned int timer_slice_1ms;

static unsigned int FSK_TX_Len   = 0;   /* transmit packet length word */
static unsigned int FSK_RX_PK_Len = 0;  /* receive packet length word */
static unsigned int FSK_index = 0;  /* word */

static unsigned char CTCSS_1st_found = 0;
unsigned char b_3_CTCSS_send;
unsigned char fsk_tx_begin    = 0;
unsigned char fsk_rx_id = 0;
unsigned char fsk_rx_count    = 0;
unsigned char fsk_times       = 0;
extern unsigned char fsk_single_send_busy;
extern unsigned char fsk_contiuous_busy;
unsigned int g_reg0_15_buff[16];
static const unsigned int reg0_18[] =
{
  0x9100,
  0x0000,
  0xB525,//2,updated
  0x08CC,//3,updated
  0x021D,//4,updated
  0xB888,//5,updated
  0x07E7,//6,updated
  0xD051,//7,NOT read
  0xFF40,//8,NOT read
  0xA1FF,//9,updated
  0x1C40,//10,updated
  0x8130,//11,updated
  0x0000,//12,NOT read
  0x8030,//13,NOT read
  0x08C0,//14,NOT read
  0x0000,//15,NOT read
  0x0000,
  0x8800,
  0x4068
};



static const unsigned int ramp_table[] =
{
  0x0000,
  0x0420,
  0x083F,
  0x0C5E,
  0x107C,
  0x149A,
  0x18B7,
  0x1CD4,
  0x20F0,
  0x250B,
  0x2927,
  0x2D41,
  0x315C,
  0x3575,
  0x398F,
  0x3DA7,
  0x41C0,
  0x45D7,
  0x49EF,
  0x4E05,
  0x521B,
  0x5631,
  0x5A46,
  0x5E5B,
  0x626F,
  0x6683,
  0x6A96,
  0x6EA9,
  0x72BB,
  0x76CD,
  0x7ADE,
  0x7EEF,
  0x82FF,
  0x870F,
  0x8B1E,
  0x8F2D,
  0x933B,
  0x9749,
  0x9B56,
  0x9F63,
  0xA36F,
  0xA77B,
  0xAB86,
  0xAF91,
  0xB39B,
  0xB7A5,
  0xBBAE,
  0xBFB7,
  0xC3BF,
  0xC7C7,
  0xCBCE,
  0xCFD5,
  0xD3DB,
  0xD7E1,
  0xDBE6,
  0xDFEB,
  0xE3EF,
  0xE7F3,
  0xEBF6,
  0xEFF9,
  0xF3FB,
  0xF7FD,
  0xFBFE,
  0xFFFF
};



static void BK_Disable_Inverse_Frequency(unsigned char dir);
static void BK_Disable_Encryption();
static void BK_Enable_Encryption();
static void Set_FSK_Air_mode(unsigned char b_enable);
static unsigned char Print_Recv_CTCSS();

void disable_fsk_test()
{
  fsk_test_cfg.fsk_syncword   = 0xFFFF;
  fsk_test_cfg.fsk_scrmb      = 0xFF;
  fsk_test_cfg.fsk_addr       = 0xFF;
  fsk_test_cfg.fsk_type       = 0xFF;
  fsk_test_cfg.fsk_thrshld    = 0xFF;
  fsk_test_cfg.fsk_len        = 0xFF;
  fsk_test_cfg.fsk_delay      = 0xFF;
  fsk_test_cfg.fsk_times      = 0xFFFF;
  fsk_test_cfg.fsk_is_scrmb   = 0xFF;
  fsk_test_cfg.fsk_dir        = 0xFF;
}
/*************************************************
  Function:       debug print
  Description:    send debug to serial port
  Input:          registry and number of values

  Output:         None
  Return:         None
*************************************************/

void USBPrint32B(unsigned char registry, unsigned char values) {
  Serial.print(registry, HEX);
  Serial.print("-");
  for (byte j = 0; j < values; j++) {
    Serial.print(pInPayload[j], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
void USBPrint255B(unsigned char registry, unsigned char value) {
  Serial.print(registry, HEX);
  Serial.print("-");
  Serial.print(value, HEX);
  Serial.println();
}
/*************************************************
  Function:       EnableGlobalIntr
  Description:    Enable global interrupt
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void EnableGlobalIntr(unsigned short intr_value)
{

  EX0 = 1;

  BK_Enable_Intr(intr_value);
}

/*************************************************
  Function:       DisableGlobalIntr
  Description:    Disable global interrupt
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void DisableGlobalIntr()
{

  EX0 = 0;

  BK_Enable_Intr(0);
}

/*************************************************
  Function:       BK_Ramp_Table_Init
  Description:    ramp table initilaize
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Ramp_Table_Init()
{
  unsigned char i;
  for (i = 0; i < 64; i++)
    BK_Write_Reg(42, ramp_table[i]);
}

/*************************************************
  Function:       BK_RampUp_Enable
  Description:    ramp  up enable
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_RampUp_Enable()
{
  unsigned int val = 0;


  val = BK_Read_Reg(43);
  val &= 0x3FFF;
  val |= 0x8000;
  BK_Write_Reg(43, val);


  delay(20);



}

/*************************************************
  Function:       BK_RampDown_Enable
  Description:    ramp  down enable
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_RampDown_Enable()
{

  unsigned int val = 0;
  delay(20);


  val = BK_Read_Reg(43);
  val &= 0x3FFF;
  val |= 0x4000;
  BK_Write_Reg(43, val);

  delay(20);



}

/*************************************************
  Function:       BK_Analog_Init
  Description:    analog register initilaize
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Analog_Init()
{
  unsigned int val = 0;
  unsigned char i;

  for (i = 1; i <= 15; i++)
  {
    g_reg0_15_buff[i] = reg0_18[i];
    BK_Write_Reg(i, g_reg0_15_buff[i]);
  }

  BK_Ramp_Table_Init();

  BK_Write_Reg(16, 0x3684);//0x3684:IF=88K,0x4B80: IF=121.875kHz, 0x05CF: IF=9.375kHz
  BK_Write_Reg(17, 0x8800);//updated 4812
  BK_Write_Reg(18, 0x406c);//updated 4812
  BK_Write_Reg(19, 0x8000);//updated 4812
  BK_Write_Reg(20, 0x0000);//updated

  BK_Write_Reg(21, 0x0000);
  BK_Write_Reg(22, 0x3200);//updated
  BK_Write_Reg(23, 0x2000);//updated
  BK_Write_Reg(24, 0x086c);
  BK_Write_Reg(25, 0x13ba);
  BK_Write_Reg(26, 0x0000);
  BK_Write_Reg(27, 0x0000);
  BK_Write_Reg(28, 0x0000);
  BK_Write_Reg(29, 0x0000);
  BK_Write_Reg(30, 0x0000);

  BK_Write_Reg(31, 0x8000);
  BK_Write_Reg(32, 0x0000);
  BK_Write_Reg(33, 0x0000);
  BK_Write_Reg(34, 0x0740);
  BK_Write_Reg(35, 0x0000);
  BK_Write_Reg(36, 0x8000);//updated
  BK_Write_Reg(37, 0x04D5);//updated
  BK_Write_Reg(38, 0xE000);//updated,bit13  0: 23 bits CDCSS,1: 24 bits CDCSS ,121203
  BK_Write_Reg(39, 0x0013);//updated
  BK_Write_Reg(40, 0x9020);//updated

  BK_Write_Reg(41, 0x0000);
  //BK_Write_Reg(42, 0x0000);//updated
  BK_Write_Reg(43, 0x403F);//updated
  BK_Write_Reg(44, 0xBA24);//updated 4812
  BK_Write_Reg(45, 0x1900);//updated 4812
  BK_Write_Reg(46, 0x0000);
  BK_Write_Reg(47, 0x0000);

  BK_Write_Reg(64, 0x0000);
  BK_Write_Reg(65, 0x0000);
  BK_Write_Reg(66, 0xD083);//updated
  BK_Write_Reg(67, 0xDA1D);//updated
  BK_Write_Reg(68, 0x0000);
  BK_Write_Reg(69, 0x07FF);//updated
  BK_Write_Reg(70, 0x0000);

  BK_Write_Reg(71, 0x1E20);//updated
  BK_Write_Reg(72, 0x2006);//updated
  BK_Write_Reg(73, 0x270D);//updated
  BK_Write_Reg(74, 0x0000);//updated
  BK_Write_Reg(75, 0x7A80);
  BK_Write_Reg(76, 0xE204);
  BK_Write_Reg(77, 0x0000);
  BK_Write_Reg(78, 0x800F);
  BK_Write_Reg(79, 0x0000);
  BK_Write_Reg(80, 0x0000);

  BK_Write_Reg(81, 0xB200);
  BK_Write_Reg(82, 0x0000);
  BK_Write_Reg(83, 0x8000);
  BK_Write_Reg(84, 0xFC40);
  BK_Write_Reg(85, 0x70a6);
  BK_Write_Reg(86, 0x03a0);
  BK_Write_Reg(87, 0x800F);
  BK_Write_Reg(88, 0x0000);
  BK_Write_Reg(89, 0xFF81);//updated
  BK_Write_Reg(90, 0x7D26);//updated

  BK_Write_Reg(91, 0x09AA);//updated
  BK_Write_Reg(92, 0x8000);//updated
  BK_Write_Reg(93, 0x0000);
  BK_Write_Reg(94, 0x8C00);//updated
  BK_Write_Reg(95, 0x0237);
  BK_Write_Reg(96, 0x0080);
  BK_Write_Reg(97, 0x0000);
  BK_Write_Reg(98, 0x0000);
  BK_Write_Reg(99, 0x0000);
  BK_Write_Reg(100, 0x0000);

  BK_Write_Reg(101, 0x0000);
  BK_Write_Reg(102, 0x0000);
  BK_Write_Reg(103, 0x0000);
  BK_Write_Reg(104, 0x0000);
  BK_Write_Reg(105, 0x0000);
  BK_Write_Reg(106, 0xCC13);//Firmware Version
  BK_Write_Reg(107, 0x0000);
  BK_Write_Reg(108, 0x81F2);//updated
  BK_Write_Reg(109, 0x143C);//updated
  BK_Write_Reg(110, 0x0022);//updated,121203

  BK_Write_Reg(111, 0x0000);
  BK_Write_Reg(112, 0xA000);//updated
  BK_Write_Reg(113, 0x4B83);//updated
  BK_Write_Reg(114, 0x8401);//updated
  BK_Write_Reg(115, 0x0000);//updated
  BK_Write_Reg(116, 0x0000);//updated
  BK_Write_Reg(117, 0x0000);//updated
  //BK_Write_Reg(118, 0x0400);
  BK_Write_Reg(119, 0x0000);
  BK_Write_Reg(120, 0x00C9);

  BK_Write_Reg(121, 0x0000);
  BK_Write_Reg(122, 0x46a3 );
  BK_Write_Reg(123, 0x0002);//updated
  BK_Write_Reg(124, 0x76B2);//updated
  BK_Write_Reg(125, 0x0000);
  BK_Write_Reg(126, 0x0048);//updated
  BK_Write_Reg(127, 0x6D08);//updated
}

/*************************************************
  Function:       BK_SPI_Trigger
  Description:   SPI trigger for VCO calibration
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_SPI_Trigger()
{
  unsigned int cur_val = 0;

  cur_val = g_reg0_15_buff[3]; //Keep the current value

  g_reg0_15_buff[3] &= 0x1fff; //Set LOsel<2:0> to 0 for calibration
  BK_Write_Reg(3, g_reg0_15_buff[3]);

#ifdef FOR_BK4811_12V1X
  g_reg0_15_buff[5] &= 0xFBFF; //REG5<10>=0
  BK_Write_Reg(5, g_reg0_15_buff[5]);
  g_reg0_15_buff[5] |= 0x0400; //REG5<10>=1
  BK_Write_Reg(5, g_reg0_15_buff[5]);
  g_reg0_15_buff[5] &= 0xFBFF; //REG5<10>=0
  BK_Write_Reg(5, g_reg0_15_buff[5]);
#else

#ifdef FOR_BK4811_MP_8200
  g_reg0_15_buff[4] &= 0xFBFF; //REG4<10>=0
  BK_Write_Reg(4, g_reg0_15_buff[4]);
  g_reg0_15_buff[4] |= 0x0400; //REG4<10>=1
  BK_Write_Reg(4, g_reg0_15_buff[4]);
  g_reg0_15_buff[4] &= 0xFBFF; //REG4<10>=0
  BK_Write_Reg(4, g_reg0_15_buff[4]);
#endif

#endif



  delay(5);

  g_reg0_15_buff[3] = cur_val;
  BK_Write_Reg(3, g_reg0_15_buff[3]);

}

/*************************************************
  Function:       BK_Set_Channel_Space
  Description:    select channel spacing
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Set_Channel_Space(CHN_SPC chn_spc)
{
  unsigned int val = 0;

  val = BK_Read_Reg(1);
  val &= 0x7FFF;

  if (SPACING_25K == chn_spc)
  {
    val |= 0x8000;
  }

  BK_Write_Reg(1, val);
}


/*************************************************
  Function:       BK_RX2TX
  Description:    switch rx mode to tx mode
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_RX2TX()
{
  unsigned int val = 0;

  /* work in tx mode */
  val = BK_Read_Reg(112);
  val |= 0x4000;//reg112<14>=1(TX)
  BK_Write_Reg(112, val);

  BK_Reset();//RESET digital state machine

  g_reg0_15_buff[7] |= 0x1; //reg7<0>=1, High supply LDO
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  g_reg0_15_buff[7] |= 0x2f08; //reg7<3,8,9,10,11,13>=1
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  g_reg0_15_buff[7] &= 0xFF1B; //reg7<2,5,6,7>=0,
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  BK_SPI_Trigger();

  g_reg0_15_buff[7] &= 0xffef; //reg7<4>=0, Power up TXRF
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  BK_RampUp_Enable();
}

/*************************************************
  Function:       BK_TX2RX
  Description:    switch tx mode to rx mode
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_TX2RX()
{
  unsigned char vox_enabled;
  unsigned int val;

  BK_RampDown_Enable();

  g_reg0_15_buff[7] |= 0x1; //reg7<0>=1, High supply LDO
  BK_Write_Reg(7, g_reg0_15_buff[7]);


  g_reg0_15_buff[7] &= 0xd053; //reg7<2,3,5,7,8.9,10,11,13>=0
  g_reg0_15_buff[7] |= 0x50; //reg7<4,6>=1
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  BK_SPI_Trigger();

  /* mode switch Tx to Rx and reset state machine*/
  val = BK_Read_Reg(112);
  val &= 0xBFFF;//reg112<14>=0(RX),reg112<13>=0(RESET)
  BK_Write_Reg(112, val);

  BK_Reset();//RESET digital state machine

  BK_Disable_Hard_Mute();

  val = BK_Read_Reg(22);
  vox_enabled = (val & 0x8000) >> 8;
  if (vox_enabled)
  {
    g_reg0_15_buff[7] &= 0xffbf; // reg7<6> =0
  }
  else
  {
    g_reg0_15_buff[7] |= 0x0040; //reg7<6> =1

  }
  BK_Write_Reg(7, g_reg0_15_buff[7]);
}

/*************************************************
  Function:       BK_Switch_TX_Type
  Description:    enter tx mode and selcect one tx type
  Input:          type:
                        SPEECH
                        DTMF,
                        SELCALL,
                        FSK,
                        CTCSS,
                        CDCSS,
  Output:         None
  Return:         None
*************************************************/
void BK_Switch_TX_Type(SIGNAL_TYPE type)
{
  unsigned int val = 0;


  switch (type)
  {
    case SPEECH:
      val = BK_Read_Reg(40);
      val &= 0x9FFF;
      val |= 0x8000;              /* inband type bit14,13 :00 */
      BK_Write_Reg(40, val);      /* Speech */
      break;

    case DTMF:
      val = BK_Read_Reg(40);
      val &= 0x1FFF;
      val |= 0x2000;              /* bit14,13 :01 */  //revised 2009.12.09, inband send was enabled in BK_DTMF_TX
      BK_Write_Reg(40, val);      /* DTMF */
      break;

    case SELCALL:
      val = BK_Read_Reg(40);
      val &= 0x1FFF;
      val |= 0x6000;                          /* bit14,13 :11 */ //revised 2009.12.09, inband send was enabled in BK_SELCALL_TX
      BK_Write_Reg(40, val);      /* SELCALL */
      break;

    case FSK:
      val = BK_Read_Reg(40);
      val &= 0x1FFF;
      val |= 0x4000;                          /* bit14,13 :10 *///revised 2009.12.09, inband send was enabled in BK_FSK_TX_Init
      BK_Write_Reg(40, val);      /* FSK */
      break;

    case CTCSS:
      val = BK_Read_Reg(40);
      val &= 0xFF3F;
      val |= 0x80;                            /* bit6 0:CTCSS 1:CDCSS and disable sub audio */
      BK_Write_Reg(40, val);      /* CTCSS */
      break;

    case CDCSS:
      val = BK_Read_Reg(40);
      val &= 0xFF3F;
      val |= 0xc0;                            /* bit6 0:CTCSS 1:CDCSS and disable sub audio */
      BK_Write_Reg(40, val);      /* CTCSS */
      break;

    default:
      break;
  }

  BK_RX2TX();

  delay(10);//delay 10ms

  BK_RampUp_Enable();

}

/*************************************************
  Function:       BK_Switch_RX_Type
  Description:    enter rx mode and selcect one rx type
  Input:          type:
                        SPEECH
                        DTMF,
                        SELCALL,
                        FSK,
                        CTCSS,
                        CDCSS,
  Output:         None
  Return:         None
*************************************************/
void BK_Switch_RX_Type(SIGNAL_TYPE type)
{
  unsigned int val = 0;


  switch (type)
  {
    case SPEECH:
      break;

    case DTMF:
      break;

    case SELCALL:
      break;

    case FSK:
      break;

    case CTCSS:

      val  = BK_Read_Reg(66);
      val &= 0xFFE7;
      val |= 0x0010;
      BK_Write_Reg(66, val);  //select ctcss

      break;

    case CDCSS:
      val  = BK_Read_Reg(66);
      val &= 0xFFE7;
      val |= 0x0018;
      BK_Write_Reg(66, val);  //select cdcss

      break;
  }

  BK_TX2RX();

}

/*************************************************
  Function:       BK_VOX_RX2TX
  Description:    vox active switch rx mode to tx mode
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_VOX_RX2TX()
{

  BK_Switch_TX_Type(SPEECH);
}

/*************************************************
  Function:       BK_VOX_TX2RX
  Description:    after vox active, when vox inactive
                  switch tx mode to rx mode
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_VOX_TX2RX()
{

  BK_Switch_RX_Type(SPEECH);
}

/*************************************************
  Function:       BK_Enable_Intr
  Description:    enable chip interrupt
  Input:          enable_value : interrupt to be enable

  Output:         None
  Return:         None
*************************************************/
void BK_Enable_Intr(unsigned short enable_value)
{
  BK_Write_Reg(115, enable_value);
}

/*************************************************
  Function:       BK_Clear_Intr
  Description:    clear interrupt status
  Input:          intr_value : interrupt to be clear

  Output:         None
  Return:         None
*************************************************/
void BK_Clear_Intr(unsigned int intr_value)
{
  BK_Write_Reg(116, intr_value);
}

/*************************************************
  Function:       BK_Enable_TX_InbandSignal
  Description:    enable tx inbandsignal
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Enable_TX_InbandSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(40);

  val |= 0x8000;

  BK_Write_Reg(40, val);
}

/*************************************************
  Function:       BK_Disable_TX_InbandSignal
  Description:    disable tx inbandsignal
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Disable_TX_InbandSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(40);

  val &= 0x7FFF;

  BK_Write_Reg(40, val);
}

/*************************************************
  Function:       BK_Is_RX_SubAudioSignal
  Description:
  Input:          None

  Output:         None
  Return:         0: RX SubAudioSignal
                  1: not RX SubAudioSignal
*************************************************/
unsigned char BK_Is_RX_SubAudioSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(66);
  val &= 0x0010;
  if (val != 0)
    return 1;
  else
    return 0;
}

/*************************************************
  Function:       BK_Is_TX_SubAudioSignal
  Description:
  Input:          None

  Output:         None
  Return:         0: TX SubAudioSignal
                  1: not TX SubAudioSignal
*************************************************/
unsigned char BK_Is_TX_SubAudioSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(40);
  val &= 0x0080;
  if (val != 0)
    return 1;
  else
    return 0;
}

/*************************************************
  Function:       BK_Enable_TX_SubAudioSignal
  Description:    enable tx subaudio signal
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Enable_TX_SubAudioSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(40);

  val |= 0x0080;

  BK_Write_Reg(40, val);
}

/*************************************************
  Function:       BK_Disable_TX_SubAudioSignal
  Description:    disable tx subaudio signal
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Disable_TX_SubAudioSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(40);

  val &= 0xfF7F;

  BK_Write_Reg(40, val);
}

/*************************************************
  Function:       BK_Enable_RX_SubAudioSignal
  Description:    enable rx subaudio signal
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Enable_RX_SubAudioSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(66);

  val |= 0x0010;

  BK_Write_Reg(66, val);
}

/*************************************************
  Function:       BK_Disable_RX_SubAudioSignal
  Description:    disable rx subaudio signal
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Disable_RX_SubAudioSignal()
{
  unsigned int val = 0;
  val = BK_Read_Reg(66);

  val &= 0xFFEF;

  BK_Write_Reg(66, val);
}

/*************************************************
  Function:       BK_Is_Soft_Mute
  Description:    whether soft mute enabled
  Input:          None

  Output:         None
  Return:         1 : enable
                  0 : disable
*************************************************/
unsigned char BK_Is_Soft_Mute()
{
  if (BK_Read_Reg(73) & 0x4000)
    return 1;
  else
    return 0;
}

/*************************************************
  Function:       BK_Disable_Soft_Mute
  Description:    disable soft mute
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Disable_Soft_Mute()
{
  unsigned int val = 0;
  val = BK_Read_Reg(73);
  val &= 0xBFBF;
  BK_Write_Reg(73, val);
}

/*************************************************
  Function:       BK_Enable_Soft_Mute
  Description:    enable soft mute
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Enable_Soft_Mute()
{
  unsigned int val = 0;
  val = BK_Read_Reg(73);
  val |= 0x4040;
  BK_Write_Reg(73, val);
}

/*************************************************
  Function:       BK_SELCALL_Read
  Description:    SELCALL read
  Input:          None

  Output:         None
  Return:         selcall_addr  received SELCALL address
*************************************************/
unsigned char BK_SELCALL_Read()
{
  unsigned char  selcall_addr = 0;
  unsigned int val = 0;

  val = BK_Read_Reg(87);

  selcall_addr = (val >> 4) & 0x0F; //bit7-4

  return selcall_addr;
}

/*************************************************
  Function:       BK_Enable_Hard_Mute
  Description:    enable hard mute
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Enable_Hard_Mute()
{
  unsigned int val = 0;

  val = BK_Read_Reg(73);
  val |= 0x8000;
  BK_Write_Reg(73, val);
}

/*************************************************
  Function:       BK_Disable_Hard_Mute
  Description:    disable hard mute
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Disable_Hard_Mute()
{
  unsigned int val = 0;

  val = BK_Read_Reg(73);
  val &= 0x7FFF;
  BK_Write_Reg(73, val);
}

/*************************************************
  Function:       BK_CTCSS_Read
  Description:    CTCSS read
  Input:          None

  Output:         None
  Return:         ctcss symbol address
*************************************************/
unsigned char BK_CTCSS_Read()
{
  unsigned int val = 0;

  if (b_3_CTCSS_send)
  {

    if (received_CTCSS_index > 0)
    {
      if (timer_slice_1ms > 600) ////if the time between two CTCSS received >1000ms,discard the last symbol
      {
        received_CTCSS_index = 0; //reset index
        received_CTCSS_array[0] = 0;
        received_CTCSS_array[1] = 0;
        received_CTCSS_array[2] = 0;
      }
    }

    timer_slice_1ms = 0;

    val = BK_Read_Reg(92);
    val = (val >> 4) & 0x000F; //bit7-4

    if (received_CTCSS_index > 0)
    {
      if (received_CTCSS_array[received_CTCSS_index - 1] == val) //if received two same symbol,discard the last symbol
      {
        received_CTCSS_index -= 1;
      }
    }
    received_CTCSS_array[received_CTCSS_index] = val;

    received_CTCSS_index++;

    if (received_CTCSS_index > 2)
    {
      if ( (received_CTCSS_array[0] != received_CTCSS_array[1]) &&
           (received_CTCSS_array[1] != received_CTCSS_array[2])  ) //if symbol 0 differ with symbol1 ,and symbol 1 differ with symbol2
      {
        Print_Recv_CTCSS();
        CTCSS_1st_found = 1;
        BK_Disable_Hard_Mute();
      }
      received_CTCSS_index = 0;
    }

  }
  else
  {
    val = BK_Read_Reg(92);
    val = (val >> 4) & 0x000F; //bit7-4
    BK_Disable_Hard_Mute();
    CTCSS_1st_found = 1;

    pInPayload[0] = 1;
    pInPayload[1] = (unsigned char)val;
    USBPrint32B(BKN_DEMO_READ_CTCSS, 2);


  }

  return val;
}


/*************************************************
  Function:       BK_FSK_Read_Init
  Description:    prepare for FSK rx
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_FSK_Read_Init()
{
  unsigned int val = 0;

  fsk_buffer = pInPayload;

  FSK_index = 0;

  val = BK_Read_Reg(80);
  FSK_RX_PK_Len = (val >> 9) & 0x007F;

  fsk_buffer[0] = (BK_Read_Reg(79) & 0xFF00) >> 8;  /* Addr */
  fsk_buffer[1] = (BK_Read_Reg(79) & 0x00FF);     /* Type */
  fsk_buffer[2] = FSK_RX_PK_Len * 2;                          /* Size byte */
  fsk_buffer[3] = (BK_Read_Reg(80) & 0x00FF);     /* CRCA */
  FSK_index += 2; /* word */
}

/*************************************************
  Function:       BK_FSK_Read_FIFO
  Description:    begin FSK read
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_FSK_Read_FIFO()
{
  unsigned int val = 0, dummy = 0, i = 0;

  val = BK_Read_Reg(83);

  while ((val & 0x000F) > 0)
  {
    if (FSK_index < (FSK_RX_PK_Len + FSK_HEAD_LEN)) /* +2 for read the addr,type,size,crca */
    {
      val = BK_Read_Reg(82);
      *((unsigned int *)fsk_buffer + FSK_index) = val;
      FSK_index++;
    }
    else
    {
      dummy = BK_Read_Reg(82);
    }
    val = BK_Read_Reg(83);
  }
}

/*************************************************
  Function:       BK_DTMF_RX_Read
  Description:    begin DTMF read
  Input:          None

  Output:         None
  Return:         dtmf_addr received dtmf address
*************************************************/
unsigned char BK_DTMF_RX_Read()
{
  unsigned char  dtmf_addr = 0;
  unsigned int val = 0;

  val = BK_Read_Reg(78);

  dtmf_addr = (val >> 4) & 0x0F; //bit7-4

  return dtmf_addr;
}


/*************************************************
  Function:       BK_Intr_Task
  Description:    interrupt service routine
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Intr_Task()
{
  unsigned int intr_value = 0;
  unsigned int reg_value = 0;
  unsigned char i = 0, j = 0;
  unsigned char return_value = 0;
  unsigned char fsk_rx_noerror  = 0;


  EX0 = 0;

  intr_value = BK_Read_Reg(116);
  //BK_Clear_Intr(0xFFFF);

  if (intr_value & INTS_FSK_TX_SUCCESS)
  {
    BK_Clear_Intr(INTS_FSK_TX_SUCCESS);

    reg_value = 0;
    //  reg_value = BK_Read_Reg(29);


    //  reg_value = BK_Read_Reg(31);
    delay(10);
    reg_value = BK_Read_Reg(40);
    reg_value &= 0x7FFF;                            /* first switch rx mode for v6 chip*/
    //      reg_value |= 0x6000;
    BK_Write_Reg(40, reg_value);

    DisableGlobalIntr();
    FSK_index = FSK_TX_Len = 0; //very important,otherwise,INTS_FSK_FIFO_NEEDFILL will processed
    fsk_tx_begin = 0;


    fsk_single_send_busy = 0;

    if (fsk_contiuous_busy)
    {

      if (0 == fsk_times)
        fsk_test = 0;
      else
        fsk_test = 1;

      if (0 == fsk_test_cfg.fsk_times)
        fsk_test = 1;



      //bExtIntr0 = 0;
    }




    return;
  }

  if (intr_value & INTS_FSK_FIFO_NEEDFILL)
  {
    BK_Clear_Intr(INTS_FSK_FIFO_NEEDFILL);

    if (FSK_index < FSK_TX_Len)
    {
      reg_value = BK_Read_Reg(31);

      if ((reg_value & 0xf) <= 4)  /* remaining words in fifo */
      {
        j = FSK_TX_Len - FSK_index;

        if (j < FSK_THRESHOLD)
        {
          for (i = 0; i < j; i++)
          {
            //                      g_reg_value = *((unsigned int *)fsk_ptr + FSK_index + i);
            BK_Write_Reg(30, *((unsigned int *)fsk_ptr + FSK_index + i));
          }
          FSK_index += j;
        }
        else
        {
          for (i = 0; i < FSK_THRESHOLD; i++)
          {
            //                      g_reg_value = *((unsigned int *)fsk_ptr + FSK_index + i);
            BK_Write_Reg(30, *((unsigned int *)fsk_ptr + FSK_index + i));
          }
          FSK_index += FSK_THRESHOLD;
        }
      }
    }

  }

  if (intr_value & INTS_FSK_HEAD_RX)
  {
    BK_Clear_Intr(INTS_FSK_HEAD_RX);
    BK_FSK_Read_Init();
  }

  if (intr_value & INTS_FSK_FIFO_NEEDREAD)
  {
    BK_FSK_Read_FIFO();
    BK_Clear_Intr(INTS_FSK_FIFO_NEEDREAD);
  }

  if (intr_value & INTS_DTMF_RX)
  {
    BK_Clear_Intr(INTS_DTMF_RX);
    return_value = BK_DTMF_RX_Read();

    pInPayload[0] = return_value;
    USBPrint32B(BKN_DEMO_READ_DTMF, 1);

  }


  if (intr_value & INTS_CTCSS_LOSS)
  {
    BK_Clear_Intr(INTS_CTCSS_LOSS);
    //      reg_value = BK_Read_Reg(92);
    /* confirm real loss */
    //      if(reg_value & 0x2000)
    BK_Enable_Hard_Mute();

    if (received_CTCSS_index == 0)
    {
      pInPayload[0] = 0;
      pInPayload[1] = (unsigned char)reg_value;
      USBPrint32B(BKN_DEMO_READ_CTCSS, 2);
    }

  }

  if (intr_value & INTS_CTCSS_RX)
  {
    /* CTCSS match */
    BK_Clear_Intr(INTS_CTCSS_RX);
    BK_CTCSS_Read();
  }


  /*    if(intr_value & CTCSS_PHASE_CHANGE_INTR)
      {
          BK_Clear_Intr(CTCSS_PHASE_CHANGE_INTR);
      }
  */

  if (intr_value & INTS_AFC_LINKLOST)
  {
    BK_Clear_Intr(INTS_AFC_LINKLOST);
    reg_value = BK_Read_Reg(69);
    reg_value |= 0x8000;

    BK_Write_Reg(69, reg_value);
    reg_value &= 0x7FFF;
    BK_Write_Reg(69, reg_value);
  }

  if (intr_value & INTS_CDCSS_RX)
  {
    BK_Clear_Intr(INTS_CDCSS_RX);
    BK_Disable_Hard_Mute();


    pInPayload[0] = 1;
    pInPayload[1] = 0;
    USBPrint32B(BKN_DEMO_READ_CDCSS, 2);


  }

  if (intr_value & INTS_CDCSS_LOSS)
  {
    BK_Clear_Intr(INTS_CDCSS_LOSS);
    BK_Enable_Hard_Mute();

    pInPayload[0] = 0;
    pInPayload[1] = 0;
    USBPrint32B(BKN_DEMO_READ_CDCSS, 2);

  }

  if (intr_value & INTS_VOX_DETECT)
  {
    BK_VOX_RX2TX();
    BK_Clear_Intr(INTS_VOX_DETECT);
  }

  if (intr_value & INTS_TOT_TIMEOUT)
  {
    BK_VOX_TX2RX();
    BK_Clear_Intr(INTS_TOT_TIMEOUT);

  }

  if (intr_value & INTS_FSK_RX_COMPLETE)
  {
    BK_Clear_Intr(INTS_FSK_RX_COMPLETE);
    BK_FSK_Read_FIFO();
    reg_value = BK_Read_Reg(83);
    if (reg_value & 0x0010)
      //    if(1)
    {

      /* Demo mode not test mode */
      if (fsk_contiuous_busy == 0)
        USBPrint255B(BKN_DEMO_READ_FSK, FSK_index * 2);
      else
        fsk_rx_noerror = 1;
      //          fsk_rx_complete = 1;

    }
    else
    {
      //memcpy(((unsigned char *)fsk_buffer + FSK_index * 2), "Error", 5);
      //          fsk_buffer[FSK_index * 2] = 'E';
      //          fsk_buffer[FSK_index * 2 + 1] = 'r';
      //          fsk_buffer[FSK_index * 2 + 2] = 'r';
      //          fsk_buffer[FSK_index * 2 + 3] = 'o';
      //          fsk_buffer[FSK_index * 2 + 4] = 'r';
      //          fsk_buffer[2] += 5;

      /* Demo mode not test mode */
      //          if(fsk_contiuous_busy==0)
      //              USBPrint255B(BKN_DEMO_READ_FSK, (FSK_index * 2 + 5));

      fsk_rx_noerror = 0;
      //          fsk_rx_complete = 1;

    }


    if ( (fsk_rx_noerror) )
    {
      if (fsk_test_cfg.fsk_times == 0) //only infinite
      {
        if (fsk_buffer[4] < fsk_rx_id)
        {
          pInPayload[0] = 0;
          pInPayload[1] = fsk_rx_count;
          USBPrint32B(BKN_DEMO_FSK_TEST_PRINT, 2);
          fsk_rx_count = 0;
        }
      }

      fsk_rx_count++;
      fsk_rx_id = fsk_buffer[4];
    }


    FSK_RX_PK_Len = 0;
    FSK_index = 0;
  }

  if (intr_value & INTS_SELCALL_RX)
  {
    BK_Clear_Intr(INTS_SELCALL_RX);
    return_value = BK_SELCALL_Read();

    pInPayload[0] = return_value;
    USBPrint32B(BKN_DEMO_READ_SELCALL, 1);

  }


  EX0 = 1;

}

static const unsigned int RecvDtmfLowArr[] PROGMEM = {0x38D, 0x2A2, 0x2A2, 0x2A2, 0x2E8, 0x2E8, 0x2E8, 0x337, 0x337, 0x337, 0x2A2, 0x2E8, 0x337, 0x38D, 0x38D, 0x38D};
static const unsigned int RecvDtmfHighArr[] PROGMEM = {0x50B, 0x490, 0x50B, 0x593, 0x490, 0x50B, 0x593, 0x490, 0x50B, 0x593, 0x62A, 0x62A, 0x62A, 0x62A, 0x490, 0x593};
static const unsigned int SendDtmfLowArr[] PROGMEM = {0xB5E, 0x86C, 0x86C, 0x86C, 0x94D, 0x94D, 0x94D, 0xA4B, 0xA4B, 0xA4B, 0x86C, 0x94D, 0xA4B, 0xB5E, 0xB5E, 0xB5E};
static const unsigned int SendDtmfHighArr[] PROGMEM = {0x1024, 0x0E9B, 0x1024, 0x11D8, 0x0E9B, 0x1024, 0x11D8, 0x0E9B, 0x1024, 0x11D8, 0x13BA, 0x13BA, 0x13BA, 0x13BA, 0x0E9B, 0x11D8};
static const unsigned int SendSELCALLArr[] PROGMEM = {0x17EE, 0x0D94, 0x0E76, 0x0F67, 0x1068, 0x1178, 0x129B, 0x13D0, 0x1518, 0x1678, 0x1cfe, 0x0b3c, 0x1b25, 0x0bf9, 0x197d, 0x0cbf};
static const unsigned int RecvSELCALLArr[] PROGMEM = {0x0bf7, 0x06CA, 0x073B, 0x07B4, 0x0834, 0x08BC, 0x094D, 0x09E8, 0x0A8D, 0x0B3C, 0x0E7F, 0x059E, 0x0D93, 0x05FC, 0x0CBF, 0x065F};



static const unsigned int SendCTCSSArr[] PROGMEM = {
  0x305, 0x320, 0x33d, 0x359, 0x379, 0x398, 0x3b9, 0x3da,
  0x3fd, 0x420, 0x447, 0x46c, 0x495, 0x4b5, 0x4d5, 0x500,
  0x52e, 0x55c, 0x58c, 0x5be, 0x5f2, 0x627, 0x65e, 0x699,
  0x6d4, 0x711, 0x751, 0x792, 0x7b9, 0x7d6, 0x81d, 0x866,
  0x8b1, 0x8de, 0x8ff, 0x92d, 0x951, 0x980, 0x9a4, 0x9d5,
  0x9fa, 0xa2e, 0xa8a, 0xae8, 0xb12, 0xb4a, 0xbaf, 0xc18,
};
static const unsigned int RecvCTCSSArr[] PROGMEM = {
  0x60a , 0x641 , 0x67a , 0x6b3 , 0x6f3 , 0x731 , 0x771 , 0x7b4 ,
  0x7f9 , 0x841 , 0x88e , 0x8d8 , 0x929 , 0x96a , 0x9aa , 0xa01 ,
  0xa5c , 0xab8 , 0xb18 , 0xb7b , 0xbe3 , 0xc4d , 0xcbd , 0xd31 ,
  0xda8 , 0xe21 , 0xea2 , 0xf25 , 0xf72 , 0xfad , 0x103a, 0x10cc,
  0x1163, 0x11bc, 0x11ff, 0x125a, 0x12a2, 0x1300, 0x1348, 0x13ab,
  0x13f5, 0x145d, 0x1514, 0x15d0, 0x1624, 0x1693, 0x175e, 0x1831,
};



static const unsigned int  CDCSS_Addr[] PROGMEM = { 023, 025,  026,  031,  032,  036,  043,  047,
                                                    051, 053,  054,  065,  071,  072,  073,  074,
                                                    0114, 0115, 0116, 0122, 0125, 0131, 0132, 0134,
                                                    0143, 0145, 0152, 0155, 0156, 0162, 0165, 0172,
                                                    0174, 0205, 0212, 0223, 0225, 0226, 0243, 0244,
                                                    0245, 0246, 0251, 0252, 0255, 0261, 0263, 0265,
                                                    0266, 0271, 0274, 0306, 0311, 0315, 0325, 0331,
                                                    0332, 0343, 0346, 0351, 0356, 0364, 0365, 0371,
                                                    0411, 0412, 0413, 0423, 0431, 0432, 0445, 0446,
                                                    0452, 0454, 0455, 0462, 0464, 0465, 0466, 0503,
                                                    0506, 0516, 0523, 0526, 0532, 0546, 0565, 0606,
                                                    0612, 0624, 0627, 0631, 0632, 0654, 0662, 0664,
                                                    0703, 0712, 0723, 0731, 0732, 0734, 0743, 0754
                                                  };


/*************************************************
  Function:       BK_DTMF_TX
  Description:    Begin DTMF TX
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_DTMF_TX(unsigned char *buf, unsigned char len)
{
  unsigned char i = 0, index = 0;

  if (0 == len)
    return;

  BK_Disable_TX_InbandSignal();

  for (i = 0; i < len ; i++)
  {
    index = buf[i];

    BK_Write_Reg(24, SendDtmfLowArr[index]);
    BK_Write_Reg(25, SendDtmfHighArr[index]);

    BK_Enable_TX_InbandSignal();

    BK_Disable_TX_InbandSignal();

    delay(40); /* delay 40ms */
  }
}

/*************************************************
  Function:       BK_DTMF_RX
  Description:    Begin DTMF RX
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_DTMF_RX()
{
  unsigned int i = 0;

  //  EnableGlobalIntr(INTM_ENABLE_INTR |
  //                   INTM_DTMF_RX|INTM_AFC_LINKLOST
  //                  );

  BK_Switch_RX_Type(DTMF);

  for (i = 0; i < 16; i++)
  {
    BK_Write_Reg(77, ((i) << 12) |
                 ((1  << 11)) |
                 RecvDtmfHighArr[i]);

    BK_Write_Reg(77, ((i) << 12) |
                 RecvDtmfLowArr[i]);
  }
}

/*************************************************
  Function:       BK_SELCALL_TX
  Description:    begin SELCALL tx
  Input:          buf: SELCALL tx buffer
                  len: SELCALL tx lenght

  Output:         None
  Return:         None
*************************************************/
void BK_SELCALL_TX(unsigned char *buf, unsigned char len)
{
  unsigned char i = 0, index = 0;
  unsigned int Set = 0;

  if (0 == len)
    return;

  BK_Enable_TX_InbandSignal();

  for (i = 0; i < len ; i++)
  {
    index = buf[i];

    BK_Write_Reg(34, SendSELCALLArr[index]);

    delay(40); /* delay 40ms */
  }

  BK_Disable_TX_InbandSignal();
}

/*************************************************
  Function:       BK_SELCALL_RX
  Description:    begin SELCALL rx
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_SELCALL_RX()
{
  unsigned int i = 0;

  //  EnableGlobalIntr(INTM_ENABLE_INTR |
  //                   INTM_SELCALL_RX|INTM_AFC_LINKLOST
  //                  );

  BK_Switch_RX_Type(SELCALL);

  for (i = 0; i < 16; i++)
  {
    BK_Write_Reg(86, (i << 12) | RecvSELCALLArr[i]);
  }
}

/*************************************************
  Function:       BK_FSK_TX_Init
  Description:    FSK TX Init
  Input:          type:
                        FSK_TX_TYPE0: Only head, no payload
                        FSK_TX_TYPE1: Head with payload
                        FSK_TX_TYPE2: Head with FEC encoded payload
                        FSK_TX_TYPE3: Head with FEC and interleaved encoded payload
                        FSK_TX_TYPE4: Free format that no automatic CRC insertion,
                                      CRCA is a user writable byte

                  len : length(byte) of FSK TX data
  Output:         None
  Return:         None

  Note:
  if use type 3, the number of payload is restricted. The allowed payload number is
  either odd number less than 8 or even number greater than 9. Payload number 8 and 9 is not
  allowed for type 3.
*************************************************/
void BK_FSK_TX_Init(FSK_TYPE type, unsigned char *buf, unsigned char len)
{
  unsigned int val = 0, align = 0;

  if (0 == len)
    return;

  /* disable inband signal */
  BK_Disable_TX_InbandSignal();

  fsk_ptr = buf;
  /* reset the write index */
  FSK_index = 0;
  /* for odd number len a 0 pad at the end of the buf*/
  fsk_ptr[len] = 0;

  /* Set FSK type */
  //  val = BK_Read_Reg(28);
  //  val &= 0xFF00;
  val = fsk_test_cfg.fsk_addr;
  val <<= 8;
  val |= (unsigned int)type;
  //  val &= 0x00FF;
  BK_Write_Reg(28, val);

  /* Set TX length */
  if (FSK_TX_TYPE3 == type)
  {
    val = BK_Read_Reg(29);
    val &= 0xff;


    align = (len + 1) & (~1);   /* first:align to 2 bytes */
    align /= 2;

    if (align <= 7)      /* 2,4,6 align to 1,3,5,7 */
    {
      if (!(align & 0x01))
        align += 1;
    }
    else if ((align == 8) || (align == 9)) /* 8,9 align to 10*/
      align = 10;
    else                /* >10 align to even words */
    {
      align = (align + 1) & (~1);
    }

    /* send length */
    FSK_TX_Len = align; /* word */

    /* new chip use bit9-15 word length */
    val |= FSK_TX_Len << 9;

    BK_Write_Reg(29, val);
  }
  else if (FSK_TX_TYPE1 == type)
  {
    val = BK_Read_Reg(29);
    len = (len + 1) & (~0x01);
    FSK_TX_Len = len / 2;
    val &= 0x00FF;
    val |= (FSK_TX_Len << 9);
    BK_Write_Reg(29, val);
  }
  else
  {
    val = BK_Read_Reg(29);
    len = (len + 1) & (~0x01);
    FSK_TX_Len = len / 2;
    val &= 0x00FF;
    val |= (FSK_TX_Len << 9);
    BK_Write_Reg(29, val);
  }

  BK_Enable_TX_InbandSignal();
}

/*************************************************
  Function:       BK_FSK_TX
  Description:    begin FSK TX
  Input:          buf: tx buffer
                  len: tx length

  Output:         None
  Return:         None
*************************************************/
void BK_FSK_TX(unsigned char *buf, unsigned char len)
{
  unsigned int val = 0;
  unsigned char i = 0;
  unsigned int fsk_val = 0;


  if (0xFF == fsk_test_cfg.fsk_dir)
    fsk_tx_begin = 1;


  len = FSK_TX_Len;

  val  = BK_Read_Reg(40);
  val &= 0x9FFF;                          /* first switch rx mode for v6 chip*/
  val |= 0x6000;
  BK_Write_Reg(40, val);


  val  = BK_Read_Reg(117);
  val |= 0x0800;
  BK_Write_Reg(117, val); /* bypass Gate clk for v6 chip*/


  /* only write once */
  if (len <= FSK_FIFO_MAX_WRITE_SIZE)
  {
    for (i = 0; i < len; i++)
    {
      fsk_val = *((unsigned int *)buf + i);
      BK_Write_Reg(30, *((unsigned int *)buf + i));
    }
    FSK_index += len;
  }
  else /* need multiple write */
  {
    for (i = 0; i < FSK_FIFO_MAX_WRITE_SIZE; i++)
    {
      fsk_val = *((unsigned int *)buf + i);
      BK_Write_Reg(30, *((unsigned int *)buf + i));
    }
    FSK_index += FSK_FIFO_MAX_WRITE_SIZE;
  }

  if (FSK_TX_Len <= FSK_FIFO_MAX_WRITE_SIZE)
  {
    EnableGlobalIntr(INTM_ENABLE_INTR |
                     INTM_FSK_TX_SUCCESS
                    );
  }
  else
  {
    EnableGlobalIntr(INTM_ENABLE_INTR |
                     INTM_FSK_TX_SUCCESS |
                     INTM_FSK_FIFO_NEEDFILL
                    );
  }

  val = BK_Read_Reg(40);
  val &= 0x9FFF;                          /* first switch rx mode for v6 chip */
  val |= 0x4000;
  BK_Write_Reg(40, val);
}

/*************************************************
  Function:       BK_FSK_RX
  Description:    begin FSK RX
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_FSK_RX()
{
  unsigned int val = 0;

  BK_Switch_RX_Type(FSK);

  BK_Clear_Intr(0xFFFF);

  //  EnableGlobalIntr(INTM_ENABLE_INTR |
  //                   INTM_FSK_HEAD_RX |
  //                   INTM_FSK_FIFO_NEEDREAD |
  //                   INTM_FSK_RX_COMPLETE|
  //                   INTM_AFC_LINKLOST
  //                  );
}

/*************************************************
  Function:       BK_CTCSS_TX
  Description:    Set CTCSS TX frequency
  Input:          index : 0: diable CTCSS TX
                          1-7:sub audio frequence index,ie.symbol address
  Output:         None
  Return:         None
*************************************************/
void BK_CTCSS_TX(unsigned char index)
{

  if (index > MAX_CTCSS_NUM)
    return;

  if (0 == index)
  {
    BK_Disable_TX_SubAudioSignal();
    return;
  }

  /* index-- for locate in the table from 0 */
  index--;

  BK_Disable_TX_SubAudioSignal();
  BK_Write_Reg(37, SendCTCSSArr[index]);

  BK_Enable_TX_SubAudioSignal();

}


/*************************************************
  Function:       BK_CTCSS_RX
  Description:    Set CTCSS RX frequency
  Input:          index:
              0:disable CTCSS
              1-7:CTCSS index,enable CTCSS

  Output:         None
  Return:         None
*************************************************/
void BK_CTCSS_RX(unsigned char index)
{
  unsigned char i = 0;
  unsigned int val = 0;
  //  float freq;

  val = BK_Read_Reg(115);

  EnableGlobalIntr(INTM_ENABLE_INTR |
                   INTM_CTCSS_RX |
                   INTM_CTCSS_LOSS |
                   INTM_AFC_LINKLOST |
                   val
                  );

  if (0 == index)
  {
    CTCSS_1st_found = 0;
    BK_Disable_RX_SubAudioSignal();
    BK_Disable_Hard_Mute();
    received_CTCSS_index = 0;
    return;
  }

  if (!BK_Is_RX_SubAudioSignal())
    BK_Enable_RX_SubAudioSignal();

  BK_Enable_Hard_Mute();

  /* index-- for locate in the table from 0 */

  //freq = CTCSS_freq[index] / 0.0404;
  if (b_3_CTCSS_send)
  {
    //      for(i=1;i<=7;i++)
    //      {
    //          BK_Write_Reg(91,((unsigned int)((unsigned int)i)<<13) +RecvCTCSSArr[i]);
    //      }
  }
  else
  {
    index--;
    BK_Write_Reg(91, ((0 << 13) + RecvCTCSSArr[index]));
  }

  received_CTCSS_index = 0;
}

/*************************************************
  Function:       BK_CDCSS_TX
  Description:    Set CDCSS TX frequency
  Input:          index
                  0:disable CDCSS
                  index in the table

  Output:         None
  Return:         None
*************************************************/
void BK_CDCSS_TX(unsigned char index)
{
  if (index > MAX_CDCSS_NUM)
    return;

  if (0 == index)
  {
    BK_Disable_TX_SubAudioSignal();
    return;
  }

  /* index-- for locate in the table from 0 */
  index--;

  //  BK_Disable_TX_SubAudioSignal();
  BK_Write_Reg(39, CDCSS_Addr[index]);
  delay(200); /* delay 40ms */
  BK_Enable_TX_SubAudioSignal();
}

/*************************************************
  Function:       BK_CDCSS_RX
  Description:    Set CDCSS RX frequency
  Input:          index
                  0:disable CDCSS
                  index in the table

  Output:         None
  Return:         None
*************************************************/
void BK_CDCSS_RX(unsigned char index)
{
  unsigned char i = 0;
  unsigned int val = 0;

  val = BK_Read_Reg(115);

  EnableGlobalIntr(INTM_ENABLE_INTR |
                   INTM_CDCSS_RX |
                   INTM_CDCSS_LOSS |
                   INTM_AFC_LINKLOST |
                   val
                  );

  if (index > MAX_CDCSS_NUM)
    return;

  if (0 == index)
  {
    BK_Disable_RX_SubAudioSignal();
    BK_Disable_Hard_Mute();
    return;
  }

  BK_Enable_Hard_Mute();

  /* index-- for locate in the table from 0 */
  index--;

  BK_Disable_RX_SubAudioSignal();
  BK_Write_Reg(39, CDCSS_Addr[index]);
  BK_Enable_RX_SubAudioSignal();

}

/*************************************************
  Function:       BK_Set_TOT
  Description:    set TOT
  Input:          timeout unit second
                  0:disable TOT
                  1:0.5, 2:1,  3:2,  4:4,
                  5:8,   6:16, 7:32, 8:64

  Output:         None
  Return:         None
*************************************************/
void BK_Set_TOT(unsigned char timeout)
{
  unsigned int val = 0;

  if (timeout > MAX_TOT)
    timeout = MAX_TOT;

  if (DISABLE_TOT == timeout) /* disable TOT */
  {
    val = BK_Read_Reg(23);
    val &= 0x7FFF;
    BK_Write_Reg(23, val);
  }
  else                /* enable TOT & set timeout value */
  {
    val = BK_Read_Reg(23);
    val &= 0x0FFF;
    val |= (0x8000 | (((timeout - 1) & 0x7) << 12));
    BK_Write_Reg(23, val);

    EnableGlobalIntr(INTM_ENABLE_INTR |
                     INTM_VOX_DETECT  |
                     INTM_TOT_TIMEOUT
                    );
  }
}

/*************************************************
  Function:       BK_Set_VOX
  Description:    set VOX threshold
  Input:          vox_threshold
                  0:disable vox threshold
                  1:2,  2:3,  3:4, 4:6,
                  5:8,  6:12, 7:16, 8:24
  Output:         None
  Return:         None
*************************************************/
void BK_Set_VOX(unsigned char vox_threshold)
{
  unsigned int val = 0;

  if (vox_threshold > MAX_VOX_THRESHOLD)
    vox_threshold = MAX_VOX_THRESHOLD;

  if (DISABLE_VOX == vox_threshold)   /* disable VOX */
  {
    val = BK_Read_Reg(22);
    val &= 0x7FFF;
    BK_Write_Reg(22, val);
    BK_Switch_RX_Type(SPEECH);
  }
  else    /* enable VOX & set VOX threshold value */
  {
    val = BK_Read_Reg(22);
    val &= 0x63FF;
    val |= (0x8000 | (((vox_threshold - 1) & 0x7) << 10));
    BK_Write_Reg(22, val);

    EnableGlobalIntr(INTM_ENABLE_INTR |
                     INTM_VOX_DETECT |
                     INTM_TOT_TIMEOUT
                    );
  }
}

/*************************************************
  Function:       BK_Set_MIC_Threshold
  Description:    set MIC threshold
  Input:          mic_threshold  threshold

  Output:         None
  Return:         None
*************************************************/
void BK_Set_MIC_Threshold(unsigned char mic_threshold)
{
  unsigned int val = 0;

  if (mic_threshold > MAX_MIC_THRESHOLD)
    mic_threshold = MAX_MIC_THRESHOLD;

  val = BK_Read_Reg(21);
  val &= 0xFF00;
  val |= (mic_threshold & 0xFF);
  BK_Write_Reg(21, val);
}



void BK_Set_Freq(unsigned char dir,  double frequency)
{
  int DIV;
  byte REG126_4;
  byte REG3_14_15;
  unsigned int REG127;
  byte chan_num;
  if (frequency < 127) return;
  if (frequency > 525) return;
  if (frequency < 525) {
    DIV = 8;
    REG126_4 = 0;
    REG3_14_15 = 0;
    REG127 = 43455;
    chan_num = 5;
  }
  if (frequency < 360) {
    DIV = 12;
    REG126_4 = 0;
    REG3_14_15 = 1;
    REG127 = 65182;
    chan_num = 4;
  }
  if (frequency < 262) {
    DIV = 16;
    REG126_4 = 1;
    REG3_14_15 = 2;
    REG127 = 21373;
    chan_num = 3;
  }
  if (frequency < 210) {
    DIV = 20;
    REG126_4 = 1;
    REG3_14_15 = 3;
    REG127 = 43101;
    chan_num = 2;
  }
  if (frequency < 175) {
    DIV = 24;
    REG126_4 = 1;
    REG3_14_15 = 4;
    REG127 = 64828;
    chan_num = 1;
  }
  unsigned int REG16 = 19328 / (121875 / IF);
  unsigned long N_IF = 9.75 * REG127;
  unsigned long N = DIV * frequency * 8388608 / F_REF - N_IF;
  unsigned int freq_l = N & 0xFFFF;
  unsigned int freq_h = N >> 16 & 0xFFFF;

  unsigned int  val = 0;
  if (DIR_TX == dir)
  {
    BK_RampDown_Enable();
  }
  if ((chan_num == 1) || (chan_num == 2))
  {
    val = BK_Read_Reg(126);
    val |= 0x0010;
    BK_Write_Reg(126, val);
    if (chan_num == 1)
      BK_Write_Reg(127, 0x4719);
    else
      BK_Write_Reg(127, 0x1095);

  }
  if ((chan_num == 3) || (chan_num == 4) || (chan_num == 5))
  {
    val = BK_Read_Reg(126);
    val &= 0xffef;
    BK_Write_Reg(126, val);
    if (chan_num == 3)
      BK_Write_Reg(127, 0xDA11);

    else if (chan_num == 4)
      BK_Write_Reg(127, 0xA38D);

    else
      BK_Write_Reg(127, 0x6D08);

  }


  BK_Write_Reg(113, freq_h);
  BK_Write_Reg(114, freq_l);

  BK_SPI_Trigger();

  g_reg0_15_buff[3] &= 0x1FFF;
  if (chan_num == 1)
  {
    g_reg0_15_buff[3] |= 0x8000;

  }
  else if (chan_num == 2)
  {
    g_reg0_15_buff[3] |= 0x6000;
  }
  else if (chan_num == 3)
  {
    g_reg0_15_buff[3] |= 0x4000;
  }
  else if (chan_num == 4)
  {
    g_reg0_15_buff[3] |= 0x2000;
  }
  else if (chan_num == 5)
  {
    g_reg0_15_buff[3] |= 0x0000;
  }

  BK_Write_Reg(3, g_reg0_15_buff[3]);

  delay(15);

  if (DIR_TX == dir)
  {
    BK_RampUp_Enable();
  }
}

/*************************************************
  Function:       BK_Reset
  Description:    reset
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Reset()
{
  unsigned int val = 0;
  val = BK_Read_Reg(112);

  val &= 0xDFFF; //clear bit 13 enter reset
  BK_Write_Reg(112, val);

  delay(40); /* delay 40ms */

  val |= 0x2000;
  BK_Write_Reg(112, val);
}

/*************************************************
  Function:       BK_Powerdown
  Description:    power down
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Powerdown()
{
  unsigned int val = 0;
  unsigned char tx_mode;

  val = BK_Read_Reg(112);
  tx_mode = (val & 0x4000) >> 8;

  if (tx_mode) //if tx mode ,ramp down
  {
    BK_RampDown_Enable();
  }

  //for BK4811_MP_8200
  g_reg0_15_buff[7] |= 0x0FFD;
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  delay(1);

  g_reg0_15_buff[7] &= 0xFFFE; // power down REG7[0]=SPICEN=0 at last
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  val &= 0x5FFF;//powerup=0;sys_reset=0
  BK_Write_Reg(112, val);

}

/*************************************************
  Function:       BK_Powerup
  Description:    power up
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void BK_Powerup()
{
  unsigned int val = 0;
  unsigned char tx_mode;

  val = BK_Read_Reg(112);
  val |= 0x8000;
  BK_Write_Reg(112, val);
  tx_mode = (val & 0x4000) >> 8;

  BK_Reset();//RESET digital state machine

  g_reg0_15_buff[7] |= 0x0001; // REG7<0>=1 High Power LDO at first
  BK_Write_Reg(7, g_reg0_15_buff[7]);

  delay(1);

  BK_Write_Reg(2, g_reg0_15_buff[2]);
  BK_Write_Reg(3, g_reg0_15_buff[3]);
  BK_Write_Reg(4, g_reg0_15_buff[4]);
  BK_Write_Reg(5, g_reg0_15_buff[5]);

  if (tx_mode) //tx mode
  {
    g_reg0_15_buff[7] |= 0x2f08; //reg7<3,8,9,10,11,13>=1
    g_reg0_15_buff[7] &= 0xFF1B; //reg7<2,5,6,7>=0,
    BK_Write_Reg(7, g_reg0_15_buff[7]);

    BK_SPI_Trigger();

    g_reg0_15_buff[7] &= 0xffef; //reg7<4>=0, Power up TXRF
    BK_Write_Reg(7, g_reg0_15_buff[7]);

  }
  else// RX Mode
  {
    g_reg0_15_buff[7] &= 0xd053; //reg7<2,3,5,7,8.9,10,11,13>=0
    g_reg0_15_buff[7] |= 0x50; //reg7<4,6>=1
    BK_Write_Reg(7, g_reg0_15_buff[7]);

    BK_SPI_Trigger();
  }

  if (tx_mode) //tx mode
  {
    delay(15);
    BK_RampUp_Enable();
  }

}

/*************************************************
  Function:       BK_Set_SNR_Threshold
  Description:    set SNR trheshold
  Input:          thrshld : 6bit value

  Output:         None
  Return:         None
*************************************************/
void BK_Set_SNR_Threshold(unsigned char thrshld)
{
  unsigned int val = 0;

  if (thrshld > MAX_SNR_THRESHOLD)
    thrshld = MAX_SNR_THRESHOLD;

  val  = BK_Read_Reg(67);
  val &= 0xC0FF;
  val |= ((thrshld & 0x3F) << 8);
  BK_Write_Reg(67, val);
}

/*************************************************
  Function:       BK_Set_RSSI_Threshold
  Description:    set RSSI trheshold
  Input:          thrshld : 7bit value

  Output:         None
  Return:         None
*************************************************/
void BK_Set_RSSI_Threshold(unsigned char thrshld)
{
  unsigned int val = 0;

  if (thrshld > MAX_RSSI_THRESHOLD)
    thrshld = MAX_RSSI_THRESHOLD;

  val  = BK_Read_Reg(67);
  val &= 0xFF80;
  val |= (thrshld & 0x7F);
  BK_Write_Reg(67, val);
}

/*************************************************
  Function:       BK_Set_Soft_Mute
  Description:    set soft mute
  Input:          index : 0 disable soft mute
                1 -6  db
                            2 -12 db
                            3 -18 db
                            4 -24 db
  Output:         None
  Return:         None
*************************************************/
void BK_Set_Soft_Mute(unsigned char index)
{
  unsigned int val = 0;

  if (index > MAX_SOFT_MUTE)
    index = MAX_SOFT_MUTE;

  if (0 == index)
  {
    BK_Disable_Soft_Mute();
    return;
  }

  if (!BK_Is_Soft_Mute())
    BK_Enable_Soft_Mute();

  index--;

  val  = BK_Read_Reg(73);
  val &= 0xC7FF;
  val |= (((unsigned int)index) << 11);
  BK_Write_Reg(73, val);
}


/*************************************************
  Function:       BK_Set_TX_Volume
  Description:    set tx volume
  Input:          volume  :  -25db,-24db,...,6db
                             1 per step

  Output:         None
  Return:         None
*************************************************/
void BK_Set_TX_Volume(unsigned char volume)
{
  unsigned int val = 0;

  if (volume > MAX_TX_VOLUME)
    volume = MAX_TX_VOLUME;

  val = BK_Read_Reg(18);
  val &= 0xFF83;
  val |= ((volume & 0x1F) << 2);
  BK_Write_Reg(18, val);
}

/*************************************************
  Function:       BK_Set_RX_Volume
  Description:    set rx volume
  Input:          volume : -39db,-36db,...,6db
                           3 per step

  Output:         None
  Return:         None
*************************************************/
void BK_Set_RX_Volume(unsigned char volume)
{
  unsigned int val = 0;

  if (volume > MAX_RX_VOLUME)
    volume = MAX_RX_VOLUME;

  val = BK_Read_Reg(73);
  val &= 0xFFF0;
  val |= (volume & 0xF);
  BK_Write_Reg(73, val);
}
/*************************************************
  Function:       BK_init
  Description:    Initalisation
  Input:          frequency
  Output:         None
  Return:         None
*************************************************/
void BK_Init(double frequency) {
  pinMode(sbit_spi_clk, OUTPUT);
  pinMode(sbit_spi_sdio, OUTPUT);
  pinMode(sbit_spi_sen, OUTPUT);
  digitalWrite(sbit_spi_sen, 0);
  RX_Parallel_Mode = 0x10; //default: audio on
  b_3_CTCSS_send = 0;

  //EA = 1;

  delay(100);
  BK_Analog_Init();

  BK_Set_Freq(DIR_RX, frequency);
  delay(50);
  BK_Switch_RX_Type(SPEECH);

  DisableGlobalIntr();

  disable_fsk_test();
}
/*************************************************
  Function:       BK_Disable_Inverse_Frequency
  Description:    disable inverse frequence
  Input:          dir  1 : RX
                       2 : TX
  Output:         None
  Return:         None
*************************************************/
static void BK_Disable_Inverse_Frequency(unsigned char dir)
{
  unsigned int val = 0;

  if (DIR_TX == dir) /* RX */
  {
    val = BK_Read_Reg(18);
    val &= 0xBFFF;
    val |= 0x4000;  /* Bypass */
    BK_Write_Reg(18, val);
  }
  else if (DIR_RX == dir) /* TX */
  {
    val = BK_Read_Reg(72);
    val &= 0xDFFF;
    val |= 0x2000;  /* Bypass */
    BK_Write_Reg(72, val);
  }
}

/*************************************************
  Function:       BK_Enable_Inverse_Frequency
  Description:    enable inverse frequence
  Input:          dir  1 : RX
                       2 : TX
  Output:         None
  Return:         None
*************************************************/
static void BK_Enable_Inverse_Frequency(unsigned char dir)
{
  unsigned int val = 0;

  if (DIR_TX == dir) /* TX */
  {
    val = BK_Read_Reg(18);
    val &= 0xBFFF;      /* Not Bypass */
    BK_Write_Reg(18, val);
  }
  else if (DIR_RX == dir) /* RX */
  {
    val = BK_Read_Reg(72);
    val &= 0xDFFF;      /* Not Bypass */
    BK_Write_Reg(72, val);
  }
}

/*************************************************
  Function:       BK_Disable_Encryption
  Description:    disable encryption
  Input:          None
  Output:         None
  Return:         None
*************************************************/
static void BK_Disable_Encryption()
{
  unsigned int val = 0;

  val = BK_Read_Reg(120);
  val &= 0x7FFF;
  BK_Write_Reg(120, val);
}

/*************************************************
  Function:       BK_Enable_Encryption
  Description:    enable encryption
  Input:          None
  Output:         None
  Return:         None
*************************************************/
static void BK_Enable_Encryption()
{
  unsigned int val = 0;

  val = BK_Read_Reg(120);
  val |= 0x8000;
  BK_Write_Reg(120, val);
}

/*************************************************
  Function:       BK_Disable_Scrambling
  Description:    disable encryption & inverse frequence
  Input:          dir  1 : RX
                       2 : TX
  Output:         None
  Return:         None
*************************************************/
void BK_Disable_Scrambling(unsigned char dir)
{
  unsigned int val = 0;

  BK_Disable_Inverse_Frequency(dir);
  BK_Disable_Encryption();
}

/*************************************************
  Function:       BK_Set_Inverse_Frequency
  Description:    inverse frequence setting
  Input:          dir  1 : RX
                       2 : TX
                  freq   : frequence 0-7
  Output:         None
  Return:         None
*************************************************/
void BK_Set_Inverse_Frequency(unsigned char dir, unsigned char freq)
{
  unsigned int val = 0;

  if (freq > MAX_INVERSE_FREQ)
    freq = MAX_INVERSE_FREQ;

  /* disable encryption since they are mutx */
  BK_Disable_Encryption();
  BK_Enable_Inverse_Frequency(dir);

  if (DIR_TX == dir) /* TX */
  {
    val = BK_Read_Reg(20);
    val &= 0x1FFF;
    val |= ((freq & 0x7) << 13);
    BK_Write_Reg(20, val);
  }
  else if (DIR_RX == dir) /* RX */
  {
    val = BK_Read_Reg(74);
    val &= 0x1FFF;
    val |= ((freq & 0x7) << 13);
    BK_Write_Reg(74, val);
  }
}

/*************************************************
  Function:       BK_Set_Encryption
  Description:    Encryption setting
  Input:          dir  1 : RX
                       2 : TX
                  TM: 0-63
                  TR: 0-63

  Output:         None
  Return:         None
*************************************************/
void BK_Set_Encryption(unsigned char dir, unsigned char TM, unsigned char TR)
{
  unsigned int val = 0;

  if (TM > MAX_ENCRYPTION)
    TM = MAX_ENCRYPTION;

  if (TR > MAX_ENCRYPTION)
    TR = MAX_ENCRYPTION;

  BK_Disable_Inverse_Frequency(dir);

  BK_Disable_Encryption();

  val = BK_Read_Reg(120);

  val &= 0xF000;
  val |= (((unsigned int)TM) << 6);
  val |= TR;

  BK_Write_Reg(120, val);


  BK_Enable_Encryption();
}

/*************************************************
  Function:       BK_Read_RSSI
  Description:    read RSSI value
  Input:          None

  Output:         None
  Return:         RSSI_val  RSSI value
*************************************************/
unsigned char BK_Read_RSSI()
{
  unsigned int val = 0;
  unsigned char RSSI_val = 0;

  val = BK_Read_Reg(68);

  RSSI_val = (val & 0x007F); /* RSSI 7bit */

  return RSSI_val;
}

/*************************************************
  Function:       BK_Read_Signal_Valid
  Description:    read Signal Valid
  Input:          None

  Output:         None
  Return:         Signal Valid
*************************************************/
unsigned char BK_Read_Signal_Valid()
{
  unsigned int val = 0;
  unsigned char signal_val = 0;

  val = BK_Read_Reg(68);

  signal_val = (val >> 14) & 0x01; /* 1 bit */

  return signal_val;
}

/*************************************************
  Function:       BK_Read_SNR
  Description:    read SNR value
  Input:          None

  Output:         None
  Return:         SNR_val       SNR value
*************************************************/
unsigned char BK_Read_SNR()
{
  unsigned int val = 0;
  unsigned char SNR_val = 0;

  val = BK_Read_Reg(68);

  SNR_val = ((val & 0x3F00) >> 8); /* SNR 7bit */

  return SNR_val;
}

/*************************************************
  Function:       BK_Read_Audio_RSSI
  Description:    Audio RSSI read
  Input:          None

  Output:         None
  Return:         Audio RSSI value
*************************************************/
unsigned char BK_Read_Audio_RSSI()
{
  unsigned int val = 0;
  unsigned char Audio_RSSI_val = 0;

  val = BK_Read_Reg(74);

  Audio_RSSI_val = ((val & 0xFF)); /* Audio RSSI 8bit */

  return Audio_RSSI_val;

}

/*************************************************
  Function:       BK_Read_AFC_Indicator
  Description:    Read AFC Indicator
  Input:          None

  Output:         None
  Return:         AFC Indicator value
*************************************************/
unsigned int BK_Read_AFC_Indicator()
{
  unsigned int val = 0;
  unsigned int AFC_Indicator_val = 0;

  val = BK_Read_Reg(70);

  AFC_Indicator_val = ((val & 0x3fFF)); /* AFC Indicator 14bit */

  return AFC_Indicator_val;

}


/*************************************************
  Function:       Send_3_CTCSS
  Description:    send 3 CTCSS tone
  Input:          a array contain 3 CTCSS index (value:1-7)

  Output:         None
  Return:         NOne
*************************************************/
void Send_3_CTCSS(unsigned char* buf)
{
  unsigned char i;
  for (i = 0; i < 3; i++)
  {
    BK_CTCSS_TX(buf[i]);
    delay(150); /* delay 250ms */
  }
}



/*************************************************
  Function:       Set_FSK_Air_mode
  Description:    set FSK air mode
  Input:          b_enable;
                     1:enable 1.2K;
                     2:enabel 2.4K
                     0:disable

  Output:         None
  Return:         None
*************************************************/
static void Set_FSK_Air_mode(unsigned char b_enable)
{
  unsigned int val;

  val = BK_Read_Reg(1);

  if (b_enable == 2)
  {
    val = val | 0x4000;
  }
  else
  {
    val = val & 0xBFFF;
  }
  BK_Write_Reg(1, val);


  val = BK_Read_Reg(95);

  if (b_enable == 2) //reg95 <11:0>=237h===>013b
  {
    val &= 0xf000;
    val |= 0x42f;
  }
  else//reg95 <11:0>=042f
  {
    val &= 0xf000;
    val |= 0x42f;
    BK_Write_Reg(95, val);

    val &= 0xf000;
    val |= 0x13b;

  }

  BK_Write_Reg(95, val);

}

/*************************************************
  Function:       Enable_TX_FSK_Air
  Description:    FSK TX air mode enable or disable
  Input:          b_enable;
                     1:enable 1.2K;
                     2:enabel 2.4K
                     0:disable

  Output:         None
  Return:         None
*************************************************/
void Enable_TX_FSK_Air(unsigned char b_enable)
{
  unsigned int val;

  Set_FSK_Air_mode(b_enable);

  val = BK_Read_Reg(40);

  if (b_enable)
  {
    val = val | 0x0100;
  }
  else
  {
    val = val & 0xfeff;
  }

  BK_Write_Reg(40, val);

}


/*************************************************
  Function:       Enable_RX_FSK_Air
  Description:    FSK RX air mode enable or disable
  Input:          b_enable;
                     1:enable 1.2K;
                     2:enabel 2.4K
                     0:disable

  Output:         None
  Return:         None
*************************************************/
void Enable_RX_FSK_Air(unsigned char b_enable)
{
  unsigned int val;
  unsigned int tmpval;

  Set_FSK_Air_mode(b_enable);

  val = BK_Read_Reg(66);

  if (b_enable)
  {
    //////////////////////////////////////////////////////
    tmpval = val & 0x0060;
    val = val & 0xFAFB; //clear all fsk air bit
    BK_Write_Reg(66, val);

    if (tmpval == 0x40) //if FSK is set already
    {
      val = val & 0xff9f; //clear bit 6:5
      val |= 0x20; //set DTMF mode(only a different type),in order to close FSK mode all.
      BK_Write_Reg(66, val);

      val = val & 0xff9f; //clear bit 6:5.
      val |= 0x40; //set FSK mode again.
    }
    //////////////////////////////////////////////////////

    val = val | 0x0004; //set bit 2

  }
  else
  {
    val = val & 0xfffb; //clear bit 2
  }

  BK_Write_Reg(66, val);

}

/*************************************************
  Function:       Enable_RX_Parallel_Mode
  Description:    Enable Parallel Mode when RX mode
  Input:          p_mode:
               bit 4:audio enable/disable;
               bit 3:DTMF enable/disable;
               bit 2:FSK enable/disable;
               bit 1:SELCALL enable/disable;
               bit 0:FSK Air enable/disable;

  Output:         None
  Return:         None
*************************************************/
void Enable_RX_Parallel_Mode(unsigned char p_mode)
{
  unsigned int val, tmp16Value;
  unsigned int vtmp;

  val = BK_Read_Reg(66);
  val = val & 0xe09f; //clear bit 12-8

  tmp16Value = p_mode;

  val = val | (tmp16Value << 8); //set bit 12-8 value

  if (p_mode & 0x10) //audio
  {
    vtmp = 0;
  }
  else if (p_mode & 0x08) //dtmf
  {
    vtmp = 1;
  }
  else if (p_mode & 0x04) //fsk
  {
    vtmp = 2;
  }
  else if (p_mode & 0x01) //fsk air
  {
    vtmp = 2;
  }
  else if (p_mode & 0x02) //selcall
  {
    vtmp = 3;
  }

  if (p_mode & 0x1f) //enable bit7
  {
    val = val | 0x80;
  }
  else
  {
    val = val & 0xff7f;
  }

  vtmp = vtmp << 5;

  val = val | vtmp;


  //      val  = BK_Read_Reg(66);
  //      val &= 0xFFE7;
  //      val |= 0x0018;
  //      BK_Write_Reg(66, val);  //select cdcss



  BK_Write_Reg(66, val);

  tmp16Value = INTM_ENABLE_INTR;

  if (p_mode & 0x08)
    tmp16Value |= INTM_DTMF_RX;

  if (p_mode & 0x02)
    tmp16Value |= INTM_SELCALL_RX;

  if (p_mode & 0x05)
    tmp16Value |= ( INTM_FSK_HEAD_RX |
                    INTM_FSK_FIFO_NEEDREAD |
                    INTM_FSK_RX_COMPLETE);


  tmp16Value |= INTM_AFC_LINKLOST;

  EnableGlobalIntr(tmp16Value);


}

/*************************************************
  Function:       Enter_TX_Audio_LoopBack_Mode
  Description:    enable TX -->Class AB or TX-->class D
  Input:          b_to_class_ab:
                                1:enable TX-->Class AB
                                0:enable TX-->Class D
  Output:         None
  Return:         None
*************************************************/
void Enter_TX_Audio_LoopBack_Mode()
{
  unsigned int value;
  //set reg117<12>
  value = BK_Read_Reg(117);
  value = value | 0x1000; //set reg117<12>
  BK_Write_Reg(117, value);

  value = BK_Read_Reg(66);
  value = value | 0x1000; //set reg66<12>
  BK_Write_Reg(66, value);


  value = g_reg0_15_buff[7];
  value = value & 0xfbff; //clear reg7<10>,not save reg7 to g_reg0_15_buff
  BK_Write_Reg(7, value);



}
/*************************************************
  Function:       Exit_TX_Audio_LoopBack_Mode
  Description:    diable  TX -->Class AB or TX-->class D
  Input:          None

  Output:         None
  Return:         None
*************************************************/
void Exit_TX_Audio_LoopBack_Mode()
{
  unsigned int value;

  //clear reg117<12>
  value = BK_Read_Reg(117);
  value = value & 0xefff; //clear reg117<12>
  BK_Write_Reg(117, value);

  BK_Write_Reg(7, g_reg0_15_buff[7]);
}



/* for internal test use */




void BK_FSK_Init(FSK_CFG *cfg)
{
  unsigned int val = 0;
  unsigned char len = cfg->fsk_len;
  FSK_TYPE fsk_type = FSK_TYPE(cfg->fsk_type);
  //BK_FSK_TX_Init(cfg->fsk_type, pOutPayload, len);
  BK_FSK_TX_Init(fsk_type, pOutPayload, len);
  if (DIR_TX == cfg->fsk_dir)
  {
    val = cfg->fsk_addr;         /* Address */
    val <<= 8;
    val = val | cfg->fsk_type; //fsk type
    BK_Write_Reg(28, val);

    val  = BK_Read_Reg(31);
    val &= 0x1FFF;
    val |= (cfg->fsk_thrshld << 13);       /* Threshold */
    BK_Write_Reg(31, val);

  }
  else if (DIR_RX == cfg->fsk_dir)
  {
    val = cfg->fsk_addr;         /* Address */
    val <<= 8;
    val |= cfg->fsk_type;//fsk type
    BK_Write_Reg(79, val);

    val  = BK_Read_Reg(83);
    val &= 0x1FFF;
    val |= (cfg->fsk_thrshld << 13);       /* Threshold */
    BK_Write_Reg(83, val);

  }

  BK_Write_Reg(27, cfg->fsk_syncword);//syncword

  if (cfg->fsk_is_scrmb)                       /* Scramble */
    BK_Write_Reg(32, 0x8000 | (cfg->fsk_scrmb << 8));




}


void BK_FSK_Test()
{
  //  static unsigned char fsk_rx_id = 0;
  unsigned char infinite = 0;

  //  fsk_times = fsk_test_cfg.fsk_times;

  if (0 == fsk_test_cfg.fsk_times)
    infinite = 1;


  if (DIR_TX == fsk_test_cfg.fsk_dir)
  {
    OutUsbDataHit = 1;

    if (1 == fsk_test)
    {
      fsk_test = 2;

      pOutPayload[0] += 1;

      if (infinite == 1)
      {
        if (pOutPayload[0] > 99) //100
          pOutPayload[0] = 0;
      }


      pInPayload[0] = 1;
      pInPayload[1] = pOutPayload[0];
      USBPrint32B(BKN_DEMO_FSK_TEST_PRINT, 2);


      delay(fsk_test_cfg.fsk_delay);

      BK_Switch_TX_Type(FSK);
      FSK_TYPE fsk_type = FSK_TYPE(fsk_test_cfg.fsk_type);
      BK_FSK_TX_Init(fsk_type, pOutPayload, fsk_test_cfg.fsk_len);

      BK_FSK_TX(pOutPayload, fsk_test_cfg.fsk_len);

      if (0 == infinite)
      {
        fsk_times--;
        if (0 == fsk_times)
        {
          OutUsbDataHit = 0;
          fsk_test = 0;
        }
      }
    }

    //      if(BKN_DEMO_FSK_TEST_STOP == cur_test_item)
    //      {
    //          OutUsbDataHit = 0;
    //          fsk_test = 0;
    //      }
  }
  else if (DIR_RX == fsk_test_cfg.fsk_dir)
  {
    BK_FSK_RX();
    BK_Clear_Intr(0xFFFF);
    EX0 = 1;
  }
}


/*************************************************
  Function:       Send_CTCSS_Tag
  Description:    transmit 3 CTCSS(2 tag CTCSS+main CTCSS)
  Input:          tag :from 0-41
                  send_main_CTCSS:1-7
                  Send_CTCSS_times:send times

  Output:         None
  Return:         None
*************************************************/
void Send_CTCSS_Tag(unsigned char tag1, unsigned char tag2, unsigned char send_main_CTCSS, unsigned char Send_CTCSS_times)
{
  unsigned char buff[3];
  unsigned char i = 0;

  buff[0] = tag1; //(1--7)
  buff[1] = tag2; //(1--7)
  buff[2] = send_main_CTCSS; //(1--7)

  for (i = 0; i < Send_CTCSS_times; i++)
  {
    Send_3_CTCSS(buff);


    pInPayload[0] = 0;
    pInPayload[1] = i + 1;
    USBPrint32B(BKN_DEMO_SEND_CTCSS_END, 6);

    delay(500); /* delay 800ms */
  }

  pInPayload[0] = 1;
  USBPrint32B(BKN_DEMO_SEND_CTCSS_END, 6);


}

void Send_3_CTCSS_Test()
{
  Send_CTCSS_Tag(2, 7, 2, 3);
}


/*************************************************
  Function:       Print_Recv_CTCSS
  Description:    receive a tag with 3 CTCSS
  Input:          none

  Output:         None
  Return:         reveived tag
*************************************************/
static unsigned char Print_Recv_CTCSS()
{


  pInPayload[0] = 1;
  pInPayload[1] = (unsigned char)received_CTCSS_array[0];
  pInPayload[2] = (unsigned char)received_CTCSS_array[1];
  pInPayload[3] = (unsigned char)received_CTCSS_array[2];
  USBPrint32B(BKN_DEMO_READ_CTCSS, 5);
  delay(100);


  return 0;

}

/*************************************************
  Function:       Squelch
  Description:    Close RX audio path when the signal quality is bad.
          Signal quality indicator: RSSI REG68[6:0]; SNR REG68[13:8]; Ex-noise REG102[12:0]; Glitch REG125[7:0]
  Input:          none

  Output:         None
  Return:         None
*************************************************/
void BK_RX_SQ()
{
  unsigned char i = 0;
  unsigned char SQCounter = 3;
  unsigned int val = 0;
  unsigned char valRSSI = 0;
  unsigned char valSNR = 0;
  unsigned int valExNoise = 0;
  unsigned char valGlitch = 0;

  for (i = 0; i < 3; i++)
  {
    valRSSI = BK_Read_Reg(68) & 0x007F;
    valSNR = ((BK_Read_Reg(68) & 0x3F00) >> 8);
    valExNoise = BK_Read_Reg(102) & 0x1FFF;
    valGlitch = BK_Read_Reg(125) & 0x00FF;

    if (valRSSI < 0x001D)
    {
      SQCounter++;
    }
    else if ( (valSNR < 0x19) & (valExNoise > 0x01F4) & (valGlitch > 2) )
    {
      SQCounter++;
    }
    else
    {
      SQCounter--;
    }

    delay(15);
  }

  if (SQCounter > 5)
  {
    g_reg0_15_buff[7] |= 0x0400;
    BK_Write_Reg(7, g_reg0_15_buff[7]);
  }

  if (SQCounter < 1)
  {
    g_reg0_15_buff[7] &= 0xFBFF;
    BK_Write_Reg(7, g_reg0_15_buff[7]);
  }

}

/*************************************************
  Function:       command process
  Description:    Process commands from serial port
  Input:          none

  Output:         None
  Return:         None
*************************************************/
void command_handle(void)
{
  unsigned char  timeout = 0;
  unsigned char  vox_threshold = 0;
  unsigned char  mic_threshold = 0;
  unsigned char  volume = 0;
  unsigned char  output = 0;
  unsigned char  dir = 0;
  unsigned char  freq = 0;
  double frequency;
  unsigned char  len = 0;
  unsigned char  type = 0;
  FSK_TYPE fsk_type;
  //unsigned char message_type=OUT_BUFFER.Ptr->MessageType;
  unsigned char incomingByte;
  unsigned char HexNibble = 0xFF;
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    // detect end of line cr
    if (incomingByte == 13) {
      switch (message_type)
      {
        /* Speech TX */
        case BKN_DEMO_SPEECH_TX:    //0x80:
          //BK_Speech_TX();
          BK_Switch_TX_Type(SPEECH);
          break;

        /* Speech RX */
        case BKN_DEMO_SPEECH_RX:    //0x81:
          //BK_Speech_RX();

          if (pOutPayload[0])
          {
            BK_Switch_RX_Type(SPEECH);
            RX_Parallel_Mode |= 0x10;
          }
          else
            RX_Parallel_Mode &= 0xef;
          Enable_RX_Parallel_Mode(RX_Parallel_Mode);

          break;

        /* DTMF TX */
        case BKN_DEMO_DTMF_TX:      //0x82:
        case BKN_DEMO_DTMF_TX_ONLY:

          len = OUT_BUFFER.len;


          if (cur_test_item != BKN_DEMO_DTMF_TX)
            BK_Switch_TX_Type(DTMF);
          //BK_Switch_DTMF_TX();

          if (0 == len)
            break;

          if (BKN_DEMO_DTMF_TX_ONLY == message_type)
            break;

          BK_DTMF_TX(pOutPayload, len);
          break;

        /* DTMF RX */
        case BKN_DEMO_DTMF_RX:      //0x83:
          if (pOutPayload[0])
          {
            BK_DTMF_RX();
            RX_Parallel_Mode |= 0x08;;
          }
          else
            RX_Parallel_Mode &= 0xf7;
          Enable_RX_Parallel_Mode(RX_Parallel_Mode);
          break;

        /*  SELCALL TX */
        case BKN_DEMO_SELCALL_TX:   //0x84:
        case BKN_DEMO_SELCALL_TX_ONLY:

          len = OUT_BUFFER.len;

          if (cur_test_item != BKN_DEMO_SELCALL_TX)
            //BK_Switch_SCLCALL_TX();
            BK_Switch_TX_Type(SELCALL);

          if (0 == len)
            break;

          if (BKN_DEMO_SELCALL_TX_ONLY == message_type)
            break;

          BK_SELCALL_TX(pOutPayload, len);
          break;

        /* SELCALL RX */
        case BKN_DEMO_SELCALL_RX:   //0x85:

          if (pOutPayload[0])
          {
            BK_SELCALL_RX();
            RX_Parallel_Mode |= 0x02;
          }
          else
            RX_Parallel_Mode &= 0xfd;

          Enable_RX_Parallel_Mode(RX_Parallel_Mode);

          break;

        case BKN_DEMO_FSK_TX:
        case BKN_DEMO_FSK_TX_ONLY:
          if (fsk_single_send_busy)
            return;



          if (cur_test_item != BKN_DEMO_FSK_TX)
          {
            //BK_Switch_FSK_TX();
            BK_Switch_TX_Type(FSK);
          }

          len = OUT_BUFFER.len;

          if (0 == len)
            break;

          if (BKN_DEMO_FSK_TX_ONLY == message_type)
            break;

          fsk_single_send_busy = 1;
          fsk_type = FSK_TYPE(fsk_test_cfg.fsk_type);
          BK_FSK_TX_Init(fsk_type, pOutPayload, len);

          disable_fsk_test();
          BK_FSK_TX(pOutPayload, len);
          break;

        case BKN_DEMO_FSK_RX:

          if (pOutPayload[0])
          {
            disable_fsk_test();
            BK_FSK_RX();

            if (fsk_test_cfg.fsk_air)
            {
              RX_Parallel_Mode |= 0x01;
              RX_Parallel_Mode &= 0xfb;
            }
            else
            {
              RX_Parallel_Mode |= 0x04;
              RX_Parallel_Mode &= 0xfe;
            }

          }
          else
            RX_Parallel_Mode &= 0xfa;
          Enable_RX_Parallel_Mode(RX_Parallel_Mode);

          break;

        case BKN_DEMO_CTCSS_TX:
          len = OUT_BUFFER.len;
          if (0 == len)
            break;

          freq = pOutPayload[0];

          if (cur_test_item != BKN_DEMO_CTCSS_TX)
          {
            BK_Switch_TX_Type(CTCSS);
          }

          BK_CTCSS_TX(freq);
          //          Send_3_CTCSS_Test();
          break;

        case BKN_DEMO_CTCSS_3_TX:
          if (cur_test_item != BKN_DEMO_CTCSS_TX)
          {
            BK_Switch_TX_Type(CTCSS);
          }

          Send_CTCSS_Tag(pOutPayload[0], pOutPayload[1], pOutPayload[2], pOutPayload[3]);
          break;

        case BKN_DEMO_CTCSS_RX:
          len = OUT_BUFFER.len;
          if (0 == len)
            break;

          freq = pOutPayload[0];

          if (cur_test_item != BKN_DEMO_CTCSS_RX)
          {
            BK_Switch_RX_Type(CTCSS);
          }

          b_3_CTCSS_send = 0;

          BK_CTCSS_RX(freq);
          break;

        case BKN_DEMO_CDCSS_TX:
          len = OUT_BUFFER.len;
          if (0 == len)
            break;

          freq = pOutPayload[0];

          if (cur_test_item != BKN_DEMO_CDCSS_TX)
          {
            BK_Switch_TX_Type(CDCSS);
          }

          BK_CDCSS_TX(freq);
          break;

        case BKN_DEMO_CDCSS_RX:
          len = OUT_BUFFER.len;
          if (0 == len)
            break;

          freq = pOutPayload[0];

          if (cur_test_item != BKN_DEMO_CDCSS_RX)
          {
            BK_Switch_RX_Type(CDCSS);
          }

          BK_CDCSS_RX(freq);
          break;

        case BKN_DEMO_SET_TOT:
          timeout = pOutPayload[0];
          BK_Set_TOT(timeout);
          break;

        case BKN_DEMO_SET_VOX:
          vox_threshold = pOutPayload[0];
          BK_Set_VOX(vox_threshold);
          break;

        case BKN_DEMO_SET_MIC_THRSHLD:
          mic_threshold = pOutPayload[0];
          BK_Set_MIC_Threshold(mic_threshold);
          break;

        case BKN_DEMO_SET_TX_FREQ:
          len = OUT_BUFFER.len;
          if (0 == len)
            break;

          frequency = pOutPayload[0];
          BK_Set_Freq(DIR_TX, frequency);
          break;

        case BKN_DEMO_SET_RX_FREQ:
          len = OUT_BUFFER.len;
          if (0 == len)
            break;

          frequency = pOutPayload[0];
          BK_Set_Freq(DIR_RX,  frequency);
          break;

        //case BKN_DEMO_SET_CHANNEL_NUM:
        //    BK_Set_Channel_Num(pOutPayload[0],pOutPayload[1]);
        //    break;

        case BKN_DEMO_RESET:
          BK_Reset();
          break;

        case BKN_DEMO_POWERDOWN:
          BK_Powerdown();
          break;

        case BKN_DEMO_POWERUP:
          BK_Powerup();
          break;

        case BKN_DEMO_SET_SNR_THRSHLD:
          BK_Set_SNR_Threshold(pOutPayload[0]);
          break;

        case BKN_DEMO_SET_RSSI_THRSHLD:
          BK_Set_RSSI_Threshold(pOutPayload[0]);
          break;

        case BKN_DEMO_SET_MUTE:
          BK_Set_Soft_Mute(pOutPayload[0]);
          break;

        case BKN_DEMO_SET_TX_VOUME:
          volume = pOutPayload[0];
          BK_Set_TX_Volume(volume);
          break;

        case BKN_DEMO_SET_RX_VOUME:
          volume = pOutPayload[0];
          BK_Set_RX_Volume(volume);
          break;


        case BKN_DEMO_DISABLE_SCRAMBLING:
          dir = pOutPayload[0];
          BK_Disable_Scrambling(dir);
          break;

        case BKN_DEMO_SET_INVERSE_FREQUENCY:
          //freq = *((unsigned char *)(&(OUT_BUFFER.Ptr->pPayload)));
          //dir = *((unsigned char *)(&(OUT_BUFFER.Ptr->pPayload)) + 1);
          freq = pOutPayload[0];
          dir =  pOutPayload[1];
          BK_Set_Inverse_Frequency(dir, freq);
          break;

        case BKN_DEMO_SET_ENCRYPTION:
          dir = pOutPayload[2];
          BK_Set_Encryption(dir, pOutPayload[0], pOutPayload[3]);
          break;

        case BKN_DEMO_QUERY_SIGNAL:
          pInPayload[0] = BK_Read_RSSI(); /* RSSI 7bit */
          pInPayload[1] = BK_Read_SNR(); /* SNR 7bit */
          pInPayload[2] = BK_Read_Audio_RSSI(); /* Audio RSSi 8bit */

          encryption = BK_Read_AFC_Indicator(); //AFC indicator
          pInPayload[3] = (encryption >> 8) & 0xff;
          pInPayload[4] = (encryption >> 0) & 0xff;

          pInPayload[5] = BK_Read_Signal_Valid();

          USBPrint32B(BKN_DEMO_READ_SIGNAL, 6);
          break;

        case BKN_DEMO_FSK_TEST_CONFIG:
          fsk_test_cfg.fsk_syncword   = (pOutPayload[1] << 8) | pOutPayload[0];
          fsk_test_cfg.fsk_scrmb      = pOutPayload[2];
          fsk_test_cfg.fsk_addr       = pOutPayload[3];
          fsk_test_cfg.fsk_type       = pOutPayload[4];
          fsk_test_cfg.fsk_thrshld    = pOutPayload[5];
          fsk_test_cfg.fsk_len        = pOutPayload[6];
          fsk_test_cfg.fsk_delay      = pOutPayload[7];
          fsk_test_cfg.fsk_times      = pOutPayload[8];
          fsk_test_cfg.fsk_is_scrmb   = pOutPayload[10];
          fsk_test_cfg.fsk_dir        = pOutPayload[11];
          //          fsk_test_cfg.fsk_space      = pOutPayload[12];
          fsk_test_cfg.fsk_air      = pOutPayload[13];




          BK_FSK_Init(&fsk_test_cfg);
          EnableAirMode(fsk_test_cfg.fsk_dir, fsk_test_cfg.fsk_air);

          break;

        case BKN_DEMO_FSK_CONFIG:
          fsk_test_cfg.fsk_addr       = pOutPayload[0];
          fsk_test_cfg.fsk_syncword   = (pOutPayload[1] << 8) | pOutPayload[2];
          fsk_test_cfg.fsk_type       = pOutPayload[3];
          fsk_test_cfg.fsk_air        = pOutPayload[4];
          fsk_test_cfg.fsk_dir        = pOutPayload[5];
          fsk_test_cfg.fsk_thrshld = 4;
          fsk_test_cfg.fsk_is_scrmb = 0;


          BK_FSK_Init(&fsk_test_cfg);
          EnableAirMode(fsk_test_cfg.fsk_dir, fsk_test_cfg.fsk_air);


          break;


        case BKN_DEMO_FSK_TEST_START:
          fsk_contiuous_busy = 1;
          if (0 == fsk_test)
          {
            EX0 = 0;
            fsk_rx_count = 0;
            fsk_test = 1;


            fsk_times = fsk_test_cfg.fsk_times;

            for (len = 0; len < 255; len++)
            {
              pOutPayload[0] = len + 1;
            }

          }

          BK_FSK_Test();
          break;


        case BKN_DEMO_FSK_TEST_STOP:

          if (pOutPayload[0] == DIR_RX)
          {
            pInPayload[0] = 0;
            pInPayload[1] = fsk_rx_count;
            USBPrint32B(BKN_DEMO_FSK_TEST_PRINT, 2);
          }

          fsk_contiuous_busy = 0;
          OutUsbDataHit = 0;
          fsk_rx_count = 0;
          fsk_test = 0;
          break;




        case BK_CMD_WRITE_REG:
          EX0 = 0;
          encryption = (unsigned int)(pOutPayload[1]);


          encryption = encryption << 8;
          encryption |= pOutPayload[2];

          if ((pOutPayload[0] >= 2) && (pOutPayload[0] <= 15))
          {
            g_reg0_15_buff[pOutPayload[0]] = encryption;
          }

          BK_Write_Reg(pOutPayload[0], encryption);
          break;

        case BK_CMD_READ_REG:
          EX0 = 0;

          if ((pOutPayload[0] >= 2) && (pOutPayload[0] <= 15))
            encryption = g_reg0_15_buff[pOutPayload[0]];
          else
            encryption = BK_Read_Reg(pOutPayload[0]);

          pInPayload[0] = (encryption & 0xFF00) >> 8;
          pInPayload[1] = encryption & 0xFF;
          USBPrint32B(BK_CMD_READ_REG, 2);
          //USBPrintWord(BK_CMD_READ_REG, encryption);
          break;

        case BK_CMD_DISABLE_EX0:
          EX0 = 0;
          break;

        case BKN_DEMO_ENABLE_INTR:
          EX0 = 1;
          break;

        case BKN_DEMO_ENABLE_AGC:
          Enable_AGC(pOutPayload[0]);
          break;

        case BKN_DEMO_FSK_AIR_MODE:
          if (pOutPayload[0] == DIR_RX) //RX
          {
            if (pOutPayload[1]) //air mode
            {
              RX_Parallel_Mode |= 0x01;
              RX_Parallel_Mode &= 0xfb;
            }
            else//not air mode
            {
              RX_Parallel_Mode |= 0x04;
              RX_Parallel_Mode &= 0xfe;
            }

            Enable_RX_FSK_Air(pOutPayload[1]);

            Enable_RX_Parallel_Mode(RX_Parallel_Mode);
          }
          else//TX
          {
            Enable_TX_FSK_Air(pOutPayload[1]);
          }

          break;

        case BKN_DEMO_AUDIO_LOOP_MODE:
          if (pOutPayload[0] == DIR_RX)
          {

          }
          else
          {
            if (pOutPayload[2]) //start
              Enter_TX_Audio_LoopBack_Mode();
            else
              Exit_TX_Audio_LoopBack_Mode();

          }

          break;

        //      case BK_RAMP_UP_DOWN2:
        //          if(pOutPayload[0])
        //          {
        //      //      Start_Ramp();
        //          }
        //          break;


        default :
          cur_test_item = STEP_IDLE;
          break;
      }//switch
      message_type = 0;
      pOutPayloadpos = 0;
    }
     // convert ASCII HEX to message type and pOutPayload array
     if (incomingByte > 47 && incomingByte < 58) {
      HexNibble = incomingByte - 48; //ASCII Decimal
    }
    if (incomingByte > 64 && incomingByte < 71) {
      HexNibble = incomingByte - 55; // Lowercase a-f
    }
    if (incomingByte > 96 && incomingByte < 103) {
      HexNibble = incomingByte - 87; //Uppercase A-F
    }


    //if (incomingByte == 32) {
      //space
      //Serial.print(outgoingByte, HEX);
    //}
    if (HexNibble != 0xFF) {
      if (outgoingnibble == 1) {
        outgoingByte = outgoingByte * 16 + HexNibble;
        if (message_type == 0) {
          message_type = outgoingByte;
        } else {
          pOutPayload[pOutPayloadpos] = outgoingByte;
          pOutPayloadpos++;
        }
        outgoingByte = 0;
        outgoingnibble = !outgoingnibble;
      } else {
        outgoingByte = HexNibble;
        outgoingnibble = !outgoingnibble;
      }
    }
  }
  if (BKN_DEMO_AUDIO_LOOP_MODE != message_type)
    cur_test_item = message_type;
}


