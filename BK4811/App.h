/*This file is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;  
 
 website: http://www.bekencorp.com 
---------------------------------------------------------------------------------------*/ 
#define BKN_APP_H_ 
 
#define BK_demo_C8051 // for demoboard use 
 
#define CODE code 
 
#define BYTE unsigned char 
#define INT8 char
#define UINT8 unsigned char 
#define UINT16 unsigned int 
#define UINT32 unsigned long 
 
#define F_REF	(21.7)
#define IF 137000 
#define POW2_23 (1<<23) 

 
#define sbit_spi_clk 13 
#define sbit_spi_sdio 12 
#define sbit_spi_sen 10 

 
 
 
#define STEP_IDLE       0x00 
 
#define DIR_RX 		0x01 
#define DIR_TX 		0x02 
 
#define EARPHONE 	0x01 
#define SPEAKER	 	0x02 
 
#define DISABLE_VOX 0x00 
#define DISABLE_TOT 0x00 
 
#define CHN_GB 		0x00 
#define FRS 		0x01 
#define GMRS 		0x02 
#define PMR 		0x03 
 
/* Chinese GB frequence */ 
#define CHNGB_FREQ_H16	0x4b83 
#define CHNGB_FREQ_L16	0x8401

/* Debug values*/
#define BKN_DEMO_AUDIO_LOOP_MODE 7
#define BKN_DEMO_READ_SIGNAL 8
#define BKN_DEMO_READ_FSK 28
#define BKN_DEMO_FSK_TEST_PRINT 29
#define BKN_DEMO_READ_CTCSS 36
#define BKN_DEMO_SEND_CTCSS_END 37
#define BKN_DEMO_READ_DTMF  24
#define BKN_DEMO_READ_CDCSS 38
#define BKN_DEMO_READ_SELCALL 34


/* Config values*/ 
#define BKN_DEMO_SPEECH_TX    0x80
#define BKN_DEMO_SPEECH_RX    0x81
#define BKN_DEMO_DTMF_TX      0x82
#define BKN_DEMO_DTMF_TX_ONLY 0x42
#define BKN_DEMO_DTMF_RX     0x83
#define BKN_DEMO_SELCALL_TX  0x84
#define BKN_DEMO_SELCALL_TX_ONLY 0x44
#define BKN_DEMO_SELCALL_RX  0x85
#define BKN_DEMO_FSK_TX       0x86
#define BKN_DEMO_FSK_TX_ONLY  0x46
#define BKN_DEMO_FSK_RX       0x87
#define BKN_DEMO_CTCSS_TX     0x88
#define BKN_DEMO_CTCSS_3_TX   0x89
#define BKN_DEMO_CTCSS_RX     0x8A
#define BKN_DEMO_CDCSS_TX     0x8B
#define BKN_DEMO_CDCSS_RX     0x8C
#define BKN_DEMO_SET_TOT      0x8D
#define BKN_DEMO_SET_VOX      0x8E
#define BKN_DEMO_SET_MIC_THRSHLD 0x8F
#define BKN_DEMO_SET_TX_FREQ  0x90
#define BKN_DEMO_SET_RX_FREQ  0x91
#define BKN_DEMO_SET_CHANNEL_NUM 0x92
#define BKN_DEMO_RESET        0x93
#define BKN_DEMO_POWERDOWN    0x94
#define BKN_DEMO_POWERUP      0x95
#define BKN_DEMO_SET_SNR_THRSHLD 0x96
#define BKN_DEMO_SET_RSSI_THRSHLD 0x97
#define BKN_DEMO_SET_MUTE     0x98
#define BKN_DEMO_SET_TX_VOUME 0x99
#define BKN_DEMO_SET_RX_VOUME 0x9A
#define BKN_DEMO_DISABLE_SCRAMBLING 0x9B
#define BKN_DEMO_SET_INVERSE_FREQUENCY 0x9C
#define BKN_DEMO_SET_ENCRYPTION 0x9D
#define BKN_DEMO_QUERY_SIGNAL   0x9E
#define BKN_DEMO_FSK_TEST_CONFIG 0x9F
#define BKN_DEMO_FSK_CONFIG    0xA0
#define BKN_DEMO_FSK_TEST_START 0xA1
#define BKN_DEMO_FSK_TEST_STOP 0xA2
#define BK_CMD_WRITE_REG      0xA3
#define BK_CMD_READ_REG       0xA4
#define BK_CMD_DISABLE_EX0    0xA5
#define BKN_DEMO_ENABLE_INTR  0xA6
#define BKN_DEMO_ENABLE_AGC   0xA7
#define BKN_DEMO_FSK_AIR_MODE 0xA8
#define BKN_DEMO_AUDIO_LOOP_MODE 0xA9
#define BK_RAMP_UP_DOWN2      0xAA

/* interrupt mask define */ 
#define INTM_RESERVED1         0x0001 
#define INTM_FSK_RX_COMPLETE   0x0002 
#define INTM_TOT_TIMEOUT       0x0004 
#define INTM_VOX_DETECT        0x0008 
#define INTM_CDCSS_LOSS        0x0010 
#define INTM_CDCSS_RX          0x0020 
#define INTM_AFC_LINKLOST      0x0040 
#define INTM_CTCSS_LOSS        0x0080 
#define INTM_CTCSS_RX          0x0100 
#define INTM_SELCALL_RX        0x0200 
#define INTM_DTMF_RX           0x0400 
#define INTM_FSK_FIFO_NEEDREAD 0x0800 
#define INTM_FSK_HEAD_RX       0x1000 
#define INTM_FSK_FIFO_NEEDFILL 0x2000 
#define INTM_FSK_TX_SUCCESS    0x4000 
#define INTM_ENABLE_INTR       0x8000 
 
/* interrupt status define */ 
#define INTS_RESERVED1         0x0001 
#define INTS_FSK_RX_COMPLETE   0x0002 
#define INTS_TOT_TIMEOUT       0x0004 
#define INTS_VOX_DETECT        0x0008 
#define INTS_CDCSS_LOSS        0x0010 
#define INTS_CDCSS_RX          0x0020 
#define INTS_AFC_LINKLOST      0x0040 
#define INTS_CTCSS_LOSS        0x0080 
#define INTS_CTCSS_RX          0x0100 
#define INTS_SELCALL_RX        0x0200 
#define INTS_DTMF_RX           0x0400 
#define INTS_FSK_FIFO_NEEDREAD 0x0800 
#define INTS_FSK_HEAD_RX       0x1000 
#define INTS_FSK_FIFO_NEEDFILL 0x2000 
#define INTS_FSK_TX_SUCCESS    0x4000 
#define INTS_RESET_ALL         0x8000 
 
/* register define */ 
#define REG_DTMF_TX_LOW_FREQ  		24 
#define REG_DTMF_TX_HIGH_FREQ  		25 
#define REG_FSK_ADDR_TYPE        	28 
#define REG_FSK_SIZE_CRCA        	29 
#define REG_FSK_PAYLOAD          	30 
#define REG_FSK_TX_FIFO_STATUS    	31 
 
#define REG_SELCALL_TX_FREQ   		34 
#define REG_CTCSS_TX_FREQ   		37 
 
#define REG_DTMF_RX_FREQ   			77 
#define REG_DTMF_RX_STATUS    		78 
 
#define REG_FSK_RX_SIZE_CRCA 		80 
#define REG_FSK_PAYLOAD_FIFO 		82 
#define REG_FSK_RX_FIFO_STATUS  	83 
 
#define REG_SELCALL_RX_FREQ  		86 
#define REG_SELCALL_RX_CFG    		87 
 
#define REG_CTCSS_RX_FREQ  		    91 
#define REG_CTCSS_RX_CFG  			92 
 
#define REG_INTR_MASK    			115 
#define REG_INTR_STATUS   			116 

 
typedef enum tag_CHN_SPC_ 
{ 
	SPACING_12_5K =	0x00,	/* 12.5K channel */ 
	SPACING_25K	  =	0x01,	/*   25K channel */ 
}CHN_SPC; 
 
 
typedef enum tag_FSK_TYPE_ 
{ 
	FSK_TX_TYPE0, 
	FSK_TX_TYPE1, 
	FSK_TX_TYPE2, 
	FSK_TX_TYPE3, 
	FSK_TX_TYPE4  
}FSK_TYPE; 
 
typedef enum tag_SIGNAL_TYPE_ 
{ 
	SPEECH, 
	DTMF, 
	SELCALL, 
	FSK, 
	CTCSS, 
	CDCSS, 
	SIGNAL_MAX, 
}SIGNAL_TYPE; 
 
typedef struct  tag_FSK_CFG_ 
{ 
	UINT16  fsk_syncword; 
	unsigned char   fsk_scrmb; 
	unsigned char   fsk_addr; 
	unsigned char   fsk_type; 
	unsigned char   fsk_thrshld; 
	unsigned char   fsk_len; 
	unsigned char   fsk_delay; 
	unsigned char   fsk_times; 
	unsigned char   fsk_is_scrmb; 
	unsigned char   fsk_dir; 
	unsigned char   fsk_space; 
	unsigned char   fsk_air; 
}FSK_CFG;
 
typedef struct{
   unsigned char len;
   unsigned char* Ptr;
} BufferStructure;
 
 
 
void EnableGlobalIntr(unsigned short intr_value); 
void DisableGlobalIntr(); 
void BK_Ramp_Table_Init(); 
void BK_RampUp_Enable(); 
void BK_RampDown_Enable(); 
void BK_Analog_Init(); 
void BK_SPI_Trigger(); 
void BK_Set_Channel_Space(CHN_SPC chn_spc); 
void BK_RX2TX(); 
void BK_TX2RX(); 
void BK_Switch_TX_Type(SIGNAL_TYPE type); 
void BK_Switch_RX_Type(SIGNAL_TYPE type); 
void BK_VOX_RX2TX(); 
void BK_VOX_TX2RX(); 
void BK_Enable_Intr(unsigned short enable_value); 
void BK_Clear_Intr(UINT16 intr_value); 
void BK_Enable_TX_InbandSignal(); 
void BK_Disable_TX_InbandSignal(); 
unsigned char BK_Is_RX_SubAudioSignal(); 
unsigned char BK_Is_TX_SubAudioSignal(); 
void BK_Enable_TX_SubAudioSignal(); 
void BK_Disable_TX_SubAudioSignal(); 
void BK_Enable_RX_SubAudioSignal(); 
void BK_Disable_RX_SubAudioSignal(); 
unsigned char BK_Is_Soft_Mute(); 
void BK_Disable_Soft_Mute(); 
void BK_Enable_Soft_Mute(); 
unsigned char BK_SELCALL_Read(); 
void BK_Enable_Hard_Mute(); 
void BK_Disable_Hard_Mute(); 
unsigned char BK_CTCSS_Read(); 
void BK_FSK_Read_Init(); 
void BK_FSK_Read_FIFO(); 
unsigned char BK_DTMF_RX_Read(); 
void BK_Intr_Task(); 
void BK_DTMF_TX(unsigned char *buf, unsigned char len); 
void BK_DTMF_RX(); 
void BK_SELCALL_TX(unsigned char *buf, unsigned char len); 
void BK_SELCALL_RX(); 
void BK_FSK_TX_Init(FSK_TYPE type, unsigned char *buf, unsigned char len); 
void BK_FSK_TX(unsigned char *buf, unsigned char len); 
void BK_FSK_RX(); 
void BK_CTCSS_TX(unsigned char index); 
void BK_CTCSS_RX(unsigned char index); 
void BK_CDCSS_TX(unsigned char index); 
void BK_CDCSS_RX(unsigned char index); 
void BK_Set_TOT(unsigned char timeout); 
void BK_Set_VOX(unsigned char vox_threshold); 
void BK_Set_MIC_Threshold(unsigned char mic_threshold); 
void BK_Set_Freq(unsigned char dir,  double frequency); 
void BK_Reset(); 
void BK_Powerdown(); 
void BK_Powerup(); 
void BK_Set_SNR_Threshold(unsigned char thrshld); 
void BK_Set_RSSI_Threshold(unsigned char thrshld); 
void BK_Set_Soft_Mute(unsigned char index); 
void BK_Set_TX_Volume(unsigned char volume); 
void BK_Set_RX_Volume(unsigned char volume); 
void BK_Disable_Scrambling(unsigned char dir); 
void BK_Set_Inverse_Frequency(unsigned char dir, unsigned char freq); 
void BK_Set_Encryption(unsigned char dir, unsigned char TM,unsigned char TR); 
unsigned char BK_Read_RSSI(); 
unsigned char BK_Read_Signal_Valid(); 
unsigned char BK_Read_SNR(); 
unsigned char BK_Read_Audio_RSSI(); 
UINT16 BK_Read_AFC_Indicator(); 
void Send_3_CTCSS(unsigned char* buf); 
void Enable_AGC(unsigned char b_enable); 
void Enable_TX_FSK_Air(unsigned char b_enable); 
void Enable_RX_FSK_Air(unsigned char b_enable); 
void Enable_RX_Parallel_Mode(unsigned char p_mode); 
void Enter_TX_Audio_LoopBack_Mode(); 
void Exit_TX_Audio_LoopBack_Mode(); 
void BK_FSK_Init(FSK_CFG *cfg); 
void BK_FSK_Test(); 
void Send_CTCSS_Tag(unsigned char tag1,unsigned char tag2,unsigned char send_main_CTCSS,unsigned char Send_CTCSS_times); 
void Send_3_CTCSS_Test(); 
void BK_RX_SQ(); 
 
 
void DelayMs(UINT16 ms); 
void Delay1us(); 
void disable_fsk_test(); 
void EnableAirMode(unsigned char dir,unsigned char air);
void command_handle(void);
void BK_Init(double frequency); 
