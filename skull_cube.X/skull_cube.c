//skull cube 0.0
#include "p33FJ128GP802.h"
#include "delay.h"
#include <stdio.h>

_FOSCSEL(FNOSC_PRIPLL & IESO_OFF);   //Prim. Osc (XT, HS, EC) w/ PLL, Two speed osc off, Temp protect off
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_HS); //Clock Switch & Clock Monitor off, Osc 2 clock, High Speed

_FPOR(FPWRT_PWR1); //1 ms delay on startup
_FWDT(FWDTEN_OFF); // Turn off the Watch-Dog Timer.
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);            /* Disable Code Protection */
_FICD(ICS_PGD1 & JTAGEN_OFF);
int __C30_UART=2;

#define FCY		40000000		
#define SPIFAST 		0b0000000000111011;
#define SPISLOW 		0b0000000000100000;


//equation for REALBAUD: realbaud = (FCY/(BAUDRATE*16))-1; (round to nearest number, up or down)
#define BAUDRATE 9600
#define REALBAUD 86 //115200
#define BUTTON1   	        _RB15
#define TILT				_RB14
#define TLC5947_BLANK		_LATB6     // Control pin for servo motor
#define TLC5947_XLAT		_LATB5     // 5947
#define POK_CHG             _RB4         			 // activity light
#define ZENER               _RB7         			 // activity light

#define STEPPER1		_LATB12     // 5947
#define STEPPER2		_LATB13     // 5947
#define STEPPER3		_LATB11     // 5947
#define STEPPER4		_LATB10     // 5947


void send_to_5947(unsigned char iR1, unsigned char iG1, unsigned char iB1,unsigned char iR2, unsigned char iG2, unsigned char iB2);
void spiout2(unsigned char datatosend);
void init(void);
void update_display(void);
void status_report(void);
void merge_displays(void);
void update_servos(void);
void update_stepper(void);
void animate_stepper(int speed);
void step(char direction);
void fire (char unit, int calmness, int temperature);



typedef struct{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} pixelelement;

pixelelement pixel[8];

#define BLANKED 1
#define NOT_BLANKED 0

unsigned int const GAMMA[256]={
0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 7, 9, 10,12,14,16,
18,20,22,25,27,30,33,36,39,42,45,49,52,56,60,64,
68, 72, 76, 81, 85, 90, 95, 100,105,110,115,121,126,132,138,144,
150,156,162,169,175,182,189,196,203,210,217,225,232,240,248,256,
264,272,280,289,297,306,315,324,333,342,351,361,370,380,390,400,
410,420,430,441,451,462,473,484,495,506,517,529,540,552,564,576,
588,600,612,625,637,650,663,676,689,702,715,729,742,756,770,784,
798,812,826,841,855,870,885,900,915,930,945,961,976,992,1008,1024,
1040,1056,1072,1089,1105,1122,1139,1156,1173,1190,1207,1225,1242,1260,1278,1296,
1314,1332,1350,1369,1387,1406,1425,1444,1463,1482,1501,1521,1540,1560,1580,1600,
1620,1640,1660,1681,1701,1722,1743,1764,1785,1806,1827,1849,1870,1892,1914,1936,
1958,1980,2002,2025,2047,2070,2093,2116,2139,2162,2185,2209,2232,2256,2280,2304,
2328,2352,2376,2401,2425,2450,2475,2500,2525,2550,2575,2601,2626,2652,2678,2704,
2730,2756,2782,2809,2835,2862,2889,2916,2943,2970,2997,3025,3052,3080,3108,3136,
3164,3192,3220,3249,3277,3306,3335,3364,3393,3422,3451,3481,3510,3540,3570,3600,
3630,3660,3690,3721,3751,3782,3813,3844,3875,3906,3937,3969,4000,4032,4064,4095
};

unsigned char const hotBits[1024] = {
    230, 119, 180, 73, 0, 164, 248, 169, 18, 153, 180, 184, 174, 26, 171,
    125, 26, 45, 16, 53, 172, 48, 137, 228, 245, 117, 21, 123, 155, 36, 45,
    11, 33, 119, 96, 241, 72, 34, 238, 34, 163, 192, 27, 80, 1, 72, 140,
    16, 187, 115, 251, 238, 131, 163, 83, 135, 131, 229, 55, 241, 63, 15,
    121, 224, 254, 124, 112, 39, 57, 249, 114, 232, 36, 178, 159, 137, 242,
    222, 239, 234, 135, 41, 67, 114, 27, 152, 69, 163, 230, 141, 80, 160,
    53, 128, 62, 16, 249, 81, 195, 8, 164, 244, 248, 77, 95, 4, 18, 130,
    140, 60, 54, 181, 145, 50, 165, 97, 13, 114, 73, 181, 138, 136, 187,
    213, 230, 220, 182, 11, 243, 32, 150, 252, 227, 161, 4, 140, 25, 171,
    43, 66, 129, 170, 167, 153, 78, 247, 209, 6, 59, 195, 254, 254, 225,
    132, 165, 120, 100, 87, 142, 222, 245, 83, 250, 229, 40, 65, 116, 51,
    17, 84, 85, 203, 169, 19, 68, 52, 193, 65, 8, 73, 19, 86, 69, 96, 71,
    187, 99, 62, 223, 93, 236, 163, 174, 230, 104, 171, 46, 125, 77, 225,
    73, 85, 202, 234, 172, 107, 33, 81, 197, 93, 33, 226, 215, 219, 234,
    209, 227, 191, 139, 88, 18, 207, 68, 62, 55, 186, 211, 210, 190, 0, 18,
    10, 216, 226, 242, 93, 168, 97, 234, 91, 140, 56, 27, 23, 247, 106, 115,
    62, 60, 38, 175, 39, 175, 102, 10, 215, 64, 216, 1, 158, 122, 81, 58,
    239, 179, 31, 220, 164, 36, 164, 38, 238, 187, 149, 242, 190, 33, 67,
    197, 70, 250, 33, 77, 161, 232, 84, 171, 135, 206, 173, 25, 140, 75,
    229, 147, 252, 5, 222, 97, 35, 155, 162, 79, 212, 228, 128, 139, 50,
    66, 48, 25, 33, 53, 40, 104, 207, 167, 197, 247, 14, 8, 170, 203, 136,
    44, 65, 53, 230, 248, 219, 57, 17, 89, 253, 70, 14, 146, 32, 203, 148,
    165, 198, 125, 55, 33, 132, 201, 70, 199, 182, 87, 56, 216, 8, 218, 93,
    195, 216, 54, 247, 214, 40, 151, 64, 159, 162, 42, 121, 217, 39, 40,
    18, 91, 221, 236, 186, 31, 244, 158, 90, 127, 19, 95, 59, 172, 66, 40,
    136, 29, 251, 235, 64, 119, 225, 11, 153, 63, 166, 62, 30, 48, 71, 181,
    31, 145, 16, 52, 124, 46, 253, 248, 212, 153, 146, 64, 18, 102, 215,
    233, 60, 53, 193, 13, 136, 224, 102, 24, 156, 40, 213, 161, 149, 123,
    251, 71, 2, 165, 190, 172, 118, 51, 155, 249, 106, 248, 225, 137, 1,
    128, 132, 219, 199, 217, 7, 235, 125, 211, 188, 151, 121, 170, 115, 49,
    222, 235, 92, 1, 135, 167, 231, 230, 186, 225, 127, 112, 233, 77, 71,
    180, 185, 224, 255, 78, 13, 25, 208, 143, 170, 122, 131, 150, 60, 178,
    35, 81, 105, 87, 46, 35, 174, 250, 150, 192, 104, 203, 43, 170, 21, 125,
    200, 191, 235, 132, 32, 80, 57, 119, 228, 78, 80, 1, 81, 217, 4, 24,
    172, 120, 98, 202, 203, 128, 195, 149, 11, 109, 133, 187, 108, 143, 104,
    150, 232, 113, 4, 142, 193, 167, 77, 216, 38, 4, 53, 96, 79, 84, 223,
    241, 217, 10, 34, 162, 182, 95, 22, 145, 11, 92, 159, 69, 89, 102, 102,
    6, 16, 113, 165, 205, 10, 101, 46, 128, 175, 107, 93, 18, 47, 253, 91,
    204, 176, 33, 249, 15, 87, 74, 218, 211, 121, 102, 52, 203, 25, 98, 164,
    163, 67, 11, 69, 34, 114, 165, 192, 219, 251, 11, 151, 134, 74, 86, 91,
    241, 34, 126, 129, 5, 12, 209, 28, 81, 120, 94, 198, 213, 163, 227, 9,
    164, 47, 108, 148, 120, 95, 216, 130, 108, 148, 187, 146, 57, 21, 93,
    157, 62, 108, 181, 175, 137, 105, 230, 206, 161, 191, 169, 193, 51, 131,
    118, 237, 23, 159, 105, 81, 197, 126, 242, 181, 247, 113, 36, 211, 37,
    114, 10, 238, 139, 211, 254, 187, 51, 189, 95, 87, 241, 168, 126, 96,
    26, 27, 84, 179, 79, 88, 134, 199, 247, 175, 225, 83, 58, 36, 209, 179,
    137, 16, 115, 213, 3, 117, 225, 172, 185, 229, 114, 15, 123, 243, 178,
    175, 118, 82, 20, 252, 240, 174, 196, 2, 232, 99, 24, 156, 200, 108,
    166, 97, 2, 59, 60, 89, 251, 23, 62, 164, 228, 207, 244, 69, 6, 104,
    158, 118, 56, 234, 219, 164, 83, 31, 218, 3, 201, 197, 228, 175, 3, 216,
    82, 63, 33, 205, 168, 42, 188, 152, 246, 50, 188, 253, 220, 33, 250,
    40, 216, 48, 151, 118, 218, 61, 189, 2, 215, 80, 78, 21, 173, 243, 23,
    143, 199, 215, 255, 248, 11, 234, 144, 154, 169, 183, 143, 114, 125,
    156, 197, 101, 62, 21, 0, 228, 185, 169, 29, 14, 140, 238, 7, 99, 90,
    42, 198, 8, 252, 242, 155, 183, 193, 83, 164, 49, 123, 151, 196, 60,
    63, 108, 58, 162, 104, 12, 153, 55, 142, 173, 96, 54, 46, 250, 240, 26,
    13, 184, 36, 15, 174, 8, 37, 22, 169, 101, 228, 21, 144, 83, 156, 43,
    193, 57, 213, 28, 170, 246, 52, 171, 1, 140, 52, 116, 18, 4, 23, 205,
    126, 166, 44, 147, 51, 218, 122, 33, 191, 19, 91, 183, 230, 33, 104,
    141, 92, 180, 192, 98, 190, 239, 28, 83, 228, 194, 58, 179, 126, 231,
    205, 139, 103, 92, 108, 13, 169, 182, 26, 95, 65, 127, 215, 146, 130,
    244, 244, 54, 145, 118, 31, 223, 221, 152, 102, 184, 159, 40, 38, 193,
    201, 28, 167, 41, 218, 206, 81, 253, 172, 47, 23, 214, 190, 62, 246,
    42, 141, 168, 251, 94, 47, 48, 100, 142, 144, 91, 94, 29, 107, 93, 255,
    100, 76, 106, 100, 142, 194, 119, 171, 159, 215, 155, 69, 193, 77, 210,
    224, 169, 135, 4, 24, 63, 102, 183, 213, 8, 212, 212, 143, 22, 48, 97,
    11, 106, 13, 93
};


void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){
	static int acc=400;

	if(TILT){acc=1000;}
	if(acc){acc-=4;}
	int cal=1000-acc;
	cal/=30;
	cal+=1;
	fire(0,cal,acc);
	fire(3,cal,acc);
	fire(4,cal,acc/2);
	fire(5,cal,acc/2);
	fire(6,cal,acc/2);
	fire(7,cal,acc/2);
	IFS0bits.T2IF=0;
	IEC0bits.T2IE=1;
}

void spiout2(unsigned char datatosend){
unsigned char temp=0;
	temp = SPI2BUF;			// dummy read of the SPI1BUF register to clear the SPIRBF flag
	SPI2BUF = datatosend;		// write the data out to the SPI peripheral
    while (!SPI2STATbits.SPIRBF){;}	// wait for the data to be sent out
}

int main(void) {
	init();
	while(1){

		if(BUTTON1==0){status_report();}

//		merge_displays();
//		update_servos();
		animate_stepper(1000);
		update_display();
		__delay32(40000);
	}
} 

void status_report(void){
	if(POK_CHG==0){
		pixel[0].r++;
	}
	if(ZENER==0){
		pixel[4].r++;
	}
	if(TILT==0){
		pixel[5].g++;
	}
	pixel[7].r--;
	pixel[7].g++;
	pixel[7].b+=2;
}

void animate_stepper(int speed){
	static int  accumulate=0;
	static char oldsign=0;

	//clip the values
	if(speed>30000){speed=30000;}
	if(speed<30000){speed=-30000;}

	//invert the value to increase number of clicks as speed approaches 0
	char tick=0;
	if(speed>0){tick=1;speed=30000-speed;};
	if(speed<0){tick=-1;speed=-30000+speed;};

	//check for change of direction
	if(oldsign!=tick){accumulate=0;}
	oldsign=tick;

	//accumulate until reaching compare, call step function, clear accumulate resister  
	accumulate+=tick;
	if(accumulate==speed){
		step(tick);
		accumulate=0;
	}
}

void step(char direction){
	static unsigned char index=0;
	unsigned char step1[4]={1,0,0,1};
	unsigned char step2[4]={0,1,1,0};
	unsigned char step3[4]={1,1,0,0};
	unsigned char step4[4]={0,0,1,1};
	STEPPER1=step1[index];
	STEPPER2=step2[index];
	STEPPER3=step3[index];
	STEPPER4=step4[index];
	index+=direction;
}

void fire (char unit, int calmness, int temperature){
	static int acc=0;

	int temp_b=(hotBits[acc]/calmness)+temperature;
	int temp=0;

	temp=temp_b*2; if(temp>255){temp=255;};
	pixel[unit].r=(unsigned char)temp;

	temp=temp_b;
	temp-=87; if(temp<0){temp=0;}
	temp*=1; if(temp>255){temp=255;};
	pixel[unit].g=(unsigned char)temp;

	temp=temp_b;
	temp-=170; if(temp<0){temp=0;}
	temp*=1; if(temp>255){temp=255;};
	pixel[unit].b=(unsigned char)temp;

	acc++;
	acc%=1023;
}

void update_display(void){
	for(char count=0;count<8;count+=2){
		send_to_5947(pixel[count].r,pixel[count].g,pixel[count].b,pixel[count+1].r,pixel[count+1].g,pixel[count+1].b);
	}
	TLC5947_BLANK=BLANKED;
	TLC5947_XLAT=1;
	TLC5947_XLAT=0;
	TLC5947_BLANK=NOT_BLANKED;
}

void send_to_5947(unsigned char iR1, unsigned char iG1, unsigned char iB1,unsigned char iR2, unsigned char iG2, unsigned char iB2){
unsigned int temp=0;

	unsigned int R1=GAMMA[iR1];
	unsigned int G1=GAMMA[iG1];
	unsigned int B1=GAMMA[iB1];
	unsigned int R2=GAMMA[iR2];
	unsigned int G2=GAMMA[iG2];
	unsigned int B2=GAMMA[iB2];

	temp=B1/16;    //top 8 of Blue1
	spiout2((unsigned char)temp);
   	temp=(((B1*16)&0xF0)|((G1/256)&0x0F)); //bottom 4 of Blue1 and top 4 of Green1
	spiout2((unsigned char)temp);
	Nop();Nop();	Nop();Nop();
    temp=(unsigned char)G1;  //bottom 8 of Green1
	spiout2((unsigned char)temp);

	temp=R1/16;    //top 8 of Red1
	spiout2((unsigned char)temp);
	temp=(((R1*16)&0xF0)|((B2/256)&0x0F)); //bottom 4 of Blue and top 4 of Green
	spiout2((unsigned char)temp);
	Nop();Nop();	Nop();Nop();	Nop();Nop();	Nop();Nop();
	temp=(unsigned char)B2;//&0x0FF;  //bottom 8 of Green
	spiout2((unsigned char)temp);

	temp=G2/16;    //top 8 of Red
	spiout2((unsigned char)temp);
	temp=(((G2*16)&0xF0)|((R2/256)&0x0F)); //bottom 4 of Blue and top 4 of Green
	spiout2((unsigned char)temp);
	Nop();Nop();	Nop();Nop();
	temp=(unsigned char)R2;//&0x0FF;  //bottom 8 of Green
	spiout2((unsigned char)temp);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init(void){
unsigned int temp=0;

	//oh boy, here we go! Taks a deep breath:
	// Configure Oscillator to operate the device at 40Mips
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	// Fosc= 10M*40(2*2)=80Mhz for 10M input clock
	PLLFBD=30; // M=32
	CLKDIVbits.PLLPOST=0; // N1=2
	CLKDIVbits.PLLPRE=0; // N2=2

	//set up the Peripheral Pin Select Registers
	//*************************************************************
	 // Unlock Registers (Per DS39881B-page 102)
	//*************************************************************
	asm volatile (      
        	"MOV #OSCCON, w1 \n"       
    		"MOV #0x45, w2 \n"
     		"MOV #0x57, w3 \n"
      		"MOV.b w2, [w1] \n"
          	"MOV.b w3, [w1] \n"
          	"BCLR OSCCON,#6" 
	); 

	//INPUTS
	RPINR19bits.U2RXR=0x03;  //uart2 RX pin is B3
//	RPINR20bits.SDI1R=0x0B;  //spi1 data in pin B11
//	RPINR22bits.SDI2R=0x07;   //spi2 data in  pin B7

	//OUTPUTS
	RPOR1bits.RP2R= 0b00101;  //pin 2 is UART2 TX;
	RPOR4bits.RP8R= 0b01010;  //pin 8 is SPI 2 Data Out
	RPOR4bits.RP9R= 0b01011;  //pin 9 is SPI 2 Clock Out
//	RPOR7bits.RP14R= 0b01000; //SPI 1 clock out
//	RPOR7bits.RP15R= 0b00111;  //SPI 1 Data out



	//************************************************************
	// Lock Registers
	//************************************************************
   	asm volatile (                                                 // Lock Registers
    		"MOV #OSCCON, w1 \n"
            	"MOV #0x45, w2 \n"
            	"MOV #0x57, w3 \n"
            	"MOV.b w2, [w1] \n"
            	"MOV.b w3, [w1] \n"
            	"BSET OSCCON, #6"
 	);

	///////////////////port directions (most of the built in peripherals take over, so set those pins to input
	TRISA=0b1111111111111101;
	TRISB=0b1100001110011111;
	LATB=0b0000100000000000;
	///////////////////ADC pin configurations  (if you don't turn the damn thing off, it defaults to taking control of the pins.
 	//This even takes precedence over the peripherals 
	AD1CON1=0b1000010011100111;
	AD1CON2=0;
	AD1CON3=0b0001111111111111;
	AD1CON4=0;
	AD1CHS123=0;
	//AD1CHS0=2; //channel 2
	//AD1CHS0=1; //channel 1
	AD1CHS0=0; //channel 0
	AD1CSSL=0;
	AD1PCFGL=0b1111111111111111; //channels 0, 1 and 4 analog

	AD1CON1bits.SSRC = 0b111;  //automatic conversion start
	AD1CON1bits.ASAM = 1;  //automatic sample start after conversion
	//ADCValue = ADC1BUF0;  //read the value

	while(OSCCONbits.LOCK!=1) {;};




	//////////////////SPI2  this one is for the led driver!

	SPI2STAT = 0;  // disable the SPI module (just in case)
	IFS2bits.SPI2IF = 0; //Clear the Interrupt Flag
	IEC2bits.SPI2IE = 0; //disable the Interrupt
	SPI2CON2=0;
	SPI2CON1bits.DISSCK = 0; //Internal Serial Clock is Enabled.
	SPI2CON1bits.DISSDO = 0; //SDOx pin is controlled by the module.
	SPI2CON1bits.MODE16 = 0; //Communication is NOT word-wide (16 bits).
	SPI2CON1bits.SMP = 0; //Input Data is sampled at the middle of data output time.
	SPI2CON1bits.CKE = 1; //Serial output data changes on transition from
	//active clock state to idle clock state
	SPI2CON1bits.CKP = 0; //Idle state for clock is a low level;
	//active state is a high level
	SPI2CON1bits.MSTEN = 1; //Master Mode Enabled
	//(these two can be any combination except both being 1:1 (minimum cumulated divider is 1:2)

	SPI2CON1bits.SPRE = 0b111; //Secondary prescaler set to 1:1   //maximum allowed speed is 10MHz
	SPI2CON1bits.PPRE = 0b10; //Primary prescaler set to 1:4
//	SPI2CON1bits.SPRE = 0b000; //Secondary prescaler set to 1:1   //maximum allowed speed is 10MHz
//	SPI2CON1bits.PPRE = 0b00; //Primary prescaler set to 1:40
	SPI2STATbits.SPIEN = 1; //Enable SPI Module
	//SPI2BUF = 0x00; //Write data to be transmitted


	T2CONbits.TCS=0;
	T2CONbits.T32=0;
	T2CONbits.TCKPS=0b11; //00 
	T2CONbits.TON=1;
	TMR2 = 0x00; // Clear timer register
//	PR2 = 196; //was 50000
	PR2 = 10000;
	IFS0bits.T2IF=0;
	IEC0bits.T2IE=1;

	for(unsigned char loop=0;loop<8;loop++){
		pixel[loop].r=0;
		pixel[loop].g=0;
		pixel[loop].b=0;
	}
}

