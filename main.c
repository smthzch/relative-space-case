/*
 * Relative Spatial Case, that also tells time
 
 * Created: 3/9/2016 18:10:23
 * Author : Zach Smith
 * Email: ezke.smith@gmail.com
 *
 *built for ATMEGA328p
 *
 *Peripherals:
 *	EM506 GPS module
 *  ADXL335 accelerometer
 *	16x2 LCD w/ SPL780D display driver equivalent to HD...
 
 *LCD Library
 * Title	:   C file for the HD44780U LCD library (lcd.c)
 * Author:    Peter Fleury <pfleury@gmx.ch>  http://tinyurl.com/peterfleury
 * 
 */ 

#define F_CPU 8000000UL //clock 8MHz
#define PRESCALED_BAUD ((F_CPU / (16UL * 4800)) - 1) //for usart

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <Math.h>

#include "lcd.h"

void usart_init();
char usart_get();
void usart_set(char data);
void getGGA();
void getRMC();
void getPRM();
void shiftMode();
double getADC();
void xyz_init();
void xyz_init2();
void xyzNormalize(double xR, double yR, double zR);
void setNormalization(double xG, double yG, double zG, double g);
void setXnormalize();
void getPitchRoll();


//startup button counter
volatile int startup = 0;

//user selected mode and output strings from gps
volatile int modeNum = 0;
volatile int pressed = 0;
char *modes[] = {"GGA", "RMC", "PRM"};
char moder[] = "000";

char lat[15];
char lon[15];
char course[7];
char speed[15];
char date[15];
char time[16];
char temp;

//backlight
volatile int bckMode = 5;

//accelerometer

double xyAngle, gAngle; //to normalize gravity vector
double xAngle; //to normalize x and y axis components for pitch and roll
double norm[] = {0,0,0};

//change display out intterupts -> Lat/Long -> Speed/Course -> DateTime
//button push interrupt
ISR(PCINT1_vect){	
		//turn off button interrupt
		PCMSK1 = 0;
		//turn on timer w/ prescaler clk/64
		TCCR1B = (3<<CS10);
		//timer overflow interrupt
		TIMSK1 = (1<<TOIE1);	
	
	if (!pressed){
		if (startup==0){
			xyz_init();
			startup++;
		}
		else if (startup==1){
			xyz_init2();
			startup++;
		}
		else{
			lcd_clrscr();
			lcd_puts("   Loading...   ");
			modeNum++;
			if(modeNum==4){modeNum=0;}
		}
	}	
	//prevents follow up interrupt
	pressed = 1;
}

//timer interrupt for debounce button ~.5sec
ISR(TIMER1_OVF_vect){
	//turn on button interrupt
	PCMSK1 |= 1 << PCINT13;
	PCMSK0 |= 1 << PCINT7;
	//turn off and reset timer counter 16bit
	TCCR1B = 0;
	TCNT1H = 0;
	TCNT1L = 0;
	pressed = 0;
}

//change lcd backlight intterupt
ISR(PCINT0_vect){
	//turn off button interrupt
	PCMSK0 = 0;
	//turn on timer as above
	TCCR1B = (3<<CS10);
	TIMSK1 = (1<<TOIE1);
	if (!pressed){
		//turn off current backlight
		PORTD &= ~(1<<bckMode);
		bckMode++;
		if(bckMode==8){bckMode=5;}
		//change backlight
		PORTD |= (1<<bckMode);		
	}
	pressed = 1;
}

int main(void)
{
	//set input pullup pinc5 for display mode select
	DDRC &= ~(1<<5);
	PORTC |= (1<<5);
	
	//set PINC5 interrupt for button
	PCICR |= 1 << PCIE1;
	PCMSK1 |=  1 << PCINT13;
	
	//set PINCB0 as input for backlight select
	DDRB &= ~(1<<7);
	PORTB |= 1<<7;
	PCICR |= 1<<PCIE0;
	PCMSK0 |= 1<<PCINT7;
	//set D5-7 output pins for backlight
	DDRC |= (1<<5)|(1<<6)|(1<<7);
	PORTD |= 1<<5;	
	sei();	//enable global interrupts
	
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts("   Hold Level   ");	
	lcd_gotoxy(0,1);
	lcd_puts("<OK>");
	
	while(startup<2){}
	
	lcd_clrscr();
	lcd_puts("  Initializing  ");
	
	usart_init();
		
	while(1){
		if (modeNum == 3){
			getPitchRoll();
		}
		else{
			if (strcmp(modes[modeNum], moder) == 0)
			{	
				if (modeNum == 0){
					getGGA();				
					shiftMode();
				}
				else if (modeNum == 1){
					getRMC();
					shiftMode();
				}
				else if (modeNum == 2){
					getPRM();
					shiftMode();
				}
				else{
					shiftMode();
				}		
			}
			else
			{			
				//shift moder data left one and retrieve new char
				moder[0] = moder[1];
				moder[1] = moder[2];
				moder[2] = usart_get();
		}
		}
	}   	
	return 0; 
}

void usart_init()
{
	//set baud rate
	UBRR0H = (PRESCALED_BAUD>>8);
	UBRR0L = PRESCALED_BAUD;
	//enable receiver and transmit pins
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	//set data length and such 8 bits
	UCSR0C |= (2<<UCSZ00);
}

char usart_get()
{
	while(!(UCSR0A & (1<<RXC0))){}
	return UDR0;
}

void usart_set(char data)
{
	while(!(UCSR0A&(1<<UDRE0))){}
	UDR0 = data;
}
//lat long string
void getGGA()
{
	//skip utc time string
	int i = 0;
	for (i = 0; i < 12; i++)
	{
		usart_get();
	}
	i=0;
	temp = usart_get();
	while (temp!=',')
	{
		if (i==2){
			lat[i++] = (char)223;			
		}
		lat[i++] = temp;
		temp = usart_get();
	}
	temp = '\'';
	lat[i++] = temp;
	temp = usart_get();
	lat[i++] = temp;
	lat[i] = '\0';
	usart_get();
	i =0;
	temp = usart_get();
	while(temp != ',')
	{
		if(i == 3){
			lon[i++] = (char)223;
		}
		lon[i++] = temp;
		temp = usart_get();
	}
	temp = '\'';
	lon[i++] = temp;
	temp = usart_get();
	lon[i++] = temp;
	lon[i] = '\0';
	//check if lock valid
	usart_get();
	char fix = usart_get();
	if (fix >= '1'){
		lcd_clrscr();
		lcd_puts(lat);
		lcd_gotoxy(0,1);
		lcd_puts(lon);
	}
	else{
		lcd_clrscr();
		lcd_puts("  Initializing  ");
		lcd_gotoxy(0,1);
		lcd_puts("    GPS Lock    ");
	}	
}
//speed and course string
void getRMC()
{	
	int i = 0;
	for (i = 0; i < 12; i++)
	{
		usart_get();
	}
	char fix = usart_get();
	if (fix == 'A'){
		for (i = 0; i < 26; i++)
		{
			usart_get();
		}
		i=0;
		temp = usart_get();
		while (temp!=',')
		{
			speed[i++] = temp;
			temp = usart_get();
		}
		speed[i++] = ' ';
		speed[i++] = 'K';
		speed[i++] = 'n';
		speed[i++] = 'o';
		speed[i++] = 't';
		speed[i++] = 's';
		speed[i] = '\0';
	
		i =0;
		temp = usart_get();
		while(temp != ',')
		{
			course[i++] = temp;
			temp = usart_get();
		}
		course[i++] = (char)223; //degree symbol
		course[i++] = '\0';
		lcd_clrscr();
		lcd_puts(speed);
		lcd_gotoxy(0,1);
		lcd_puts(course);
	}
	else{
		lcd_clrscr();
		lcd_puts("  Initializing  ");
		lcd_gotoxy(0,1);
		lcd_puts("Speed and Course");
	}
}
//date and time string
void getPRM()
{
	int i = 0;
	for (i = 0; i < 2; i++)
	{
		usart_get();
	}
	i=0;
	temp = usart_get();
	while ((temp!=',')&&(temp!='.'))
	{
		if((i==2)||(i==5)){
			time[i++] = ':';
		}
		time[i++] = temp;
		temp = usart_get();
	}
	time[i] = '\0';	
	i =0;
	usart_get();
	usart_get();
	usart_get();
	usart_get();
	temp = usart_get();
	if (temp=='A'){
		for(i=0;i<7;i++){
			while(temp != ',')
			{
				temp = usart_get();
			}
			temp = usart_get();
		}
		i=0;
		while(temp != ',')
		{
			if((i==2)||(i==5)){
				date[i++] = '/';
			}
			date[i++] = temp;
			temp = usart_get();
		}
		date[i++] = ' ';
		date[i++] = 'U';
		date[i++] = 'T';
		date[i++] = 'C';
		date[i] = '\0';
		lcd_clrscr();
		lcd_puts(time);
		lcd_gotoxy(0,1);
		lcd_puts(date);
	}
	else{
		lcd_clrscr();
		lcd_puts("  Initializing  ");
		lcd_gotoxy(0,1);
		lcd_puts("    DateTime    ");
	}
}

void shiftMode()
{
	moder[0] = usart_get();
	moder[1] = usart_get();
	moder[2] = usart_get();
}

//analog to digital converter for accelerometer
double getADC(int ps1,int ps0)
{
	ADMUX = (ps0<<ADPS0)|(ps1<<ADPS1); //set pin to convert
	ADMUX |= (1<<ADLAR);
	
	/*
		[0,0,1] ADC1 = X
		[0,1,0] ADC2 = Y
		[0,1,1] ADC3 = Z
		ADPS2 always 0
	*/
	ADCSRA = (1<<ADPS0)|(1<<ADPS1); //start conversion select pin input
	ADCSRA |= (1<<ADEN);
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA&(1<<ADIF))){} //wait for conversion to complete
	/*
	1023 = 3g
	0 = -3g
	1023 / 2 = 0g
	1023 / 6 = g = 170.5
	*/
	int raw = (ADCH<<2); //least two bits in ADCL - ignore for now
	double scaled = (raw / 170.5) - 3;
	return  scaled;
}

void xyz_init()
{
	double xG = 0, yG = 0, zG = 0, g; //level vector

	lcd_clrscr();
	lcd_puts(" Calibrating... ");
	
	//retrieve xyz vectors of gravity
	int i = 0;
	for (i=0; i<30; i++){
		xG += getADC(0,1);
	}
	xG = xG /30; //get average for more accurate reading
	for (i=0; i<30; i++){
		yG += getADC(1,0);
	}
	yG = yG / 30;
	for (i=0; i<30; i++){
		zG += getADC(1,1);
	}
	zG = zG / 30;
	g = sqrt(pow(xG,2)+pow(yG,2)+pow(zG,2)); //length of base gravity vector
	setNormalization(xG, yG, zG, g); //set angles for normalization transformation
	
	lcd_clrscr();
	lcd_puts("  Tilt Forward  ");
	lcd_gotoxy(0,1);
	lcd_puts("<OK>");
}

void xyz_init2()
{	
	lcd_clrscr();
	lcd_puts(" Calibrating... ");
	
	double xG1 = 0, yG1 = 0, zG1 = 0; //tilted forward vector
	int i = 0;
	for (i=0; i<30; i++){
		xG1 += getADC(0,1);
	}
	xG1 = xG1 /30; //get average for more accurate reading
	for (i=0; i<30; i++){
		yG1 += getADC(1,0);
	}
	yG1 = yG1 / 30;
	for (i=0; i<30; i++){
		zG1 += getADC(1,1);
	}
	zG1 = zG1 / 30;
	xyzNormalize(xG1, yG1, zG1);
	setXnormalize();
}

void setNormalization(double xG, double yG, double zG, double g)
{
	//get distance from xy origin and normalized g vector [0,0,g]
	double xyDist, gDist;
	xyDist = sqrt(pow(xG, 2) + pow(yG, 2));
	gDist = sqrt(pow(xG, 2) + pow(yG, 2) + pow(zG - g, 2));
	
	//get angle from x axis and normalized g vector [0,0,g]
	xyAngle = acos(xG / xyDist);
	gAngle = asin((gDist / 2) / g) * 2;
}

void xyzNormalize(double xR, double yR, double zR)
{
	double x0, x1, y1, z1;
	//rotate about z first to remove y component
	x0 = cos(xyAngle)*xR - sin(xyAngle)*yR;
	y1 = sin(xyAngle)*xR + cos(xyAngle)*yR;
	//now rotate around y
	x1 = cos(gAngle)*x0 - sin(gAngle)*zR;
	z1 = sin(gAngle)*x0 + cos(gAngle)*zR;
	norm[0] = x1;
	norm[1] = y1;
	norm[2] = z1;
}

void setXnormalize()
{
	double xyDist;
	xyDist = sqrt(pow(norm[0], 2) + pow(norm[1], 2));
	xAngle =  acos(norm[0] / xyDist);
}

void getPitchRoll()
{
	char out1[20];
	char out2[20];

	double x, y, z, x1, y1, z1, a, b, roll, pitch;
	x = getADC(0,1);
	y = getADC(1,0);
	z = getADC(1,1);
	
	xyzNormalize(x,y,z);
	
	//rotate out x comp
	x1 = cos(xAngle)*(norm[0]) - sin(xAngle)*(norm[1]);
	y1 = sin(xAngle)*(norm[0]) + cos(xAngle)*(norm[1]);
	z1 = norm[2];
	
	a = sqrt(pow(x1, 2) + pow(z1, 2));
	b = sqrt(pow(y1, 2) + pow(z1, 2));
	pitch = -asin(x1/a)*(180/M_PI);
	roll = -asin(y1/b)*(180/M_PI);
	//format pitch string //printf rounds decimal to even at .5; (char)223 == degree symbol
	if (pitch>=9.95){
		sprintf(out1, "Pitch:  %.1f%c", pitch, (char)223);
	}
	else if (pitch>=0){
		sprintf(out1, "Pitch:   %.1f%c", pitch, (char)223);
	}
	else if (pitch>-9.94){
		sprintf(out1, "Pitch:  %.1f%c", pitch, (char)223);
	}
	else{
		sprintf(out1, "Pitch: %.1f%c", pitch, (char)223);
	}
	//format roll string //ditto
	if (roll>=9.95){
		sprintf(out2, " Roll:  %.1f%c", roll, (char)223);
	}
	else if (roll>=0){
		sprintf(out2, " Roll:   %.1f%c", roll, (char)223);
	}
	else if (roll>-9.94){
		sprintf(out2, " Roll:  %.1f%c", roll, (char)223);
	}
	else{
		sprintf(out2, " Roll: %.1f%c", roll, (char)223);
	}
		
	lcd_clrscr();
	lcd_puts(out1);
	lcd_gotoxy(0,1);
	lcd_puts(out2);
	_delay_ms(100);
}
