#include<stdio.h>
#include<stdlib.h>
#include<pic16f877a.h>
#include<xc.h>
#define _XTAL_FREQ 4000000
#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7
#include "lcd.h"
#include "adc.h"
#include "delay.h"
#include "pwm.h"
#include "stdutils.h"
void main()
{
    TRISB0=0;// TRIGGER
    TRISB1=1;// ECHO
    TRISC1;//ON/OFF
    TRISB2=0;//GREEN LED
    TRISB3=0;//RED LED
    TRISD=0;//LCD CONFIGURATION
    TRISD0=0;
    TRISA0=1;
    TRISA1=1;
    TRISC1=0;
    TRISC2=0;
    PWM_Init(0);
    PWM_Init(1);
    long a;
    long b;
    long pwmValue0;
    long pwmValue1;
    int c;
    ADC_Init();
    RD0=0;
    RB2=0;
    RB3=1;
    RC0=0;
    RC4=0;
    Lcd_Init();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("System is OFF");
    T1CON=0x10;//Initialize the timer
    if(RC4==1)
    {
      Lcd_Clear();
      Lcd_Set_Cursor(1,1);
      Lcd_Write_String("Booting Up");
      __delay_ms(500);
      Lcd_Clear();  
    }
    while(1)
    {
        
        if(RC4==1)
        {
            __delay_ms(300);
            RB2=1;
            RB3=0;
            RD0=1;
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("ENGINE ON");
            __delay_ms(200);
            Lcd_Clear();
            a=ADC_GetAdcValue(0);
            b=ADC_GetAdcValue(1);
            pwmValue0=a*100/1023;
            pwmValue1=b*100/1023;
            PWM_SetDutyCycle(1,pwmValue0);
            PWM_SetDutyCycle(0,pwmValue1);
            TMR1H=0;
            TMR1L=0;
            RB0=1;
            __delay_us(10);
            RB0=0;
            TMR1ON=1;//Timer starts
            Lcd_Clear();
            while(RB1);
            TMR1ON=0;
            c=(TMR1L|(TMR1H<<8)); 
            c=c/58.82;
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("Altitude:");
            Lcd_Set_Cursor(2,14);
            Lcd_Write_Char(c%10+48);
            c=c/10;
            Lcd_Set_Cursor(2,13);
            Lcd_Write_Char(c%10+48);
            c=c/10;
            Lcd_Set_Cursor(2,12);
            Lcd_Write_Char(c%10+48); 
            Lcd_Set_Cursor(2,15);
            Lcd_Write_String("m");
        }
        else if(RC4==0)
        {
            RB2=0;
            RB3=1;
            RD0=0;
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("System is OFF");
        }
    }
}