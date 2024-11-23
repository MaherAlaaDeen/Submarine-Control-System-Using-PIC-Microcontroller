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
    TRISA0=1;
    TRISC1=0;
    TRISB2=1;//TURN ON/OFF
    TRISB3=0;//GREEN LED
    TRISB4=0;//RED LED 
    TRISD=0;
    int a;
    TRISB0=0;//trigger
    TRISB1=1;//echo
    Lcd_Init();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("RADAR OFF");
    RB3=0;
    RB4=1;
    RC1=0;
    T1CON=0x10;
    while(1)
    {
        if(RB2==1)
        {
            RB3=1;
            RB4=0;
            TMR1H=0;
            TMR1L=0;
            RB0=1;
            __delay_us(10);
            RB0=0;
            TMR1ON=1;//Timer starts
            Lcd_Clear();
            while(RB1);
            TMR1ON=0;
            a=(TMR1L|(TMR1H<<8));
            a=a/58.82;
            RC1=1;
            if(a>=2 && a<=400)
            {
                Lcd_Set_Cursor(1,1);
                Lcd_Write_String("Distance= ");
                Lcd_Set_Cursor(1,14);
                Lcd_Write_Char(a%10+48);
                a=a/10;
                Lcd_Set_Cursor(1,13);
                Lcd_Write_Char(a%10+48);
                a=a/10;
                Lcd_Set_Cursor(1,12);
                Lcd_Write_Char(a%10+48); 
                Lcd_Set_Cursor(1,15);
                Lcd_Write_String("cm");
                __delay_ms(500);
            }
            else
            {
                Lcd_Clear();
                Lcd_Write_String(" Out of range");
                __delay_ms(400);
            }

    }
        else
        {
            RB3=0;
            RB4=1;
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("System OFF");
            __delay_ms(300);
            Lcd_Clear();
        }
 }
}