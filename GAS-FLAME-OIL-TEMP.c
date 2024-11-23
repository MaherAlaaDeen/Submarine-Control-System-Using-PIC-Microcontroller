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
#include "adc.h"
#include "delay.h"
#include "lcd.h"
#include "stdutils.h"
#include "math.h"
void main()
{
    TRISA0=1;// LM35
    TRISA1=1;// GAS SENSOR
    TRISA2=1;//FLAME SENSOR
    TRISD=0;//LCD CONFIGURATION
    //TRISC=0;// LEDS
    RD0=0;//PUMP INITIALLY OFF
    TRISB2=1;// TURN SYSTEM ON/OFF
    TRISB3=0;// GREEN LED
    TRISB4=0;// RED LED
    TRISC0=0;
    TRISC1=0;
    TRISC2=0;
    TRISC3=0;
    TRISC4=0;//SPRINKLER
    RC4=0;
    RB3=0;// GREEN OFF
    RB4=1;// RED ON
    RD0=0;
    TRISB0=0;// TRIGGER
    TRISB1=1;//ECHO
    int a;
    Lcd_Init();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("SYSTEM OFF");
    ADC_Init();
    float b;
    int c;
    int temp;
    RC0=0;
    RC1=0;
    RC2=0;
    RC3=0;
    //char s[0];
    TRISB5=1;
    TRISB6=1;
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
            b=ADC_GetAdcValue(0);
            temp=((b*5)/1023)*100;
            c=round(temp);
            c=c-1;
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("Temp= ");
            //sprintf(s,"%f",temp);
            Lcd_Set_Cursor(1,8);
            //Lcd_Write_Char(s);
            Lcd_Write_Char(temp%10+48);
            temp=temp/10;
            Lcd_Set_Cursor(1,7);
            Lcd_Write_Char(temp%10+48);
            temp=temp/10;
            
            Lcd_Set_Cursor(1,10);
            Lcd_Write_String("C");
            __delay_ms(300);
            
            if(a>400)
            {
                
                RC0=1;
                RC1=1;
                RC2=1;
                RC3=1;
                
                RD0=0;
                __delay_us(300);
                Lcd_Clear(); 
            }
            if(a>300 && a<400)
            {
              
                RC0=1;
                RC1=1;
                RC2=1;
                RC3=0;
                //RD0=0;
                
                __delay_us(300);
            }
            if(a>200 && a<300)
            {
                RC0=1;
                RC1=1;
                RC2=0;
                RC3=0;
                //RD0=0;
                
                __delay_us(300);
            }
            if(a<100)
            {
                RC0=1;
                RC1=0;
                RC2=0;
                RC3=0;
                RD0=1;
                Lcd_Set_Cursor(2,1);
                Lcd_Write_String("Pump is ON");
                
                __delay_us(300);
            }
            if(RB5==1)
            {
                //Lcd_Clear();
                Lcd_Set_Cursor(2,1);
                Lcd_Write_String("GAS LEACKAGE");
                __delay_ms(500);
                
            }
            if(RB6==1)
            {
                //Lcd_Clear();
                Lcd_Set_Cursor(2,1);
                Lcd_Write_String("Fire is Detected");
                RC4=1;
                __delay_ms(500);
                
            }
            if(RB6==0)
            {
                RC4=0;
            }
        }
        else
        {
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("SYSTEM OFF");
            RB3=0;
            RB4=1;
            RD0=0;
        }
    }
    
          
    
}