#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__ 1
#endif 


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

FILE usart_out;
FILE usart_in;

int usart_putchar(char c, FILE *stream) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

int usart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void init_usart_io (void) {

    #define BAUD 9600
    #include <util/setbaud.h>

    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 

    fdev_setup_stream(&usart_out, usart_putchar, NULL, _FDEV_SETUP_WRITE);
    fdev_setup_stream(&usart_in, NULL, usart_getchar, _FDEV_SETUP_READ);

    stdout = &usart_out;
    stdin  = &usart_in;
}

int is_button_pressed(void) {
    return ((PINB & (1 << PINB0)) == (1 << PINB0));
}


#define LCD_Dir  DDRD			/* Define LCD data port direction */
#define LCD_Port PORTD			/* Define LCD data port */
#define RS PORTD2				/* Define Register Select pin */
#define EN PORTD3 				/* Define Enable signal pin */

void LCD_Command( unsigned char cmnd ) {
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Port &= ~ (1<<RS);		/* RS=0, command reg. */
	LCD_Port |= (1<<EN);		/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}


void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Port |= (1<<RS);		/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Dir = 0xFF;			/* Make LCD port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x02);		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0c);              /* Display on cursor off*/
	LCD_Command(0x06);              /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              /* Clear display screen*/
	_delay_ms(2);
}


void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
	LCD_String(str);		/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);		/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);		/* Cursor at home position */
}
 
 /*
int main()
{


    init_usart_io();

	LCD_Init();			
    LCD_Clear();

	LCD_String("Hello World");	


    DDRB &= ~(1 << PINB0);

    int prev = 0;
    int toggle = 0;

    int started = 0;

    puts("Should I get a snack?");

    clock_t time = clock();
    clock_t prev_stop = time;
    clock_t last_toggle = -INFINITY;

    int i = 0;

    while (1) {

        int current = is_button_pressed();
        if (current != prev) {
            prev = current;
            if (current) {
                toggle = !toggle;
                if (!toggle) {
                    prev_stop = time;
                    if (time - last_toggle < 0.5) {
                        started = 0;
                        time = 0.0;
                        prev_stop = 0.0;
                        last_toggle = -1.0;
                    }
                }
                if (toggle) {
                    started = 1;
                    last_toggle = time;
                }
                printf("Give me a SNACK!");
                //LCD_Clear();
                //LCD_String("Give me a SNACK!");

            } else {
                printf("No SnAcK :(");
                //LCD_Clear();
                //LCD_String("No SnAcK :(");

            }
        }
        
        
        if (toggle) {
            if (i % 7 == 0) {
            LCD_Clear();
            char str[16] = {0};
            dtostrf(time, 5, 2, str);
            puts(str);
            LCD_String(str);
            } 
        } else {
            if (i % 7 == 0) {
            LCD_Clear();
            char str[16] = {0};
            dtostrf(prev_stop, 5, 2, str);
            puts(str);
            LCD_String(str);
            } 
        }

        if (started) {
            time += 10 / 1000.0;
        }
        _delay_ms(10);

        i += 1;
    }

    return 0;
    
}
*/

void bluetooth_set_device_name(const char* name) {
    printf("AT+NAME=%s\r\n", name);
}

int main() {
    init_usart_io();
    while(1) {
        printf("ping\n");
        _delay_ms(200);
    }
    return 0;
}