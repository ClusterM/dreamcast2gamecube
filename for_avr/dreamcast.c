#include "defines.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define PIN1_HI PORTB |= 1
#define PIN1_LOW PORTB &= ~1
#define PIN5_HI PORTB |= (1<<1)
#define PIN5_LOW PORTB &= ~(1<<1)

#define RESET_HI PORTA |= 1
#define RESET_LOW PORTA &= ~1
#define DC_CLOCK (!((PIND>>2)&1))

#define GREEN_ON PORTA |= (1<<1)
#define GREEN_OFF PORTA &= ~(1<<1)
#define RED_ON PORTA |= (1<<2)
#define RED_OFF PORTA &= ~(1<<2)

#define WAIT(t) {TCNT0=0; while(TCNT0 < (F_CPU / 1000000UL) * t);}
#define GC_SEND(t) {DDRB |= (1<<2); WAIT(t); DDRB &= ~(1<<2);}
#define GC_SEND_1 {GC_SEND(1); WAIT(3);}
#define GC_SEND_0 {GC_SEND(3); WAIT(1);}
#define GC_SEND_STOP {GC_SEND(1); WAIT(2);}
#define GC_SIGNAL (!((PINB>>2)&1))

volatile uint8_t dc_recv_buffer[256];
volatile uint16_t dc_recv_count = 0;

ISR(USART_RXC_vect) // для отладки
{
	unsigned char b;
	while (UCSRA & (1<<RXC))
	{		
		b = UDR;
	}	
}

void dc_data_out()
{
	DDRB |= (1<<0);
	DDRB |= (1<<1);
}

void dc_data_in()
{
	DDRB &= ~(1<<0);
	DDRB &= ~(1<<1);
}

void dc_send_init()
{
	PIN1_HI;
	PIN5_HI;
	_delay_us(1);
	
	PIN1_LOW;
	_delay_us(1);	
	PIN5_LOW;
	_delay_us(1);
	PIN5_HI;
	_delay_us(1);
	PIN5_LOW;
	_delay_us(1);
	PIN5_HI;
	_delay_us(1);
	PIN5_LOW;
	_delay_us(1);
	PIN5_HI;
	_delay_us(1);
	PIN5_LOW;
	_delay_us(1);
	PIN5_HI;
	_delay_us(1);
	PIN1_HI;
	_delay_us(1);
	PIN5_LOW;	
}

void dc_send_byte(uint8_t data)
{
	int b;
	int phase = 0;
	for (b = 7; b >= 0; b--)
	{
		if (!phase)
		{
			if (data & (1<<b))
			{
				PIN5_HI;
			}
			_delay_us(1);
			PIN1_LOW;
			_delay_us(1);
			PIN5_HI;
		} else {
			if (data & (1<<b))
			{
				PIN1_HI;
			}
			_delay_us(1);
			PIN5_LOW;
			_delay_us(1);
			PIN1_HI;
		}
		phase = !phase;
	}
}

void dc_terminate()
{
	PIN5_HI;
	_delay_us(1);
	PIN5_LOW;
	_delay_us(1);
	PIN1_LOW;
	_delay_us(1);
	PIN1_HI;
	_delay_us(1);
	PIN1_LOW;
	_delay_us(1);
	PIN1_HI;
	_delay_us(1);
	PIN5_HI;
}

void dc_send_data(uint8_t* data, int len)
{
	int i;
	uint8_t crc = 0;
	dc_data_out();
	dc_send_init();
	for (i = 0; i < len; i++)
	{
		dc_send_byte(*data);
		crc ^= *data;
		data++;
	}
	dc_send_byte(crc);
	dc_terminate();
	dc_data_in();
}

uint8_t gc_recv_buffer[64];

uint8_t gc_read(uint8_t* data, int max_len)
{
	int b;
	int bit = 0;
	do
	{
		TCNT0 = 0;
		while (GC_SIGNAL);
		gc_recv_buffer[bit++] = TCNT0;
		TCNT0 = 0;
		while (!GC_SIGNAL) if (TCNT0 >= 0x60) break;		
	} while (TCNT0 < 0x60);

	uint8_t gc_received = bit / 8;
	for (b = 0; b < gc_received; b++)
	{
		data[b] = 0;
		for (bit = 0; bit < 8; bit++)
		{	
			data[b] = data[b]<<1;
			if (gc_recv_buffer[b*8+bit] < 0x12) data[b] |= 1;
		}
	}
	return gc_received;
}

void gc_convert_data(uint8_t* src_data, int len, uint8_t* dst_data)
{
	int i, bit;
	for (i = 0; i < len; i++)
	{
		uint8_t d = *src_data;
		for (bit = 0; bit < 8; bit++)
		{
			if (d&0x80)
			{
				*dst_data = 1;
			} else {
				*dst_data = 0;
			}
			d <<= 1;
			dst_data++;
		}
		src_data++;
	}
}

void gc_send_data(uint8_t* data, int len)
{
	do
	{
		DDRB |= (1<<2);
		TCNT0=0;
		len--;
		if (*data)
		{
			// 1
			data++;
			while(TCNT0 < (F_CPU / 1000000UL) / 2);
			DDRB &= ~(1<<2);
			TCNT0 = 0;
			while(TCNT0 < (F_CPU / 1000000UL) * 3 * 38 / 36);
		} else {
			// 0
			data++; 
			while(TCNT0 < (F_CPU / 1000000UL) * 3 * 38 / 36);
			DDRB &= ~(1<<2);
			TCNT0 = 0;
			while(TCNT0 < (F_CPU / 1000000UL) / 2);
		}
	} while (len);	
	GC_SEND_STOP;
}

uint8_t gc_data[0x10];
uint8_t gc_init_answer[] = {0x09, 0x00, 0x20};
uint8_t gc_init_answer_raw[sizeof(gc_init_answer)*8];
uint8_t gc_origins[] = { 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00 };
uint8_t gc_origins_raw[sizeof(gc_origins)*8];
uint8_t gc_buttons_status[] = {
	0b00000000,
	0b10000000,
	0x80, 0x80, 0x80, 0x80,
	0x1F, 0x1F,
};
uint8_t gc_buttons_status_raw[80];

//ISR(INT2_vect)
void gc_proceed()
{
	uint8_t gc_received = gc_read(gc_data, sizeof(gc_data));
	if (gc_received == 1 && gc_data[0] == 0)
	{
		gc_send_data(gc_init_answer_raw, sizeof(gc_init_answer_raw));
	}	
	else if (gc_received == 1 && gc_data[0] == 0x41)
	{
		gc_send_data(gc_origins_raw, sizeof(gc_origins_raw));
	} 		
	else if (gc_received >= 3 && gc_data[0] == 0x40)
	{
		gc_send_data(gc_buttons_status_raw, 64);
		RED_ON;
	}
	else if (gc_received >= 3 && gc_data[0] == 0x42)
	{
		gc_send_data(gc_buttons_status_raw, 80);
		RED_ON;
	}	
	/*
	else {
		int i;
		USART_TransmitByte('!');
		for (i = 0; i < gc_received; i++)
			USART_TransmitByte(gc_data[i]);
	}*/
	
	if (gc_received < 3 || (gc_data[0] != 0x40 && gc_data[0] != 0x42)) RED_OFF;
}

void dc_read()
{
	dc_recv_count = 0;
	RESET_HI;
	TCNT1 = 0;
	while (1)
	{
		// ОМГ, я со школы не использовал goto!
		while (!DC_CLOCK) if (TCNT1 >= 30) goto dc_read_done;
		if ((dc_recv_count < sizeof(dc_recv_buffer)) /*&& (dc_recv_count == 0 || (dc_recv_count < dc_recv_buffer[0]*4+5))*/)
			dc_recv_buffer[dc_recv_count++] = PINC;
		if (dc_recv_count >= dc_recv_buffer[0]*4+5) break;
		while (DC_CLOCK) if (TCNT1 >= 30) goto dc_read_done;
	}
	dc_read_done:
	RESET_LOW;
}

void dreamcast_query()
{
	static uint8_t controller_addr = 0;

	if (!controller_addr)
	{
		uint8_t init_command[4] = {0x00, 0x00, 1<<5, 0x01};
		RESET_LOW;
		dc_send_data(init_command,sizeof(init_command));
		dc_read();
		if (dc_recv_count < 4 || dc_recv_buffer[3] != 5)
		{
			GREEN_OFF;
			GICR &= ~(1<<INT0);
			return;
		}
		controller_addr = dc_recv_buffer[1];
		_delay_ms(3);
	}
	
	uint8_t query_command[8] = {0x01, 0x00, controller_addr, 0x09, 0x01, 0x00, 0x00, 0x00};
	RESET_LOW;
	dc_send_data(query_command,sizeof(query_command));
	dc_read();
	
	int i;
	uint8_t crc = 0;
	for (i = 0; i < dc_recv_count; i++)
	{
		crc ^= dc_recv_buffer[i];
	}
	
	if (dc_recv_count > 0 && crc == 0)
	{
		GREEN_ON;
		static int start_time = 0;
		static int mode_time = 0;
		static uint8_t mode = 0xFF;
		static uint8_t start_locked = 0;
		// Читаем режим из энергонезависимой памяти, если надо		
		if (mode == 0xFF) mode = eeprom_read_byte((void*)0);
		mode = mode % 4;
		
		if ((dc_recv_buffer[11] & 0x08) == 0) // Start
		{
			if (start_time >= 60 && !start_locked)
			{				
				gc_buttons_status[0] |= (1 << 4);
			}			
			else if (dc_recv_buffer[11] == 0xF7 // Если Start долго удерживается, и другие кнопки не нажаты
				&& dc_recv_buffer[10] == 0xFF && dc_recv_buffer[8] == 0x00 
				&& dc_recv_buffer[9] == 0x00 && dc_recv_buffer[15] >= 0x78 && dc_recv_buffer[14] >= 0x78
				&& dc_recv_buffer[15] <= 0x88 && dc_recv_buffer[14] <= 0x88) 
				start_time++;
			else start_locked = 1; // А если что-то нажато, лочим start
		} else {
			start_time = 0;
			start_locked = 0;
			gc_buttons_status[0] &= ~(1 << 4);
		}
		
		// Комбинация кнопок для смены режима
		if (start_time && dc_recv_buffer[8]==0xFF)
		{
			// Надо не просто нажать, а подержать
			if (mode_time < 90)
			{
				mode_time++;
			} else {
				// Меняем режим на следующий
				mode = (mode+1)%4;
				// Сохраняем режим в EEPROM
				eeprom_write_byte((void*)0, mode);
				// Показываем это зелёным светодиодом
				GREEN_OFF;
				_delay_ms(500);
				// Мигаем светодиодом столько раз, сколько номер режима :)
				int i;
				for (i = 0; i <= mode; i++)
				{
					GREEN_ON;
					_delay_ms(200);
					GREEN_OFF;
					_delay_ms(200);
				}
				_delay_ms(500);
				mode_time = 0;
			}
		} else mode_time = 0;

		// Обнуляем положение аналога
		gc_buttons_status[2] = 0x80;
		gc_buttons_status[3] = 0x80;

		// Обнуляем положение C-стика
		gc_buttons_status[4] = 0x80;
		gc_buttons_status[5] = 0x80;		

		if (mode != 1 || !start_time)
		{
			if ((dc_recv_buffer[11] & 0x04) == 0) gc_buttons_status[0] |= (1 << 0); else gc_buttons_status[0] &= ~(1 << 0); // A -> A
			if ((dc_recv_buffer[10] & 0x04) == 0) gc_buttons_status[0] |= (1 << 1); else gc_buttons_status[0] &= ~(1 << 1); // X -> B
			if ((dc_recv_buffer[11] & 0x02) == 0) gc_buttons_status[0] |= (1 << 2); else gc_buttons_status[0] &= ~(1 << 2); // B -> X
			if ((dc_recv_buffer[10] & 0x02) == 0) gc_buttons_status[0] |= (1 << 3); else gc_buttons_status[0] &= ~(1 << 3); // Y -> Y
		} else { // В этом режиме кнопки заменяют C-Stick
			if ((dc_recv_buffer[10] & 0x04) == 0) gc_buttons_status[4] = 0; // X -> Left
			if ((dc_recv_buffer[11] & 0x02) == 0) gc_buttons_status[4] = 0xFF; // B -> Right
			if ((dc_recv_buffer[10] & 0x02) == 0) gc_buttons_status[5] = 0xFF; // Y -> Up
			if ((dc_recv_buffer[11] & 0x04) == 0) gc_buttons_status[5] = 0x00; // A -> Down
			// Кнопки отжаты при этом
			gc_buttons_status[0] &= ~(1 << 0); // A -> A
			gc_buttons_status[0] &= ~(1 << 1); // X -> B
			gc_buttons_status[0] &= ~(1 << 2); // B -> X
			gc_buttons_status[0] &= ~(1 << 3); // Y -> Y
		}
		
		if (!((mode == 0 && start_time) || (mode == 3 && !start_time)))
		{
			if ((dc_recv_buffer[11] & 0x40) == 0) gc_buttons_status[1] |= (1 << 0); else gc_buttons_status[1] &= ~(1 << 0); // D-Pad Left
			if ((dc_recv_buffer[11] & 0x80) == 0) gc_buttons_status[1] |= (1 << 1); else gc_buttons_status[1] &= ~(1 << 1); // D-Pad Right
			if ((dc_recv_buffer[11] & 0x10) == 0) gc_buttons_status[1] |= (1 << 3); else gc_buttons_status[1] &= ~(1 << 3); // D-Pad Up
			if ((dc_recv_buffer[11] & 0x20) == 0) gc_buttons_status[1] |= (1 << 2); else gc_buttons_status[1] &= ~(1 << 2); // D-Pad Down
		} else { // В этом режиме крестовина заменяет C-Stick
			if ((dc_recv_buffer[11] & 0x40) == 0) gc_buttons_status[4] = 0; // C-stick X Left
			if ((dc_recv_buffer[11] & 0x80) == 0) gc_buttons_status[4] = 0xFF; // C-stick X Right
			if ((dc_recv_buffer[11] & 0x10) == 0) gc_buttons_status[5] = 0xFF; // C-stick Y Up
			if ((dc_recv_buffer[11] & 0x20) == 0) gc_buttons_status[5] = 0; // C-stick Y Down
			// D-Pad отжат
			gc_buttons_status[1] &= ~(1 << 0);
			gc_buttons_status[1] &= ~(1 << 1);
			gc_buttons_status[1] &= ~(1 << 3);
			gc_buttons_status[1] &= ~(1 << 2);
		}		
		
		if (mode != 2 || !start_time)
		{
			gc_buttons_status[2] = dc_recv_buffer[15]; // JoyX
			gc_buttons_status[3] = 0xFF-dc_recv_buffer[14]; // JoyY
		} else { // В этом режиме аналог заменяет C-Stick
			gc_buttons_status[4] = dc_recv_buffer[15]; // C-stick X Left
			gc_buttons_status[5] = 0xFF-dc_recv_buffer[14]; // C-stick Y Up
		}		
		
		gc_buttons_status[6] = dc_recv_buffer[8]; // L
		if (!start_time)
			gc_buttons_status[7] = dc_recv_buffer[9]; // R
		else gc_buttons_status[7] = 0x00;
		if (dc_recv_buffer[8] >= 0xF8) gc_buttons_status[1] |= (1 << 6); else gc_buttons_status[1] &= ~(1 << 6); // L-button
		if (!start_time)
		{
			if (dc_recv_buffer[9] >= 0xF8) gc_buttons_status[1] |= (1 << 5); else gc_buttons_status[1] &= ~(1 << 5); // R-button
			gc_buttons_status[1] &= ~(1 << 4);
		} else {
			if (dc_recv_buffer[9] >= 0xF8) gc_buttons_status[1] |= (1 << 4); else gc_buttons_status[1] &= ~(1 << 4); // Z-button
			gc_buttons_status[1] &= ~(1 << 5);
		}
		
		gc_convert_data(gc_buttons_status, sizeof(gc_buttons_status), gc_buttons_status_raw);	
	} else {
		GREEN_OFF;
		controller_addr = 0;
	}
}

int main (void)
{
	dc_data_in(); // DC-data на ввод

	DDRB &= ~(1<<2); // GC-data на ввод
	PORTB &= ~(1<<2); // GC-data без подтяжки
	
	DDRA |= 1; // reset на выход
	DDRD &= ~(1<<2); // clock на вход
	PORTD |= (1<<2); // подтяжка clock
	
	DDRC = 0; // весь порт C на ввод
	PORTC = 0; // подтяжка для него не нужна... наверное
	
	DDRA |= (1<<1) | (1<<2); // светодиоды на вывод
	
	MCUCR = (1<<ISC01);
	MCUCSR &= ~(1<<ISC2);
	
	TCCR0 |= _BV(CS00); // Timer 
	TCCR1A |= (1<<WGM10)|(1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS12)|(1<<CS10);
	OCR1A = 1000;

/*	
	USART_init();
	USART_TransmitByte('!');
	USART_TransmitByte(MCUCSR);
	MCUCSR = 0;
*/

	gc_convert_data(gc_init_answer, sizeof(gc_init_answer), gc_init_answer_raw);
	gc_convert_data(gc_origins, sizeof(gc_origins), gc_origins_raw);	
	gc_convert_data(gc_buttons_status, sizeof(gc_buttons_status), gc_buttons_status_raw);		
	
	// Мигаем светодиодами при включении
	int i;
	for (i = 0; i < 3; i++)
	{
		GREEN_OFF;
		RED_ON;
		_delay_ms(100);
		RED_OFF;
		GREEN_ON;
		_delay_ms(100);
	}
	GREEN_OFF;
	_delay_ms(200);
	
//	sei();
	
	while (1)
	{
		// Ждём команды от куба
		while (!GC_SIGNAL) 
		{
			// Если её долго не было, опрашиваем контроллер дримкаста
			if (TCNT1 == 50)
			{
				dreamcast_query();
				TCNT1 = 51;
			}
			if (TCNT1 > 500) RED_OFF;
		}
		gc_proceed(); // Работаем с кубом
		TCNT1 = 0; // Обнуляем таймер
	}
	
	return 0;
}
