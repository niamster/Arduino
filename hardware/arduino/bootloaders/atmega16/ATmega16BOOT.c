/**********************************************************/
/* Serial Bootloader for Atmel mega8 AVR Controller       */
/*                                                        */
/* ATmega16BOOT.c                                         */
/*                                                        */
/* Copyright (c) 2003, Jason P. Kyle                      */
/* Copyright (c) 2011, D. Milinevskyy                     */
/*                                                        */
/* Hacked by DojoCorp - ZGZ - MMX - IVR                   */
/* Hacked by David A. Mellis                              */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http:// www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/* Target = Atmel AVR m16                                  */
/**********************************************************/

#include <inttypes.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>

#define MAX_TIME_COUNT F_CPU

/* SW_MAJOR and MINOR needs to be updated from time to time to avoid warning message from AVR Studio */
#define HW_VER	 0x02
#define SW_MAJOR 0x01
#define SW_MINOR 0x12

// AVR-GCC compiler compatibility
// avr-gcc compiler v3.1.x and older doesn't support outb() and inb()
//      if necessary, convert outb and inb to outp and inp
#ifndef outb
#define outb(sfr,val)  (_SFR_BYTE(sfr) = (val))
#endif
#ifndef inb
#define inb(sfr)        _SFR_BYTE(sfr)
#endif

/* defines for future compatibility */
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/* Adjust to suit whatever pin your hardware uses to enter the bootloader */
#define eeprom_rb(addr)         eeprom_read_byte((uint8_t *)(addr))
#define eeprom_rw(addr)         eeprom_read_word((uint16_t *)(addr))
#define eeprom_wb(addr, val)    eeprom_write_byte((uint8_t *)(addr), (uint8_t)(val))

/* Onboard LEDs */
#define LED_DDR  DDRA
#define LED_PORT PORTA
#define LED_PIN  PINA
#define LED0     PINA0
#define LED1     PINA1
#define LED2     PINA2
#define LED3     PINA3
#define LED4     PINA4
#define LED5     PINA5
#define LED6     PINA6
#define LED7     PINA7

#define SIG1	0x1E	// Yep, Atmel is the only manufacturer of AVR micros.  Single source :(
#define SIG2	0x94
#define SIG3	0x03

static void putch(uint8_t);
static uint8_t getch(void);
static void eat(uint8_t);

static void byte_response(uint8_t val);
static void stream_response(const uint8_t *val, uint8_t len);
#define nothing_response() stream_response(0, 0)

static void boot_program_page(uint32_t page, uint8_t *buf);

static void blink_led(uint8_t led);
static void launch_app(void);

#define IT_LEVEL        0
#define IT_RAISING_EDGE 1
#define IT_FALLING_EDGE 2

static void enable_interrupt(uint8_t it, uint8_t sence);
static void disable_interrupt(uint8_t it);

union address {
    uint16_t word;
    struct {
        uint8_t low;
        uint8_t high;
    } h;
};

union length {
    uint16_t word;
    struct {
        uint8_t low;
        uint8_t high;
    } h;
};

static uint8_t buff[256];
static const uint8_t sig[] = {SIG1, SIG2, SIG3};
static volatile uint8_t loop_forever = 0;

ISR(INT0_vect, ISR_BLOCK)
{
    disable_interrupt(0);

    loop_forever = 1;

    outb(LED_PORT, inb(LED_PORT) & ~_BV(LED1));
    sbi(LED_DDR, LED1);
}

ISR(INT1_vect, ISR_BLOCK)
{
    launch_app();
}

int main(void)
{
    uint8_t ch;
    uint8_t eeprom;
    union length length;
    union address address;

    /* initialize UART */
    UBRRH = (((F_CPU/BAUD_RATE)/16)-1)>>8; 	// set baud rate
    UBRRL = (((F_CPU/BAUD_RATE)/16)-1);
    UCSRB = (1<<RXEN)|(1<<TXEN);  // enable Rx & Tx
    UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // config USART; 8N1

    // Move interrupt vectors to the beginning of bootloader section
    GICR = 1<<IVCE;
    GICR = 1<<IVSEL;

    enable_interrupt(0, IT_RAISING_EDGE);
    enable_interrupt(1, IT_RAISING_EDGE);

    blink_led(LED0);

    sei();

    /* forever */
    for (;;) {
        /* get character from UART */
		ch = getch();

		/* A bunch of if...else if... gives smaller code than switch...case ! */

		/* Hello is anyone home ? */
		if (ch=='0') {
            nothing_response();
		}

		/* Request programmer ID */
		/* Not using PROGMEM string due to boot block in m128 being beyond 64kB boundry  */
		/* Would need to selectively manipulate RAMPZ, and it's only 9 characters anyway so who cares.  */
		else if (ch=='1') {
			if (getch() == ' ') {
				putch(0x14);
				putch('A');
				putch('V');
				putch('R');
				putch(' ');
				putch('I');
				putch('S');
				putch('P');
				putch(0x10);
            }
		}

		/* AVR ISP/STK500 board commands  DON'T CARE so default nothing_response */
		else if (ch=='@') {
            if (getch() > 0x85) getch();
            nothing_response();
		}

		/* AVR ISP/STK500 board requests */
		else if (ch=='A') {
            switch(getch()) {
                case 0x80:
                    byte_response(HW_VER);		// Hardware version
                    break;
                case 0x81:
                    byte_response(SW_MAJOR);	// Software major version
                    break;
                case 0x82:
                    byte_response(SW_MINOR);	// Software minor version
                    break;
                default:
                    byte_response(0x00);		// Covers various unnecessary responses we don't care about
            }
		}

		/* Device Parameters  DON'T CARE, DEVICE IS FIXED  */
		else if (ch=='B') {
            eat(20);
            nothing_response();
		}

		/* Parallel programming stuff  DON'T CARE  */
		else if (ch=='E') {
            eat(5);
            nothing_response();
		}

		/* Enter programming mode  */
		else if (ch=='P') {
            nothing_response();
		}

		/* Leave programming mode  */
		else if (ch=='Q') {
            nothing_response();

            launch_app();
		}

		/* Erase device, don't care as we will erase one page at a time anyway.  */
		else if (ch=='R') {
            nothing_response();
		}

		/* Universal SPI programming command, disabled.  Would be used for fuses and lock bits.  */
		else if (ch=='V') {
            eat(4);
            byte_response(0x00);
		}

		/* Set address, little endian. EEPROM in bytes, FLASH in words  */
		/* Perhaps extra address bytes may be added in future to support > 128kB FLASH.  */
		/* This might explain why little endian was used here, big endian used everywhere else.  */
		else if (ch=='U') {
            address.h.low = getch();
            address.h.high = getch();
            nothing_response();
		}

		/* Write memory, length is big endian and is in bytes  */
		else if (ch=='d') {
            uint8_t *b;
            uint16_t n;

            length.h.high = getch();
            length.h.low = getch();

            eeprom = 0;
            if (getch() == 'E')
                eeprom = 1;

            b = buff;
            n = length.word;
            memset(buff, 0xff, sizeof(buff));
            while (n--) {
                *b++ = getch(); // Store data in buffer, can't keep up with serial data stream whilst programming pages
            }

            b = buff;
            n = length.word;
            if (eeprom) { // Write to EEPROM one byte at a time
                while (n--) {
                    eeprom_wb(address.word++, *b++);
                }
            } else { // Write to FLASH one page at a time
                address.word = address.word << 1; // address * 2 -> byte location

                if (n<SPM_PAGESIZE)
                    n = SPM_PAGESIZE;

                while (n) {
#if 1
                    boot_program_page(address.word, b);
#else
                    asm volatile(
                                 "lds	r19,%1          \n" // Save status register and disable interrupts.
                                 "cli                   \n"

                                 "clr	r17             \n"	// word count

                                 "ldi	r18,0x03        \n"	// Erase page pointed to by Z ((1<<PGERS) | (1<<SPMEN))
                                 "rcall do_spm		    \n"

                                 "ldi	r18,0x11        \n"	// Re-enable RWW section ((1<<RWWSRE) | (1<<SPMEN))
                                 "rcall do_spm		    \n"

                                 "push:                 \n"
                                 "ld	r0,X+           \n"	// Write 2 bytes into page buffer
                                 "ld	r1,X+           \n"

                                 "ldi	r18,0x01        \n"	// Load r0,r1 into FLASH page buffer (1<<SPMEN)
                                 "rcall do_spm		    \n"

                                 "adiw	r30,2           \n"	// Next word in FLASH

                                 "inc	r17             \n"
                                 "cpi   r17,%2	        \n"
                                 "brlo	push            \n"	// Still same page in FLASH
                                 "rjmp	write           \n"

                                 "do_spm:               \n"
                                 "wait_spm:             \n"
                                 "lds	r16,%0          \n"	// Wait for previous spm to complete
                                 "sbrc	r16,0           \n"
                                 "rjmp	wait_spm        \n"
                                 "wait_ee:              \n" // Wait for EEPROM writes to complete
                                 "lds   r16,%3          \n"
                                 "sbrc  r16,1           \n"
                                 "rjmp wait_ee          \n"
                                 "sts	%0,r18          \n"
                                 "spm                   \n"
                                 "ret                   \n"

                                 "write:                \n"
                                 "mov	r30,%4          \n"	// Address of FLASH location (in words)
                                 "mov	r31,%5          \n"

                                 "ldi	r18,0x05        \n"	// Write page pointed to by Z ((1<<PGWRT) | (1<<SPMEN))
                                 "rcall do_spm		    \n"

                                 "ldi	r18,0x11        \n"	// Re-enable RWW section ((1<<RWWSRE) | (1<<SPMEN))
                                 "rcall do_spm		    \n"

                                 "sts	%1,r19          \n" // Re-enable interrupts (if they were ever enabled)

                                 "clr	__zero_reg__	\n"	// restore zero register

                                 : "=m"(SPMCR), "=m"(SREG) : "M"(SPM_PAGESIZE>>1), "m"(EECR), "r"(address.h.low), "r"(address.h.high), "z"(address.word), "x"(b) : "r0","r1","r16","r17","r18","r19");
#endif
                    b += SPM_PAGESIZE;
                    n -= SPM_PAGESIZE;
                    address.word += SPM_PAGESIZE;
                }
            }
            nothing_response();
		}

		/* Read memory block mode, length is big endian.  */
		else if (ch=='t') {
            uint8_t *b;
            uint16_t n;

            length.h.high = getch();
            length.h.low = getch();

            eeprom = 0;
            if (getch() == 'E')
                eeprom = 1;
            else
                address.word = address.word << 1; // address * 2 -> byte location in case of FLASH

            b = buff;
            n = length.word;
            while (n--) {
                if (eeprom)
                    *b = eeprom_rb(address.word);
                else
                    *b = pgm_read_byte_near(address.word);

                ++b;
                ++address.word;
            }

            stream_response(buff, length.word);
		}

		/* Get device signature bytes  */
		else if (ch=='u') {
            stream_response(sig, 3);
		}

		/* Read oscillator calibration byte */
		else if (ch=='v') {
            byte_response(0x00);
		}
	} /* end of forever loop */
}

static void putch(uint8_t ch)
{
    while (!(inb(UCSRA) & _BV(UDRE)));

    outb(UDR,ch);
}

static uint8_t getch(void)
{
    uint32_t count;

    for (;;) {
        count = MAX_TIME_COUNT;

        while (!(inb(UCSRA) & _BV(RXC)) && --count);

        if (!count) {
            if (!loop_forever)
                launch_app();
        } else {
            break;
        }
    }

    return inb(UDR);
}

static void eat(uint8_t count)
{
    while (count--)
		getch(); // need to handle time out
}

static void byte_response(uint8_t val)
{
    stream_response(&val, 1);
}

static void stream_response(const uint8_t *val, uint8_t len)
{
    if (getch() == ' ') {
        putch(0x14);
        while (len--)
            putch(*(val++));
        putch(0x10);
    }
}

static void boot_program_page(uint32_t page, uint8_t *buf)
{
    uint16_t i, w;
    uint8_t sreg;

    // Save status register and disable interrupts
    sreg = SREG;
    cli();

    eeprom_busy_wait();

    boot_page_erase(page);
    boot_spm_busy_wait();      // Wait until the memory is erased

    for (i=0; i<SPM_PAGESIZE; i+=2) {
        // Set up little-endian word
        w = *buf++;
        w |= (*buf++) << 8;

        boot_page_fill(page + i, w);
    }

    boot_page_write(page);     // Store buffer in flash page
    boot_spm_busy_wait();      // Wait until the memory is written

    // Reenable RWW-section again. We need this if we want to jump back
    // to the application after bootloading
    boot_rww_enable();

    // Re-enable interrupts (if they were ever enabled)
    SREG = sreg;
}

static void blink_led(uint8_t led)
{
    uint8_t i;

    /* blink the LED */
    outb(LED_PORT, inb(LED_PORT) | _BV(led));
    sbi(LED_DDR, led);
	for (i=0; i<16; ++i) {
		outb(LED_PORT, inb(LED_PORT) ^ _BV(led));
		_delay_loop_2(0);
	}
}

static void launch_app(void)
{
    void (*app)(void) = 0x0000;

    cli();

    // Move interrupt vectors to the beginning of FLASH and disable them
    GICR = 1<<IVCE;
    GICR = 0;

    blink_led(LED7);

    /* disable all LEDs */
    outb(LED_PORT, 0xFF);

    app();
}

/* Note: interrupt #2 for ATmega16 is not supported by this routine */
static void enable_interrupt(uint8_t it, uint8_t sence)
{
    uint8_t shift = it << 1;

    MCUCR &= ~(3<<shift);
    switch (sence) {
        case IT_LEVEL:
            /* nothing to do */
            break;
        case IT_RAISING_EDGE:
            MCUCR |= 3<<shift;
            break;
        case IT_FALLING_EDGE:
            MCUCR |= 2<<shift;
            break;
    }

    GICR |= 1 << (it+6);
}

/* Note: interrupt #2 for ATmega16 is not supported by this routine */
static void disable_interrupt(uint8_t it)
{
    GICR &= ~(1 << (it+6));
}

/* end of file ATmega16BOOT.c */
