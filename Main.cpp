#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#define DHT_BIT        PC0
#define FAN_ENA_BIT    PB2
#define FAN_IN1_BIT    PB1
#define FAN_IN2_BIT    PB0

#define TEMP_MIN       27
#define TEMP_MAX       35
#define PWM_MIN        100
#define PWM_MAX        255

#define READ_INTERVAL  2000UL
#define LCD_ADDR       0x27

static volatile uint32_t ms_ticks = 0;

ISR(TIMER0_COMPA_vect)
{
    ms_ticks++;
}

static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
    TCCR0B = (1 << CS01) | (1 << CS00);
}

static uint32_t millis(void)
{
    uint32_t m;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { m = ms_ticks; }
    return m;
}

static void timer2_init(void)
{
    TCCR2A = 0;
    TCCR2B = (1 << CS21);
}

static void pwm_init(void)
{
    DDRB |= (1 << FAN_ENA_BIT) | (1 << FAN_IN1_BIT) | (1 << FAN_IN2_BIT);
    PORTB &= ~((1 << FAN_ENA_BIT) | (1 << FAN_IN1_BIT) | (1 << FAN_IN2_BIT));

    TCCR1A = (1 << WGM10) | (1 << COM1B1);
    TCCR1B = (1 << CS11) | (1 << CS10);
    OCR1B  = 0;
}

static void set_fan_speed(uint8_t pwm)
{
    if (pwm > 0) {
        PORTB |=  (1 << FAN_IN1_BIT);
        PORTB &= ~(1 << FAN_IN2_BIT);
        OCR1B = pwm;
    } else {
        OCR1B = 0;
        PORTB &= ~((1 << FAN_IN1_BIT) | (1 << FAN_IN2_BIT));
    }
}

static void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0))) { }
    UDR0 = c;
}

static void uart_puts_P(const char *s)
{
    char c;
    while ((c = pgm_read_byte(s++))) uart_putc(c);
}

static bool wait_while(uint8_t level, uint8_t *dur)
{
    uint8_t t0 = TCNT2;
    while (((PINC & (1 << DHT_BIT)) != 0) == (level != 0)) {
        if ((uint8_t)(TCNT2 - t0) > 200) return false;
    }
    if (dur) *dur = (uint8_t)(TCNT2 - t0);
    return true;
}

static bool read_dht11(uint8_t *temperature, uint8_t *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t d;

    DDRC  |=  (1 << DHT_BIT);
    PORTC &= ~(1 << DHT_BIT);
    _delay_ms(18);
    PORTC |=  (1 << DHT_BIT);
    _delay_us(40);
    DDRC  &= ~(1 << DHT_BIT);
    PORTC |=  (1 << DHT_BIT);

    uint8_t sreg = SREG;
    cli();

    if (!wait_while(1, 0)) goto fail;
    if (!wait_while(0, 0)) goto fail;
    if (!wait_while(1, 0)) goto fail;

    for (uint8_t i = 0; i < 40; i++) {
        if (!wait_while(0, 0)) goto fail;
        if (!wait_while(1, &d)) goto fail;
        data[i / 8] <<= 1;
        if (d > 80) data[i / 8] |= 1;
    }
    SREG = sreg;

    if (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3])) return false;

    *humidity    = data[0];
    *temperature = data[2];
    return true;

fail:
    SREG = sreg;
    return false;
}

static void twi_init(void)
{
    TWSR = 0;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

static void twi_wait(void)
{
    while (!(TWCR & (1 << TWINT))) { }
}

static void twi_start(uint8_t addr_w)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    twi_wait();
    TWDR = addr_w;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
}

static void twi_write(uint8_t b)
{
    TWDR = b;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

static void pcf_write(uint8_t v)
{
    twi_start((LCD_ADDR << 1) | 0);
    twi_write(v);
    twi_stop();
}

static void lcd_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t v = (uint8_t)((nib << 4) | LCD_BL | rs);
    pcf_write(v | LCD_EN);
    _delay_us(1);
    pcf_write(v & ~LCD_EN);
    _delay_us(50);
}

static void lcd_send(uint8_t byte, uint8_t rs)
{
    lcd_nibble(byte >> 4, rs);
    lcd_nibble(byte & 0x0F, rs);
}

static void lcd_cmd(uint8_t c)  { lcd_send(c, 0); }
static void lcd_data(uint8_t c) { lcd_send(c, LCD_RS); }

static void lcd_init(void)
{
    _delay_ms(50);
    lcd_nibble(0x03, 0); _delay_ms(5);
    lcd_nibble(0x03, 0); _delay_ms(5);
    lcd_nibble(0x03, 0); _delay_us(150);
    lcd_nibble(0x02, 0);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01); _delay_ms(2);
}

static void lcd_clear(void) { lcd_cmd(0x01); _delay_ms(2); }

static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    lcd_cmd(0x80 | ((row ? 0x40 : 0x00) + col));
}

static void lcd_print(const char *s)
{
    while (*s) lcd_data((uint8_t)*s++);
}

static void lcd_print_uint(uint16_t v)
{
    char buf[6];
    utoa(v, buf, 10);
    lcd_print(buf);
}

static uint8_t temp_to_pwm(uint8_t t)
{
    if (t < TEMP_MIN) return 0;
    if (t >= TEMP_MAX) return PWM_MAX;
    return (uint8_t)(PWM_MIN +
           ((int16_t)(t - TEMP_MIN) * (PWM_MAX - PWM_MIN)) / (TEMP_MAX - TEMP_MIN));
}

int main(void)
{
    uint32_t last_read = 0;

    timer0_init();
    timer2_init();
    pwm_init();
    uart_init();
    twi_init();
    sei();

    lcd_init();
    lcd_set_cursor(0, 0); lcd_print("Temp Monitor");
    lcd_set_cursor(0, 1); lcd_print("PWM Mode Start");
    _delay_ms(1200);
    lcd_clear();

    set_fan_speed(0);

    for (;;) {
        if (millis() - last_read >= READ_INTERVAL) {
            last_read = millis();

            uint8_t t, h;
            if (!read_dht11(&t, &h)) {
                lcd_set_cursor(0, 0); lcd_print("Sensor error!   ");
                lcd_set_cursor(0, 1); lcd_print("Check wiring    ");
                uart_puts_P(PSTR("Failed to read from DHT11!\r\n"));
                continue;
            }

            uint8_t pwm = temp_to_pwm(t);
            set_fan_speed(pwm);

            uint8_t percent = (uint8_t)(((uint16_t)pwm * 100) / 255);

            lcd_set_cursor(0, 0);
            lcd_print("T: ");  lcd_print_uint(t);
            lcd_print("C  H: "); lcd_print_uint(h);
            lcd_print("%  ");

            lcd_set_cursor(0, 1);
            lcd_print("Fan: ");
            if (pwm == 0) {
                lcd_print("OFF       ");
            } else {
                lcd_print_uint(percent);
                lcd_print("%        ");
            }
        }
    }
}
