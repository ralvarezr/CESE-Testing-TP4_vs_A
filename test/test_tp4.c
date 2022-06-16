#include "unity.h"
#include "hd44780.h"

static hd44780_t lcd;

static bool gb_transmision_correcta = false;

static uint8_t datos[200];
static size_t indice = 0;
static uint16_t direccion;
//************************** Funciones de mock *****************
// Estas funciones se deben pasar a la estructura hd44780_t para que funcione.
void i2c_init_test(void)
{
        return ;
}

void i2c_transmit_test(uint16_t address, uint8_t data)
{
        direccion = address;
        datos[indice++] = data;
}

void delayms_test(uint32_t ms)
{
        return ;
}

void delayus_test(uint32_t us)
{
        return ;
}

//Funcion que corre cuando ingresa a una funcion de test.
void setUp(void)
{
        lcd.i2c_init = i2c_init_test;
        lcd.i2c_write = i2c_transmit_test;
        lcd.delay_ms = delayms_test;
        lcd.delay_us = delayus_test;
        lcd.n_rows = 2;
        lcd.address = 0;

        hd44780_init_driver(lcd);

        indice = 0;
        gb_transmision_correcta = false;
}

//Se puede imprimir un caracteres en pantalla.
void test_PrintChar(void)
{
        uint8_t trama[6] = {
            0b01101001,
            0b01101101,
            0b01101001,
            0b00011001,
            0b00011101,
            0b00011001
        };

        hd44780_print_char('a');

        size_t i = 0;
        size_t cant_correctas = 0;
        for (; i < 6; i++)
        {
                if(trama[i] == datos[i])
                        cant_correctas++;
        }

        if(cant_correctas == i)
                gb_transmision_correcta = true;

        if(lcd.address != direccion)
                gb_transmision_correcta = false;

        if(indice != i)
                gb_transmision_correcta = false;

        TEST_ASSERT_TRUE(gb_transmision_correcta);

}


// Se puede imprimir una cadena caracteres en pantalla.
void test_PrintString(void)
{

        uint8_t trama[12] = {
            0b01101001,
            0b01101101,
            0b01101001,
            0b00011001,
            0b00011101,
            0b00011001,
            0b01001001,
            0b01001101,
            0b01001001,
            0b11001001,
            0b11001101,
            0b11001001
            };

        hd44780_print_string("aL");

        size_t i = 0;
        size_t cant_correctas = 0;
        for (; i < 12; i++)
        {
                if (trama[i] == datos[i])
                {
                        cant_correctas++;
                }
        }

        if (cant_correctas == i)
        {
                gb_transmision_correcta = true;
        }

        if (lcd.address != direccion)
        {
                gb_transmision_correcta = false;
        }

        if (indice != i)
        {
                gb_transmision_correcta = false;
        }

        TEST_ASSERT_TRUE(gb_transmision_correcta);
}


// Se puede hacer parpadear el cursor.
void test_CursorBlinkOn(void)
{
        uint8_t trama[6] = {
            0b00001000,
            0b00001100,
            0b00001000,
            0b11111000,
            0b11111100,
            0b11111000
            };

        hd44780_cursor_blink_on();

        size_t i = 0;
        size_t cant_correctas = 0;
        for (; i < 6; i++)
        {
                if (trama[i] == datos[i])
                {
                        cant_correctas++;
                }
        }

        if (cant_correctas == i)
        {
                gb_transmision_correcta = true;
        }

        if (lcd.address != direccion)
        {
                gb_transmision_correcta = false;
        }

        if (indice != i)
        {
                gb_transmision_correcta = false;
        }

        TEST_ASSERT_TRUE(gb_transmision_correcta);
}


// Se puede dejar de parpadear el cursor.
void test_CursorBlinkOff(void)
{

        uint8_t trama[6] = {
            0b00001000,
            0b00001100,
            0b00001000,
            0b11101000,
            0b11101100,
            0b11101000};

        hd44780_cursor_blink_off();

        size_t i = 0;
        size_t cant_correctas = 0;
        for (; i < 6; i++)
        {
                if (trama[i] == datos[i])
                {
                        cant_correctas++;
                }
        }

        if (cant_correctas == i)
        {
                gb_transmision_correcta = true;
        }

        if (lcd.address != direccion)
        {
                gb_transmision_correcta = false;
        }

        if (indice != i)
        {
                gb_transmision_correcta = false;
        }

        TEST_ASSERT_TRUE(gb_transmision_correcta);
}


// Se puede limpiar la pantalla.
void test_ClearScreen(void)
{

        uint8_t trama[6] = {
            0b00001000,
            0b00001100,
            0b00001000,
            0b00011000,
            0b00011100,
            0b00011000};

        hd44780_clear_screen();

        size_t i = 0;
        size_t cant_correctas = 0;
        for (; i < 6; i++)
        {
                if (trama[i] == datos[i])
                {
                        cant_correctas++;
                }
        }

        if (cant_correctas == i)
        {
                gb_transmision_correcta = true;
        }

        if (lcd.address != direccion)
        {
                gb_transmision_correcta = false;
        }

        if (indice != i)
        {
                gb_transmision_correcta = false;
        }

        TEST_ASSERT_TRUE(gb_transmision_correcta);
}
