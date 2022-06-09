/*
 * hd44780.c
 *
 *  Created on: 14 abr. 2022
 *      Author: Ricardo
 */

#include "hd44780.h"

/*************************** Máscara de pines ********************************/
/* Los pines RS, RW, EN y BL están ubicados en el Low Nibble del Byte de data
 * El High Nibble está compuesto por los pines D7-D4 respectivamente
 ******************************************************************************/
#define RS_CMD 0x00 /* 0000 0000 . 0 = Commands. 1 = Characters.*/
#define RS_CHR 0x01 /* 0000 0001 . 0 = Commands. 1 = Characters.*/
#define RW 0x02     /* 0000 0010 . 0 = Write. 1 = Read. ESTE PIN SIEMPRE SE DEJA EN 0 PORQUE NO SE LEE DESDE EL DISPLAY. */
#define ENABLE 0x04 /* 0000 0100 */
#define BL_ON 0x08  /* 0000 1000 . 0 = Off. 1 = On */
#define BL_OFF 0x00 /* 0000 0000 . 0 = Off. 1 = On */

/******************************** Comandos ***********************************/
/* La conexión entre el I2C y el LCD solo abarca 4 pines de datos
 * por lo que los comandos (8 bits de largo) deben ser enviados primero la parte alta (High Nibble).
 * y luego la parte baja (Low Nibble). Los primeros 4bits son los datos del comandos
 * y el resto son los pines de control.
 ******************************************************************************/
#define CLEAR_DISPLAY_HN 0x00 /* 0000 0000 */
#define CLEAR_DISPLAY_LN 0X10 /* 0001 0000 */

#define RETURN_HOME_HN 0x00 /* 0000 0000 */
#define RETURN_HOME_LN 0X30 /* 0011 0000 */

#define NO_DATA 0X00 /* 0000 0000 */

#define SET_CURSOR_HN 0X80       /* 1000 0000 */
#define FIRST_LINE_ADDRESS 0X00  /* 0000 0000 */
#define SECOND_LINE_ADDRESS 0X40 /* 0100 0000 */

/* Entry Mode Set */
/* Todos estos comandos comparten el mismo High Nibble. */
#define ENTRY_MODE_SET_HN 0X00           /* 0000 0000 */
#define ENTRY_MODE_SET_OFF_LN 0x40       /* 0100 0000 */
#define ENTRY_MODE_CURSOR_LN 0X20        /* 0010 0000. bit 1. 1 = Right, 0 = Left */
#define ENTRY_MODE_DISPLAY_SHIFT_LN 0X10 /* 0001 0000. bit 0. 1 =  Accompanies display shift. 0 = Off */

/* Display Control */
/* Todos estos comandos comparten el mismo High Nibble. */
#define DISPLAY_HN 0x00            /* 0000 0000 */
#define DISPLAY_OFF_LN 0X80        /* 1000 0000 */
#define DISPLAY_ON_LN 0X40         /* 0100 0000 */
#define CURSOR_ON_LN 0x20          /* 0010 0000 */
#define CURSOR_BLINKING_ON_LN 0x10 /* 0001 0000 */

/* Cursor/Display Shift */
/* Todos estos comandos comparten el mismo High Nibble. */
#define SHIFT_HN 0x10               /* 0001 0000 */
#define SHIFT_CURSOR_LEFT_LN 0X00   /* 0000 0000 */
#define SHIFT_CURSOR_RIGHT_LN 0X40  /* 0100 0000 */
#define SHIFT_DISPLAY_RIGHT_LN 0xC0 /* 1100 0000 */
#define SHIFT_DISPLAY_LEFT_LN 0x80  /* 1000 0000 */

/* Function Set */
/* Todos estos comandos comparten el mismo High Nibble. */
#define FUNCTION_SET_HN 0x20          /* 0010 0000. Configuracion de 4bits. */
#define FUNCTION_SET_2LINE_LN 0X80    /* 1000 0000. bit 7. 1 = 2 Line Display. 0 = 1 Line Display */
#define FUNCTION_SET_1LINE_LN 0X00    /* 0000 0000. bit 7. 1 = 2 Line Display. 0 = 1 Line Display */
#define FUNCTON_SET_5X10_CHAR_LN 0X40 /* 0100 0000. bit 6. 1 = 5x10 char set. 0 = 5x8 char set */
#define FUNCTON_SET_5X8_CHAR_LN 0X00  /* 0000 0000. bit 6. 1 = 5x10 char set. 0 = 5x8 char set */

/************************************************************************************************************
 *  Typedef para controlar el valor de la luz de fondo.
 ************************************************************************************************************/
typedef uint8_t backlight_t;

/************************************************************************************************************
 *  Typedef para seleccionar el registro.
 ************************************************************************************************************/
typedef uint8_t select_register_t;

/************************************************************************************************************
 *  Estructura donde se almacenan los nibbles del Function Set.
 ************************************************************************************************************/
typedef struct
{

        uint8_t hn;
        uint8_t ln;

} function_set_t;

/************************************************************************************************************
 *  Estructura donde se almacenan los nibbles del Entry Mode Set.
 ************************************************************************************************************/
typedef struct
{

        uint8_t hn;
        uint8_t ln;

} entry_mode_set_t;

/************************************************************************************************************
 *  Estructura donde se almacenan los nibbles del Display Control.
 ************************************************************************************************************/
typedef struct
{

        uint8_t hn;
        uint8_t ln;

} display_control_t;

/************************************************************************************************************
 *  Estructura donde se almacenan los nibbles del Cursor/Display Shift.
 ************************************************************************************************************/
typedef struct
{

        uint8_t hn;
        uint8_t ln;

} cursor_display_shift_t;

/************************************************************************************************************
 * Estructura interna al driver que no es visible al usuario.
 * Se utiliza para cargar las funciones definidas especificamente como port para
 * el hardware especifico.
 ************************************************************************************************************/
typedef struct
{

        hd44780_t hd44780_control;
        uint8_t data;
        backlight_t backlight;
        function_set_t function_set;
        cursor_display_shift_t cursor_display_shift;
        display_control_t display_control;
        select_register_t select_register;
        entry_mode_set_t entry_mode;

} driver_t;

static driver_t driver;

/************************************************************************************************************
 * @brief Función auxiliar que hace la secuencia de pulsación del pin de Enable.
 *
 * @param data Datos a enviar al LCD.
 *
 * @return None.
 ************************************************************************************************************/
static void _press_enable(uint8_t data)
{

        /* Habilito el bit de Enable sobre la data */
        data |= ENABLE;

        driver.hd44780_control.i2c_write(driver.hd44780_control.address, data);

        driver.hd44780_control.delay_us(450);

        /* Limpio el bit de Enable de la data */
        data &= ~ENABLE;

        driver.hd44780_control.i2c_write(driver.hd44780_control.address, data);

        driver.hd44780_control.delay_us(400);
}

/************************************************************************************************************
 * @brief Función auxiliar que envía el comando por I2C.
 *      Los comandos se arman de la siguiente manera:
 *              CMD | BL | RS
 *
 *      Los comandos están dividos en dos partes: High Nibble y Low Nibble.
 *      Esto es porque usando el I2C solo se pueden usar 4bits de datos del LCD.
 *      Por lo que primero hay que enviar la primera mitad del comando y luego la segunda mitad.
 *
 *      Los bytes a enviar son de la siguiente manera: D7 D6 D5 D4 BL EN RW RS
 *      La primera parte corresponde a los datos y la segunda a los estados de los pines.
 *
 *      Luego de enviar cada comando hay que "presionar" el pin de enable para que el comando sea tomado por el LCD.
 *
 * @param driver Instancia del driver que posee las funciones, comandos y estados a enviar.
 *
 * @return None.
 ************************************************************************************************************/
static void _write_command(void)
{

        driver.data = driver.data | driver.backlight | driver.select_register;

        driver.hd44780_control.i2c_write(driver.hd44780_control.address, driver.data);

        _press_enable(driver.data);
}

/************************************************************************************************************
 * @brief Función que inicializa el driver del LCD HD44780.
 *
 * @param config Estructura del driver a inicializar.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_init_driver(hd44780_t config)
{

        /* Cargo los valores en la estructura privada */
        driver.hd44780_control.delay_ms = config.delay_ms;
        driver.hd44780_control.delay_us = config.delay_us;
        driver.hd44780_control.i2c_init = config.i2c_init;
        driver.hd44780_control.i2c_write = config.i2c_write;
        driver.hd44780_control.n_rows = config.n_rows;
        driver.hd44780_control.address = config.address;

        /* Inicialización el I2C */
        driver.hd44780_control.i2c_init();

        /* Cargos los valores por defecto en la estructura */
        driver.backlight = BL_ON;
        driver.select_register = RS_CMD;

        driver.function_set.hn = FUNCTION_SET_HN;

        if (driver.hd44780_control.n_rows == 1)
        {
                driver.function_set.ln = FUNCTION_SET_1LINE_LN | FUNCTON_SET_5X10_CHAR_LN;
        }
        else
        {
                driver.hd44780_control.n_rows = 2;
                driver.function_set.ln = FUNCTION_SET_2LINE_LN | FUNCTON_SET_5X8_CHAR_LN;
        }

        driver.display_control.hn = DISPLAY_HN;
        driver.display_control.ln = DISPLAY_OFF_LN | DISPLAY_ON_LN | CURSOR_ON_LN;

        driver.entry_mode.hn = ENTRY_MODE_SET_HN;
        driver.entry_mode.ln = ENTRY_MODE_SET_OFF_LN | ENTRY_MODE_CURSOR_LN;

        driver.cursor_display_shift.hn = SHIFT_HN;

        /* Secuencia de inicialización del LCD para 4bits como se indica en el Datasheet */
        driver.hd44780_control.delay_ms(50);

        driver.data = 0x30;
        _write_command();
        driver.hd44780_control.delay_ms(10);
        _write_command();
        driver.hd44780_control.delay_ms(10);
        _write_command();
        driver.hd44780_control.delay_ms(10);
        driver.data = 0x20;
        _write_command();
        driver.hd44780_control.delay_ms(10);

        // Function Set.
        driver.data = driver.function_set.hn;
        _write_command();

        driver.data = driver.function_set.ln;
        _write_command();

        driver.hd44780_control.delay_ms(10);

        // Display Control.
        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
        driver.hd44780_control.delay_ms(10);

        // Clear Display.
        driver.data = CLEAR_DISPLAY_HN;
        _write_command();

        driver.data = CLEAR_DISPLAY_LN;
        _write_command();
        driver.hd44780_control.delay_ms(10);

        // Entry Mode Set.
        driver.data = driver.entry_mode.hn;
        _write_command();

        driver.data = driver.entry_mode.ln;
        _write_command();
        driver.hd44780_control.delay_ms(10);
}

/************************************************************************************************************
 * @brief Función que escribe un caracater en el LCD.
 *
 * @param character caracter a imprimir.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_print_char(const char character)
{

        driver.select_register = RS_CHR;

        uint8_t data = (uint8_t)character;
        /* Escribo primero el high nibble del caracter y luego el low nibble */
        driver.data = data & 0xF0;
        _write_command();
        driver.data = (data << 4);
        _write_command();
        driver.select_register &= ~RS_CHR;
}

/************************************************************************************************************
 * @brief Función que una cadena de caracteres en el LCD.
 *
 * @param string cadena a imprimir.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_print_string(const char *string)
{

        while (*string)
        {
                hd44780_print_char(*string++);
        }
}

/************************************************************************************************************
 * @brief Función que limpia el LCD
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_clear_screen(void)
{

        driver.data = CLEAR_DISPLAY_HN;
        _write_command();

        driver.data = CLEAR_DISPLAY_LN;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que enciende el parpadeo del cursor.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_cursor_blink_on(void)
{

        driver.display_control.ln |= CURSOR_BLINKING_ON_LN;

        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que apaga el parpadeo del cursor.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_cursor_blink_off(void)
{

        driver.display_control.ln &= ~CURSOR_BLINKING_ON_LN;

        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que enciende el cursor.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_cursor_on(void)
{

        driver.display_control.ln |= CURSOR_ON_LN;

        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que apaga el cursor.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_cursor_off(void)
{

        driver.display_control.ln &= ~CURSOR_ON_LN;

        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que enciende el display.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_display_on(void)
{

        driver.display_control.ln |= DISPLAY_ON_LN;

        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que apaga el display.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_display_off(void)
{

        driver.display_control.ln &= ~DISPLAY_ON_LN;

        driver.data = driver.display_control.hn;
        _write_command();

        driver.data = driver.display_control.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que enciende la luz de fondo del display.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_backlight_on(void)
{

        driver.backlight = BL_ON;

        driver.data = NO_DATA;

        _write_command();
}

/************************************************************************************************************
 * @brief Función que apaga la luz de fondo del display.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_backlight_off(void)
{

        driver.backlight = BL_OFF;

        driver.data = NO_DATA;

        _write_command();
}

/************************************************************************************************************
 * @brief Función que devuelve el cursor y el display a la posicion inicial.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_return_home(void)
{

        driver.data = RETURN_HOME_HN;
        _write_command();

        driver.data = RETURN_HOME_LN;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que mueve el display hacia la derecha.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_display_shift_right(void)
{

        driver.cursor_display_shift.ln = SHIFT_DISPLAY_RIGHT_LN;

        driver.data = driver.cursor_display_shift.hn;
        _write_command();

        driver.data = driver.cursor_display_shift.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que mueve el display hacia la izquierda.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_display_shift_left(void)
{

        driver.cursor_display_shift.ln = SHIFT_DISPLAY_LEFT_LN;

        driver.data = driver.cursor_display_shift.hn;
        _write_command();

        driver.data = driver.cursor_display_shift.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que mueve el cursor una posición hacia la derecha.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_cursor_shift_right(void)
{

        driver.cursor_display_shift.ln = SHIFT_CURSOR_RIGHT_LN;

        driver.data = driver.cursor_display_shift.hn;
        _write_command();

        driver.data = driver.cursor_display_shift.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que mueve el cursor una posición hacia la izquierda.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_cursor_shift_left(void)
{

        driver.cursor_display_shift.ln = SHIFT_CURSOR_LEFT_LN;

        driver.data = driver.cursor_display_shift.hn;
        _write_command();

        driver.data = driver.cursor_display_shift.ln;
        _write_command();
}

/************************************************************************************************************
 * @brief Función que posiciona el cursor.
 *
 *      En la columna 1 se tiene la posicion 1 a 40, y en la columna 2 las posiciones 41 a 80.
 *
 * @return None.
 ************************************************************************************************************/
void hd44780_set_cursor(uint8_t column, uint8_t row)
{

        if (row > 39)
        {
                row = 39;
        }

        if (row < 0)
        {
                row = 0;
        }

        uint8_t pos;

        if (driver.hd44780_control.n_rows == 2)
        {

                if (column > 2)
                {
                        column = 2;
                }

                (column == 2) ? (pos = SECOND_LINE_ADDRESS + row) : (pos = FIRST_LINE_ADDRESS + row);
        }
        else
        {

                pos = FIRST_LINE_ADDRESS + row;
        }

        driver.data = SET_CURSOR_HN | (pos & 0x70);
        _write_command();

        driver.data = (pos << 4) & 0xF0;
        _write_command();
}
