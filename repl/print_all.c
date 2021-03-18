/**
 * @file Print_All.c
 * @author Arjan Kamphuis
 * @copyright 2020 AUTOMATIEK - Arjan Kamphuis
 * @date 13-10-2009
 * @brief Sends ASCII strings to output and converts values.
 *  \startuml{myimage.png} "Image Caption" width=5cm
 *    Sender->Receiver  : Command()
 *    Sender<--Receiver : Ack()
 *  \enduml
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ----------------------------------------------------------------- */
#include <stdarg.h>
#include "FixedPoint.h"

extern void ei_putchar(char c);
typedef void (* output_buffer_fptr)(char byte);

/* Local globals ------------------------------------------------------------ */
const signed int dec_maskArray[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
static char *write_buffer;

/* Static function prototypes ----------------------------------------------- */
static void SetOutputBuffer(char *outputbuffer);
static void WriteOutputBuffer(char byte);
static int HexadecimalConverter(void (* output)(char byte), signed long hex_value, unsigned int zero, unsigned int space);
static int SignPosition(signed int value, unsigned int zero);
static int DecimalConverter(void (* output)(char byte), signed long dec_value, unsigned int zero, unsigned int space, int forceNeg, int useSign);
static int ExtractFixedPoint(signed long fixedVal, signed long *abspart, unsigned long *fractionpart);

/**
 * @brief      Convert variable argument parameters to ascii characters and
 * send to output
 * Use %F for displaying fixed point in decimal format
 * <br>Some functions are not implemented:<br>
 * <ul><li>%i, %g, %e, %o, %p/li>
 * <li>number of characters (in %d) are limited to 9 digits</li></ul>
 * @param[in]  txt        The text string
 * @param[in]  ...            argument list (uses stdarg.h)
 */
void print_out(const char *txt, va_list ap)
{
    //va_list ap;
    int i, intVal;
    int useSign;
    unsigned int leadingZero, leadingSpace;

    //va_start(ap, txt);

    for (i = 0; txt[i] != '\0'; i++) /* Walk through text */
    {                                /* Clear specifiers */
        leadingZero = leadingSpace = useSign = 0;

        if (txt[i] == '%') /* % -> input from arg list */
        {
            while(txt[i+1] != '\0') /* Walk through all args of % */
            {
                /* Decimal representation */
                if (txt[i + 1] == 'd' || txt[i + 1] == 'l' || txt[i + 1] == 'h' || txt[i + 1] == 'u' ) {
                    intVal = va_arg(ap, int);

                    DecimalConverter(&ei_putchar, intVal, leadingZero, leadingSpace, 0, useSign);
                    i++;

                    if (txt[i + 1] == 'u' || txt[i + 1] == 'd') i++;
                    break;
                }

                /* Hexadecimal representation */
                else if (txt[i + 1] == 'X' || txt[i + 1] == 'x') {
                    intVal = va_arg(ap, signed long);

                    HexadecimalConverter(&ei_putchar, intVal, leadingZero, leadingSpace);

                    i++;
                    break;
                }

                /* Float representation */
                else if (txt[i + 1] == 'f') {
                    unsigned long fraction;
                    signed long absolute;
                    int forceNeg;

                    float floatVal = va_arg(ap, double);

                    intVal = floatVal * (1 << FP_FRACTION);

                    forceNeg = ExtractFixedPoint(intVal, &absolute, &fraction);

                    DecimalConverter(&ei_putchar, absolute, 0, 3, forceNeg, useSign);
                    ei_putchar('.');
                    DecimalConverter(&ei_putchar, fraction, 3, 0, 0, 0);

                    i++;
                    break;
                }

                /* Fixed Point representation */
                else if (txt[i + 1] == 'F') {
                    unsigned long fraction;
                    signed long absolute;
                    int forceNeg;
                    intVal = va_arg(ap, int);

                    forceNeg = ExtractFixedPoint(intVal, &absolute, &fraction);

                    DecimalConverter(&ei_putchar, absolute, 0, 3, forceNeg, useSign);
                    ei_putchar('.');
                    DecimalConverter(&ei_putchar, fraction, 3, 0, 0, 0);

                    i++;
                    break;
                }

                /* 1 Char representation */
                else if (txt[i + 1] == 'c') {
                    ei_putchar((char)va_arg(ap, int));
                    i++;
                    break;
                }

                /* Print percent sign */
                else if (txt[i + 1] == '%') {
                    ei_putchar(txt[i + 1]);
                    i++;
                    break;
                }

                /* String representation */
                else if (txt[i + 1] == 's') {
                    char *strptr = (char *)va_arg(ap, int *);
                    /* Use EOF !!!                   */
                    while (*(strptr) != '\0') ei_putchar(*(strptr++));
                    i++;
                    break;
                }

                /* Use Leading zeroes */
                else if (txt[i + 1] == '.') {
                    leadingZero = (txt[i + 2] - 0x31);
                    i += 2;
                }

                /* Use N leading spaces */
                else if (txt[i + 1] > 0x30 && txt[i + 1] < 0x3A) {
                    leadingSpace = (txt[i + 1] - 0x30);
                    i++;
                    if (txt[i + 1] >= 0x30 && txt[i + 1] < 0x3A) {
                        leadingSpace *= 10 + (txt[i + 1] - 0x30);
                        i++;
                    }
                    leadingSpace--;
                }

                /* Always display sign */
                else if (txt[i + 1] == 0x2B) {
                    useSign = 1;
                    i++;
                }

                /* Unknown specifier */
                else {
                    i++;
                }
            }
        }

        /* Normal character print */
        else {
            ei_putchar(txt[i]);
        }
    }

    //va_end(ap);
}


/**
 * @brief      Convert variable argument parameters to ascii characters and
 * write characters to given buffer
 * @details    Works like printf, only output is directly selectable.
 * Use %p for displaying fixed point in decimal format
 * <br>Some functions are not implemented:<br>
 * <ul><li>%i, %g, %e, %o</li>
 * <li>number of characters (in %d) are limited to 9</li></ul>
 * @param[in]  buffer     ptr to void returning function
 * @param[in]  txt        The text string
 * @param[in]  ...            argument list (uses stdarg.h)
 * @return     number of characters written
 */
int print_buf(char *buffer, const char *txt, ...)
{
    va_list ap;
    int i, intVal;
    int leadingZero, leadingSpace, useSign;
    unsigned int startAddr = (unsigned int)buffer;

    if(buffer == 0)
        return 0;

    va_start(ap, txt);

    for (i = 0; txt[i] != '\0'; i++) /* Walk through text */
    {
        leadingZero = leadingSpace = useSign = 0; /* Clear specifiers */

        /* % -> input from arg list */
        if (txt[i] == '%') {

            while(txt[i+1] != '\0') /* Walk through all args of % */
            {
                /* Decimal representation */
                if (txt[i + 1] == 'd' || txt[i + 1] == 'l' || txt[i + 1] == 'h') {

                    intVal = va_arg(ap, int);
                    SetOutputBuffer(buffer);
                    buffer += DecimalConverter(&WriteOutputBuffer, intVal, leadingZero,
                                               leadingSpace, 0, useSign);
                    i++;

                    if (txt[i + 1] == 'u' || txt[i + 1] == 'd') i++;

                    break;
                }
                /* Hexadecimal representation */
                else if (txt[i + 1] == 'X' || txt[i + 1] == 'x') {
                    intVal = va_arg(ap, signed long);

                    SetOutputBuffer(buffer);
                    buffer +=
                        HexadecimalConverter(&WriteOutputBuffer, intVal, leadingZero, leadingSpace);

                    i++;
                    break;
                }

                /* Float representation */
                else if (txt[i + 1] == 'f') {
                    unsigned long fraction;
                    signed long absolute;
                    int forceNeg;

                    float floatVal = va_arg(ap, double);

                    intVal = floatVal * (1 << FP_FRACTION);

                    forceNeg = ExtractFixedPoint(intVal, &absolute, &fraction);

                    SetOutputBuffer(buffer);
                    buffer +=
                        DecimalConverter(&WriteOutputBuffer, absolute, 0, 3, forceNeg, useSign);
                    *(buffer++) = '.';
                    SetOutputBuffer(buffer);
                    buffer += DecimalConverter(&WriteOutputBuffer, fraction, 3, 0, 0, 0);

                    i++;
                    break;
                }

                /* Fixed Point representation */
                else if (txt[i + 1] == 'p') {
                    unsigned long fraction;
                    signed long absolute;
                    int forceNeg;
                    intVal = va_arg(ap, int);

                    forceNeg = ExtractFixedPoint(intVal, &absolute, &fraction);

                    SetOutputBuffer(buffer);
                    buffer +=
                        DecimalConverter(&WriteOutputBuffer, absolute, 0, 3, forceNeg, useSign);
                    *(buffer++) = '.';
                    SetOutputBuffer(buffer);
                    buffer += DecimalConverter(&WriteOutputBuffer, fraction, 3, 0, 0, 0);

                    i++;
                    break;
                }

                /* 1 Char representation */
                else if (txt[i + 1] == 'c') {
                    *(buffer++) = ((char)va_arg(ap, int));
                    i++;
                    break;
                }

                /* Print percent sign */
                else if (txt[i + 1] == '%')
                {
                    *(buffer++) = '%';
                    i++;
                    break;
                }

                /* String representation */
                else if (txt[i + 1] == 's')
                {
                    char *strptr = (char *)va_arg(ap, int *);
                    /* Use EOF !!!                   */
                    while (*(strptr) != '\0') *(buffer++) = (*(strptr++));
                    i++;
                    break;
                }

                /* Use Leading zeroes */
                else if (txt[i + 1] == '.') {
                    leadingZero = (txt[i + 2] - 0x31);
                    i += 2;
                }

                /* Use N leading spaces */
                else if (txt[i + 1] > 0x30 && txt[i + 1] < 0x3A) {
                    leadingSpace = (txt[i + 1] - 0x30);
                    i++;
                    if (txt[i + 1] >= 0x30 && txt[i + 1] < 0x3A) {
                        leadingSpace *= 10 + (txt[i + 1] - 0x30);
                        i++;
                    }
                    leadingSpace--;
                }

                /* Always display sign */
                else if (txt[i + 1] == 0x2B) {
                    useSign = 1;
                    i++;
                }

                /* Unknown specifier */
                else {
                    i++;
                }
            }
        }

        else { /* Normal character print         */
            *(buffer++) = (txt[i]);
        }
    }

    va_end(ap);
    *(buffer) = '\0';

    return (unsigned int)buffer - startAddr;
}

/**
 * @brief      Sets the output buffer.
 * @param      outputbuffer  The outputbuffer
 */
static void SetOutputBuffer(char *outputbuffer)
{
    write_buffer = outputbuffer;
}

/**
 * @brief      Writes to an output buffer.
 * @param[in]  byte  The byte
 */
static void WriteOutputBuffer(char byte)
{
    *(write_buffer++) = byte;
}

/**
 * @brief      Convert a hexadecimal value to ASCII output
 * @param[in]  output     Function to sent a ASCII value to
 * @param[in]  hex_value  The hexadecimal value
 * @param[in]  zero       Number of zeroes to prepend (max 8)
 * @param[in]  space      Number of space characters to prepend (max 8)
 * @return     Number of bytes written to the output function
 */
static int HexadecimalConverter(void (*output)(char byte), signed long hex_value, unsigned int zero,
                                unsigned int space)
{
    signed int nibble_shift;
    unsigned int bytes_written = 0;

    zero *= 4;
    space *= 4;
    /* Walk through 32 bit           */
    for (nibble_shift = 28; nibble_shift >= 0;
         nibble_shift -= 4) {                  /* Mask nibble to check value     */
        if (hex_value &
            (0xFL << nibble_shift)) { /* place value                     */
            if (((hex_value >> nibble_shift) & 0xF) < 10)
                output(0x30 + ((hex_value >> nibble_shift) & 0xF));
            else
                output(0x37 + ((hex_value >> nibble_shift) & 0xF));

            zero = space = nibble_shift - 4;
            bytes_written++;
        }

        else                                    /* No value                      */
        {
            if (zero == (unsigned int)nibble_shift) {
                output(0x30);
                zero -= 4;
                bytes_written++;
            }

            else if (space == (unsigned int)nibble_shift) {
                output(0x20);
                space -= 4;
                bytes_written++;
            }
        }
    }

    return bytes_written;
}

/**
 * @brief      Calculate the size in characters for given value and
 * determine from that the character position for the sign.<br>
 * If zeroes are prepended, the sign has to be positioned in front.
 * @param[in]  value  The value
 * @param[in]  zero   Zero
 * @return     character position number for the sign
 */
static int SignPosition(signed int value, unsigned int zero)
{
    int y, pos;

    for (y = 9; y != 0; y--) {
        if (value >= (dec_maskArray[y] - 1)) break;
    };
    pos = y + 1;

    if (zero > 0) pos = zero + 1;

    return pos;
}

/**
 * @brief      Convert a decimal value to ASCII output
 * @param[in]  output     Function to sent a ASCII value to
 * @param[in]  dec_value  The decimal value
 * @param[in]  zero       Number of zeroes to prepend (max 8)
 * @param[in]  space      Number of space characters to prepend (max 8)
 * @return     Number of bytes written to the output function
 */
static int DecimalConverter(void (*output)(char byte), signed long dec_value, unsigned int zero,
                            unsigned int space, int forceNeg, int useSign)
{
    unsigned int y, bytes_written = 0;
    signed long highVal;
    unsigned int signPos = 0;
    unsigned char sign;

    if (dec_value < 0 || forceNeg) /* Check negativ */
    {
        dec_value = 0 - dec_value;
        signPos = SignPosition(dec_value, zero);
        sign = 0x2D; /* '-' character */
    }

    else if (useSign) /* Display sign if set */
    {
        signPos = SignPosition(dec_value, zero);
        sign = 0x2B; /* '+' character */
    }

    for (y = 9; space > y; space--) { /* Write spaces */
        output(0x20);
        bytes_written++;
    }

    for (y = 9; y != 0; y--) /* Display 9 characters */
    {
        if (dec_value >= dec_maskArray[y]) {
            highVal = dec_value / dec_maskArray[y];
            output(0x30 + highVal);
            bytes_written++;
            dec_value -= (highVal * dec_maskArray[y]);
            zero = space = y - 1;
        }

        else {
            if (signPos == y) {
                output(sign);
                bytes_written++;
            }

            else if (zero == y) {
                output(0x30);
                bytes_written++;
                zero--;
            }

            else if (space == y) {
                output(0x20);
                bytes_written++;
                space--;
            }
        }
    }

    output(0x30 + dec_value); /* Rest value */
    bytes_written++;

    return bytes_written;
}

/**
 * @brief      Extract absolute and fraction part from fixed value
 * @details    Absolute and fraction values are written to given pointer addresses
 * @param[in]  fixedVal      The fixed value
 * @param      abspart       Pointer to absolute destination value
 * @param      fractionpart  Pointer to fraction destination value
 * @return     If fixed value is negativ but absolute part is 0, return value must be 1
 */
static int ExtractFixedPoint(signed long fixedVal, signed long *abspart, unsigned long *fractionpart)
{
    int isNegativ;
    /* Swap negativ values           */
    if (fixedVal & ((unsigned int)1 << (FP_LENGTH - 1))) {
        fixedVal = ~(fixedVal) + 1;
        *(abspart) = (0 - (fixedVal >> FP_FRACTION));
        *(fractionpart) = (Fixed_Multiply(((fixedVal & FP_MASK) + 1) * 10000, 1));
        isNegativ = 1;
    }

    else {
        *(abspart) = (fixedVal >> FP_FRACTION);
        *(fractionpart) = (Fixed_Multiply(((fixedVal & FP_MASK) + 1) * 10000, 1));
        isNegativ = 0;
    }

    return isNegativ;
}
