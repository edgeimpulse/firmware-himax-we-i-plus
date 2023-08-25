/**
 * @file FixedPoint.h
 * @author Arjan Kamphuis
 * @copyright 2020 AUTOMATIEK - Arjan Kamphuis
 * @date 13-10-2009
 * @brief Fixedpoint arithmetic settings
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

#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

/* Fixed-Point Arithmetic Settings ----------------------------------------- */
#define FP_LENGTH   		32
#define FP_INTEGER        	16
#define FP_FRACTION 		16
#define FP_MASK			 	0x0000FFFF

/*******************************************************************************
* Function Name  : Fixed_Multiply
* Description    : Multiplies 2 fixed values
					values are split in Integer * Fraction part 
					then crossmultiplied and added
* Input          : fixed1, fixed2 -> 2 fixed signed ints
* Output         : None
* Return         : fixed multiply -> signed int
*******************************************************************************/
extern signed int Fixed_Multiply(signed int fixed1, signed int fixed2)
{
    signed int int1, int2;
    unsigned int frac1, frac2;
    signed long long fp_som_int = 0;
    signed long long fp_som_frac = 0;
    signed long long fp_total_som = 0;

    /* Convert to bytes */
    int1 = (fixed1 >> FP_FRACTION);
    int2 = (fixed2 >> FP_FRACTION);
    frac1 = (unsigned int)(fixed1 & FP_MASK);
    frac2 = (unsigned int)(fixed2 & FP_MASK);

    /* Multiply */
    fp_som_int = (signed long long)int1 * int2;

    fp_som_frac = (signed long long)int1 * frac2 + int2 * frac1;

    fp_total_som = (signed long long)(fp_som_int << FP_FRACTION) + (fp_som_frac);

    fp_total_som += (frac1 * frac2) >> FP_FRACTION;

    return fp_total_som;
}

#endif //FIXEDPOINT_H