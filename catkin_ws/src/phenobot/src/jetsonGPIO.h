/*
 * jetsonGPIO.h
 *
 * This modeule handles GPIOs of Jetson TX1. The code is originally from
 * https://github.com/jetsonhacks/jetsonTX1GPIO, then modified and
 * maintained by JK Jung <jkjung13@gmail.com>. The original copyright
 * notice is retained below.
 */

/*
 * Copyright (c) 2015 JetsonHacks
 * www.jetsonhacks.com
 *
 * Based on Software by RidgeRun
 * Originally from:
 * https://developer.ridgerun.com/wiki/index.php/Gpio-int-test.c
 */
/*
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Modified by Jingyao Oct 8 2018 for TX2
// J21 Header on developer kit board pinout: https://www.jetsonhacks.com/nvidia-jetson-tx2-j21-header-pinout/
// Detailed specification P24, S3.4: https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/390/JetsonTX1_5F00_TX2_5F00_Developer_5F00_Kit_5F00_Carrier_5F00_Board_5F00_Specification1.pdf
// Orbitty: http://connecttech.com/pdf/CTIM-ASG003_Manual.pdf

#ifndef JETSONGPIO_H_
#define JETSONGPIO_H_

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF        64

typedef unsigned int jetsonGPIO;
typedef unsigned int pinDirection;
typedef unsigned int pinValue;

enum pinDirections {
	inputPin  = 0,
	outputPin = 1,
} ;

enum pinValues {
        low  = 0,
        high = 1,
        off  = 0,  /* synonym for things like lights */
        on   = 1,
};

enum jetsonTX2GPIONumber {
        // TX2 develop kit only
        gpio396  = 396,   /* J21 - Pin 7 - Bidir - GPIO_GCLK     */
        gpio466  = 466,   /* J21 - Pin 11 - Output - GPIO_GEN0     */
        gpio392  = 392,   /* J21 - Pin 12 - Bidir  - GPIO_GEN1     */
        gpio397  = 397,   /* J21 - Pin 13 - Bidir  - GPIO_GEN2  */
        gpio255 = 255,  /* J21 - Pin 15 - Bidir (from expander P17)  - GPIO_GEN3 */
        gpio296 = 296,  /* J21 - Pin 16 - Input  - GPIO_GEN4   */
        gpio481 = 481,  /* J21 - Pin 18 - Input - GPIO_GEN5 */
        gpio254 = 254,  /* J21 - Pin 22 - Bidir (from expander P16) - GPIO_GEN6     */
        gpio398 = 398,  /* J21 - Pin 29 - Output - GPIO5     */
        gpio389 = 389,  /* J21 - Pin 33 - Bidir - GPIO13     */
        gpio394 = 394,  /* J21 - Pin 38 - Input - GPIO20     */
        gpio393 = 393,  /* J21 - Pin 40 - Output - GPIO21     */

        // TX2 develop kit & Orbitty
        gpio388  = 388,   /* J21 - Pin 37 - Output - GPIO26  &&  Pin 7 - GPIO_PQ4_PI4 - PU20K*/
        gpio298  = 298,   /* J21 - Pin 31 - Input - GPIO6 && Pin 8 - GCAN_GPIO2_PAA2 - PU20K*/
        
        // Orbitty only
        gpio480  = 480,   /* Pin 9 - GPIO_MDM7_PY6 - PU100K     */
        gpio486  = 486,   /* Pin 10 - GPIO_MDM1_PY0 - PU100k  */
};


int gpioExport(jetsonGPIO gpio);
int gpioUnexport(jetsonGPIO gpio);
int gpioSetDirection(jetsonGPIO, pinDirection out_flag);
int gpioSetValue(jetsonGPIO gpio, pinValue value);
int gpioGetValue(jetsonGPIO gpio, unsigned int *value);
int gpioSetEdge(jetsonGPIO gpio, char *edge);
int gpioActiveLow(jetsonGPIO gpio, unsigned int value);

#endif /* JETSONGPIO_H_ */
