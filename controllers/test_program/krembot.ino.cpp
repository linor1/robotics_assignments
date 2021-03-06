/*******************************************************************************
* Copyright (c) 2019, Elhay Rauper.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of Elhay Rauper nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


/*
 * This file simulates the .ino file of the krembot.
 * The original structure of this file must be maintained
 * for the simulator to work correctly.
 * Don't create Krembot instance, it is created automatically.
 * Copy and past real robot setup() and loop() code into the functions below
 */

# ifdef KREMBOT_SIM
#include <Krembot/controller/krembot_controller.h>
# else
#include "Krembot/krembot.h"
# endif

/*
 * todo: Your #include statements go here
 */

# ifdef KREMBOT_SIM
//DO NOT EDIT THIS MACRO
KREMBOT_CONTROLLER_HEADER(CONTROLLER_NAME)
# else
    //Only one instance of krembot object should be declared
    Krembot krembot;
#endif
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

/*
 * todo: Your global variables go here
 */


void setup() {
    krembot.setup();

    /*
     * TODO: Your setup code goes here
     */
    krembot.Led.write(0,255,0);
}

void loop() {
    krembot.loop();

    /*
     * TODO: Your loop code goes here
     */

    float distance = krembot.RgbaFront.readRGBA().Distance;
    if (distance < 15)
    {
        krembot.Base.stop();
        krembot.Led.write(255,0,0);
    } else
    {
        krembot.Base.drive(100,0);
    }
}


////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////DO NOT EDIT THESE MACROS
# ifdef KREMBOT_SIM
KREMBOT_CONTROLLER_FOOTER(CONTROLLER_NAME, CONTROLLER_LABEL)
# endif


