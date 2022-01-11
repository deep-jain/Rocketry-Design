#ifndef FLAP_INTERFACE_H
#define FLAP_INTERFACE_H

#include <Arduino.h>

class FlapInterface
{
    public:

        FlapInterface(uint8_t pospin, uint8_t negpin, uint8_t maxstate)
        {
            /*
             * WATCH OUT IT'S A BOA CONSTRUCTOR
             *       __    __    __    __
             *      /  \  /  \  /  \  /  \
             *_____/  __\/  __\/  __\/  __\____________
             *____/  /__/  /__/  /__/  /_______________|
             *    | / \   / \   / \   / \   \___
             *    |/   \_/   \_/   \_/   \    o \
             *                            \_____/--<
             */

            this->pospin = pospin;
            this->negpin = negpin;
            this->maxstate = maxstate;
            state = 0;
            pinMode(pospin, OUTPUT);
            pinMode(negpin, OUTPUT);
            kill();
        }

        uint8_t deploy()
        {
            if (state == maxstate)
            {
                kill();
                return 0;
            }
            else
            {
                state++;
                digitalWrite(negpin, LOW);
                digitalWrite(pospin, HIGH);
                return 1;
            }
        }

        uint8_t retract()
        {
            if (state == 0)
            {
                kill();
                return 0;
            }
            else
            {
                state--;
                digitalWrite(pospin, LOW);
                digitalWrite(negpin, HIGH);
                return 1;
            }
        }

        void kill()
        {
            digitalWrite(pospin, LOW);
            digitalWrite(negpin, LOW);
        }

    private:

        uint8_t state;
        uint8_t maxstate;
        uint8_t pospin;
        uint8_t negpin;
};

#endif
