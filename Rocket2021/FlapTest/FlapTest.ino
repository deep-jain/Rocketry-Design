#include <FlapInterface.h>

#define FLAP_POSITIVE 9
#define FLAP_NEGATIVE 10

FlapInterface flaps(FLAP_POSITIVE, FLAP_NEGATIVE, 50);

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting motor test.");
}

void loop()
{
    test3();
}

void test1()
{
    // test deploy/retract cycle with decreasing periods between
    for (uint16_t millis = 2000; millis > 300; millis /= 2)
    {
        Serial.print("Testing deploy/retract with period: ");
        Serial.print(millis);
        Serial.println(" milliseconds");
        cycle(millis);
        delay(1000);
    }

    flaps.kill();
}

void test2()
{
    // cyclic stress testing by very fast deployment cycles
    Serial.println("Stress testing flap deployment cycle.");
    for (int i = 0; i < 10; i++)
    {
        cycle(50);
        delay(50);
    }

    flaps.kill();
}

void test3()
{
    for (int i = 0; i < 5; i++)
    {
        cycle(400);
    }
}

void cycle(uint16_t millis)
{
    flaps.deploy();
    delay(millis);
    flaps.retract();
    delay(millis);
}
