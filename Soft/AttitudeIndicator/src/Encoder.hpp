#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "driver/gpio.h"

class Encoder{

public :

    void begin(gpio_num_t a_phase, gpio_num_t b_phase);

private:

// Piny faz A i B enkodera
    static gpio_num_t a_phase_gpio;
    static gpio_num_t b_phase_gpio;

};

#endif