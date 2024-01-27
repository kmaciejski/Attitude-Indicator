#include "Encoder.hpp"

gpio_num_t Encoder::a_phase_gpio = GPIO_NUM_NC;
gpio_num_t Encoder::b_phase_gpio = GPIO_NUM_NC;

void Encoder::begin(gpio_num_t a_phase, gpio_num_t b_phase){

    a_phase_gpio = a_phase;
    b_phase_gpio = b_phase;


    

}