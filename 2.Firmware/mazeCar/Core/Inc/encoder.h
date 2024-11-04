#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>
typedef enum
{
	NONE_WHEEL  = 0,
    LEFT_WHEEL  = 1,
	RIGHT_WHEEL = 2

} wheel_t;
typedef struct encoder
{
	wheel_t wheel;
    int32_t position;
    double speed; // units per minute
    uint32_t previous_millis;

} encoder_t;



uint8_t init_encoder(encoder_t *enc, wheel_t wheel);

void update_encoder(encoder_t *enc);

float get_speed_enc(encoder_t *enc);

int32_t get_position_enc(encoder_t *enc);

void reset_position_enc(encoder_t *enc);

void print_info_enc(encoder_t *enc);

#endif // ENCODER_H
