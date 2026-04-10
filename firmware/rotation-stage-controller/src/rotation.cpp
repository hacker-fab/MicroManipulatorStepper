#include <Arduino.h>
#include <SPI.h>

#include "rotation.h"
#include "config.h"
#include "utils.h"



/**
 * MT6835 Sensor
 */
static SPIClass SensorSPI(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCK);
static SPISettings MT6835SPISettings(1000000, MSBFIRST, SPI_MODE3);

static int32_t last_sensor_angle = 0;
static int32_t sensor_angle_wraps = 0;



/**
 * Endstop & homing
 */
static uint32_t field_angle_at_home = 0;
static int32_t sensor_angle_at_home = 0;


/**
 * Control
 */
static int32_t angle_lookup_table[LOOKUP_TABLE_SIZE] = {

803, 80199, 162763, 263964, 371354, 462582, 536723, 617580, 706971, 800610, 879437, 957299, 1029922, 1103962, 1179770, 1272338, 1374357, 1470539, 1546842, 1623162, 1706264, 1795393, 1875997, 1954384, 2029325, 2100516, 2168396, 2252479, 2347715, 2450448, 2536654, 2617727, 2695038, 2781146, 2867179, 2952507, 3030339, 3105127, 3173395, 3251020, 3340049, 3442290, 3537875, 3624271, 3702328, 3784521, 3869919, 3961190, 4044754, 4123548, 4192388, 4268921, 4357371, 4458730, 4556530, 4649105, 4729597, 4808611, 4889235, 4979749, 5062323, 5138204, 5205310, 5277009, 5354044, 5443407, 5537779, 5640217, 5726592, 5805483, 5883114, 5975064, 6065341, 6146889, 6219051, 6292696, 6368919, 6456105, 6550672, 6658377, 6750948, 6832292, 6908976, 6999950, 7093455, 7176867, 7248518, 7320870, 7394292, 7474394, 7558951, 7660078, 7757334, 7840228, 7911167, 7994132, 8083699, 8169145, 8241334, 8314500, 8383936, 8455684, 8531101, 8622022, 8721983, 8819039, 8895692, 8974108, 9060796, 9157728, 9243712, 9327148, 9407157, 9486797, 9567923, 9671851, 9788430, 9887590, 9963725, 10046357, 10140046, 10234450, 10314029, 10393697, 10469465, 10543741, 10620795, 10717519, 10823981, 10918007, 10994514, 11072747, 11157563, 11245054, 11322343, 11397481, 11468333, 11536616, 11602904, 11680944, 11768962, 11866566, 11954377, 12036342, 12112595, 12196772, 12282668, 12372353, 12453821, 12531291, 12600225, 12678635, 12769602, 12872126, 12966143, 13052261, 13130174, 13208996, 13288390, 13375329, 13455236, 13529041, 13594061, 13663691, 13738436, 13824935, 13914152, 14014230, 14102325, 14180391, 14255688, 14347708, 14442270, 14529720, 14607398, 14685946, 14767032, 14861196, 14964773, 15071292, 15158830, 15237920, 15313487, 15399768, 15485549, 15563065, 15630525, 15699397, 15768666, 15843722, 15922634, 16019020, 16119305, 16206091, 16279260, 16360828, 16453691, 16547603, 16625269, 16703587, 16778520, 16856122, 16937709, 17038307, 17144251, 17236948, 17312099, 17392687, 17482366, 17575970, 

};


// PID control variables
static float i_term = 0;
static int32_t last_diff = 0;
static float filtered_d_term = 0;

static int32_t target_sensor_angle = 0;
static int32_t interpolated_sensor_target_angle = 0;
static float target_angle_rad = 0; // only for the getter function, not used for control


static uint8_t calc_crc(uint32_t angle, uint8_t status) {
    uint8_t crc = 0x00;

    uint8_t input = angle>>13;
    crc ^= input;
    for (int k = 8; k > 0; k--)
        crc = (crc & (0x01<<7))?(crc<<1)^0x07:crc<<1;

    input = (angle>>5) & 0xFF;
    crc ^= input;
    for (int k = 8; k > 0; k--)
        crc = (crc & (0x01<<7))?(crc<<1)^0x07:crc<<1;

    input = ((angle<<3) & 0xFF) | (status & 0x07);
    crc ^= input;
    for (int k = 8; k > 0; k--)
        crc = (crc & (0x01<<7))?(crc<<1)^0x07:crc<<1;

    return crc;
};


static int32_t read_raw_angle() {

    uint8_t data[6];
    data[0] = (0b1010<<4);
    data[1] = 0x03;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;

    digitalWrite(SENSOR_SS, LOW);
    SensorSPI.beginTransaction(MT6835SPISettings);

    SensorSPI.transfer(data, 6);

    SensorSPI.endTransaction();
    digitalWrite(SENSOR_SS, HIGH);

    uint32_t raw_angle = ((uint32_t)data[2] << 13) | ((uint32_t)data[3] << 5) | ((uint32_t)data[4] >> 3);
    uint8_t status = data[4]&0x07;
    uint8_t crc = data[5];

    uint8_t expected_crc = calc_crc(raw_angle, status);
    if (expected_crc != crc) {
        return -1;
    }
    return raw_angle;
}

static float raw_angle_to_radians(uint32_t raw_angle) {
    return raw_angle / (float)MT6835_CPR * PI * 2;
}





static void poll_sensor()
{
    // angle in radians, 0 to 2pi. -1 on error
    int32_t raw_angle = read_raw_angle();
    if (raw_angle < 0)
    {
        // error, ignore
        return;
    }

    if (raw_angle - last_sensor_angle > MT6835_CPR / 2)
    {
        sensor_angle_wraps -= 1;
    }
    else if (raw_angle - last_sensor_angle < -MT6835_CPR / 2)
    {
        sensor_angle_wraps += 1;
    }
    last_sensor_angle = raw_angle;
}

static int32_t get_total_sensor_angle()
{
    return last_sensor_angle + sensor_angle_wraps * MT6835_CPR - sensor_angle_at_home;
}




static void writeFieldAngle(uint32_t angle)
{
    // use integer angle directly corresponding to pwm value
    // Each quadrant has PWM_MAX_VALUE * 2 steps, full rotation has 4 quadrants
    angle = (angle + field_angle_at_home) % (PWM_MAX_VALUE * 8);

    int32_t a_val = 0;
    int32_t b_val = 0;


    // a value
    if ((0 <= angle && angle < PWM_MAX_VALUE) || (PWM_MAX_VALUE * 7) <= angle && angle < PWM_MAX_VALUE * 8)
    {
        // vertical edge
        a_val = PWM_MAX_VALUE-1;
        b_val = angle;
        if (b_val > PWM_MAX_VALUE * 4)
        {
            b_val -= PWM_MAX_VALUE * 8;
        }
    }
    else if (PWM_MAX_VALUE <= angle && angle < PWM_MAX_VALUE * 3)
    {
        // horizontal edge
        a_val = -(angle - PWM_MAX_VALUE * 2);
        b_val = PWM_MAX_VALUE-1;
    }
    else if (PWM_MAX_VALUE * 3 <= angle && angle < PWM_MAX_VALUE * 5)
    {
        // vertical edge
        a_val = -PWM_MAX_VALUE+1;
        b_val = -(angle - PWM_MAX_VALUE * 4);
    }
    else if (PWM_MAX_VALUE * 5 <= angle && angle < PWM_MAX_VALUE * 7)
    {
        // horizontal edge
        a_val = angle - (PWM_MAX_VALUE * 6);
        b_val = -PWM_MAX_VALUE+1;
    }
    else
    {
        // default case, should not happen
        a_val = 0;
        b_val = 0;
    }

    if (a_val > 0)
    {
        pwm_start(MOT_A_POS, PWM_FREQ, a_val, PWM_RES);
        pwm_start(MOT_A_NEG, PWM_FREQ, 0, PWM_RES);
    }
    else
    {
        pwm_start(MOT_A_POS, PWM_FREQ, 0, PWM_RES);
        pwm_start(MOT_A_NEG, PWM_FREQ, -a_val, PWM_RES);
    }
    if (b_val > 0)
    {
        pwm_start(MOT_B_POS, PWM_FREQ, b_val, PWM_RES);
        pwm_start(MOT_B_NEG, PWM_FREQ, 0, PWM_RES);
    }
    else
    {
        pwm_start(MOT_B_POS, PWM_FREQ, 0, PWM_RES);
        pwm_start(MOT_B_NEG, PWM_FREQ, -b_val, PWM_RES);
    }
}


void buildAngleLookupTable()
{
    writeFieldAngle(0);
    delay(10);

    for (uint32_t i = 0; i < LOOKUP_TABLE_SIZE; i++)
    {
        writeFieldAngle(i * PWM_MAX_VALUE * 2);
        delay(100);
        poll_sensor();
        angle_lookup_table[i] = get_total_sensor_angle();
        Serial.println(i);
    }
}



void printAngleLookupTable()
{
    for (uint16_t i = 0; i < 210; i++)
    {
        Serial.print(angle_lookup_table[i]);
        Serial.print(", ");
    }
    Serial.println();
}


static uint32_t sensorAngleToFieldAngle(int32_t sensor_angle)
{
    // find the closest angle in the lookup table
    // TODO: binary search
    while (sensor_angle < angle_lookup_table[0])
    {
        // map to the end of the table
        sensor_angle += MT6835_CPR * 8;
    }
    while (sensor_angle > angle_lookup_table[LOOKUP_TABLE_SIZE - 1])
    {
        // map to the start of the table
        sensor_angle -= MT6835_CPR * 8;
    }

    uint16_t closest_index = 0;
    for (uint16_t i = 1; i < LOOKUP_TABLE_SIZE; i++)
    {
        if (angle_lookup_table[i] > sensor_angle)
        {
            closest_index = i - 1;
            break;
        }
    }

    // interpolate between closest_index and closest_index + 1
    int32_t angle_a = angle_lookup_table[closest_index];
    int32_t angle_b = angle_lookup_table[closest_index + 1];
    float ratio = (sensor_angle - angle_a) / (float)(angle_b - angle_a);

    // return closest_index * PWM_MAX_VALUE * 2 + (sensor_angle - angle_a) * PWM_MAX_VALUE * 2 / (angle_b - angle_a);
    return closest_index * PWM_MAX_VALUE * 2 + ratio * PWM_MAX_VALUE * 2;
}


uint8_t rotation_home(){
    // Try to find the endstop, then set current angle as home (0)
    // rotate forward and backward for 90 deg, if the endstop is not found, return error
    int32_t current_angle = 0;

    // one full rotation has 50 step cycles, and each step cycle has PWM_MAX_VALUE * 8 field angles
    const int32_t full_round_field_angle = PWM_MAX_VALUE * 8 * 50;
    
    // forward rotation 90 deg
    bool forward_found = true;
    while (digitalRead(ROTATION_ENDSTOP) == HIGH)
    {
        if (current_angle > full_round_field_angle / 4) {
            forward_found = false;
            break;
        }
        writeFieldAngle(current_angle);
        current_angle += 256;
        current_angle = positive_mod(current_angle, full_round_field_angle);
        
        delay(1);
    }
    
    // backward rotation 90 deg
    bool backward_found = true;
    while (digitalRead(ROTATION_ENDSTOP) == HIGH)
    {
        if (current_angle < -full_round_field_angle / 4) {
            backward_found = false;
            break;
        }
        writeFieldAngle(current_angle);
        current_angle -= 256;
        current_angle = positive_mod(current_angle, full_round_field_angle);

        delay(1);
    }

    if (!forward_found && !backward_found) {
        // endstop not found
        return 1;
    }
    
    
    // now the homing switch is triggered, move backward to find the front edge
    
    while (digitalRead(ROTATION_ENDSTOP) == LOW)
    {
        writeFieldAngle(current_angle);
        current_angle -= 20;
        current_angle = positive_mod(current_angle, full_round_field_angle);
        
        delay(1);
    }

    // moved past the front edge, move forward a little to trigger the endstop again
    while (digitalRead(ROTATION_ENDSTOP) == HIGH)
    {
        writeFieldAngle(current_angle);
        current_angle += 1;
        current_angle = positive_mod(current_angle, full_round_field_angle);

        delay(1);
    }

    // fully homed

    field_angle_at_home = current_angle;
    sensor_angle_wraps = 0;
    sensor_angle_at_home = read_raw_angle();

    return 0;
}

















int rotation_init(void){
    
    SensorSPI.begin();
    pinMode(SENSOR_SS, OUTPUT);
    digitalWrite(SENSOR_SS, HIGH);

    pinMode(ROTATION_ENDSTOP, INPUT_FLOATING);

    return 0;
}
int rotation_poll(uint32_t dt_us){
    poll_sensor();

    // update the interpolate target angle (for speed limiting)
    int32_t interpolate_diff = target_sensor_angle - interpolated_sensor_target_angle;
    int32_t max_turn = MT6835_CPR * (dt_us / 1000000.0) * 2;
    interpolated_sensor_target_angle += constrain(interpolate_diff, -max_turn, max_turn);
    

    int32_t sensor_angle = get_total_sensor_angle();
    int32_t diff = interpolated_sensor_target_angle - sensor_angle;

    i_term += ((float)diff / MT6835_CPR) * dt_us / 1000000.0 * 500; // integral term, with time in seconds
    i_term = constrain(i_term, -0.5, 0.5); // anti-windup

    float d_term = (float)(diff - last_diff) / MT6835_CPR / (dt_us / 1000000.0) * 2; // derivative term
    last_diff = diff;
    d_term = constrain(d_term, -0.5, 0.5); // limit derivative term
    filtered_d_term = filtered_d_term * 0.99 + d_term * 0.01; // low-pass filter for derivative term

    float torque = ((float)diff / MT6835_CPR) * 20 + i_term; - filtered_d_term;
    if (torque < -1)
    {
        torque = -1;
    }
    else if (torque > 1)
    {
        torque = 1;
    }
    
    int32_t field_angle_diff = torque * PWM_MAX_VALUE * 2; // max torque corresponds to 90 degree field angle difference
    uint32_t field_angle = sensorAngleToFieldAngle(sensor_angle) + field_angle_diff;

    writeFieldAngle(field_angle);

    return 0;
}

void rotation_set(float angle_rad){
    target_angle_rad = angle_rad;

    // the hardware only support -90 to 270 degrees
    // map the rotation to that range
    // angle_rad += PI / 2.0; // 90 deg offset
    // angle_rad = fmod(angle_rad, 2 * PI);
    while (angle_rad < -PI/2)
    {
        angle_rad += 2 * PI;
    }
    while (angle_rad > PI/2*3)
    {
        angle_rad -= 2 * PI;
    }
    Serial.println(angle_rad * 180 / PI);

    float single_loop_angle = angle_rad;
    while(single_loop_angle < 0){
        single_loop_angle += 2 * PI;
    }

    bool negative = angle_rad < 0;

    
    // angle_rad -= PI / 2.0; // remove 90 deg offset

    // convert using the lookup table to counter any magnetic non-linearity
    // a full rotation is index 200
    float index_float = single_loop_angle / (2 * PI) * 200;
    uint32_t index = (uint32_t)(index_float);
    float ratio = index_float - index;
    index = index % 200;

    target_sensor_angle = angle_lookup_table[index] * (1 - ratio) + angle_lookup_table[(index + 1) % 200] * ratio;
    if (negative){
        target_sensor_angle -= MT6835_CPR * 8;
    }
}
float rotation_get(void){
    return get_total_sensor_angle() / (float)MT6835_CPR / 8 * 2 * PI;
}

float rotation_target_get(){
    return target_angle_rad;
}