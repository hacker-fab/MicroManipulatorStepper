#include <Arduino.h>
#include <SPI.h>

#include "rotation.h"
#include "config.h"




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

    88, 73468, 144162, 217968, 299524, 392963, 495124, 587029, 664615, 742582, 832565, 926229, 1011229, 1089670, 1166791, 1242996, 1326204, 1423911, 1534225, 1630769, 1709275, 1787039, 1874922, 1964390, 2046281, 2120404, 2194579, 2267403, 2345279, 2434676, 2542563, 2646433, 2731082, 2808645, 2896512, 2990302, 3074006, 3147290, 3219543, 3289404, 3358997, 3434336, 3522631, 3620457, 3708046, 3784252, 3859348, 3940763, 4023959, 4101832, 4175037, 4244045, 4308229, 4371863, 4442688, 4523186, 4609187, 4699247, 4786158, 4863488, 4940634, 5026535, 5121367, 5211882, 5294927, 5373336, 5456016, 5551607, 5670733, 5780918, 5869442, 5952576, 6042805, 6135536, 6223927, 6305032, 6380345, 6450471, 6527734, 6617766, 6719378, 6818440, 6906505, 6986977, 7069436, 7158361, 7248035, 7328114, 7402903, 7472877, 7546647, 7629311, 7726090, 7829805, 7924167, 8005066, 8086370, 8174526, 8266402, 8349489, 8425870, 8495763, 8567334, 8646064, 8735518, 8835190, 8935453, 9020854, 9099467, 9182957, 9276211, 9364652, 9441650, 9513174, 9583451, 9656781, 9738017, 9829579, 9929908, 10021394, 10099456, 10177055, 10264714, 10356552, 10440692, 10517849, 10592486, 10666321, 10746655, 10839170, 10947146, 11047482, 11129239, 11207409, 11296323, 11391661, 11479294, 11557448, 11633820, 11708819, 11789193, 11881583, 11990945, 12092152, 12175146, 12253381, 12342258, 12435100, 12519133, 12594098, 12667636, 12738055, 12810065, 12890554, 12984124, 13082559, 13168785, 13245039, 13323321, 13410123, 13499515, 13581399, 13658916, 13731973, 13802292, 13876459, 13963549, 14059033, 14152865, 14232997, 14308421, 14384088, 14465157, 14546902, 14626024, 14700258, 14769399, 14836192, 14909552, 14993796, 15085084, 15180083, 15269248, 15347158, 15422824, 15506721, 15597406, 15683206, 15761563, 15836145, 15910973, 15993750, 16089132, 16194566, 16291557, 16372323, 16448764, 16530903, 16618631, 16702163, 16778071, 16850220, 16920916, 16994980, 17076976, 17170750, 17273202, 17364908, 17441941, 17520020, 

};


// PID control variables
static float i_term = 0;
static int32_t last_diff = 0;
static float filtered_d_term = 0;

static int32_t target_sensor_angle = 0;
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
    // rotate until endstop is triggered, then set current angle as home (0)
    uint32_t current_angle = 0;
    uint8_t home_error = 0;
    while (digitalRead(ROTATION_ENDSTOP) == HIGH)
    {
        if (current_angle > PWM_MAX_VALUE * 8 * 50) {
            home_error = 1;
            break;
        }
        writeFieldAngle(current_angle);
        current_angle += 256;

        delay(1);
    }

    if (home_error){
        return 1;
    }

    while (digitalRead(ROTATION_ENDSTOP) == LOW)
    {
        writeFieldAngle(current_angle);
        current_angle -= 10;

        delay(1);
    }

    while (digitalRead(ROTATION_ENDSTOP) == HIGH)
    {
        writeFieldAngle(current_angle);
        current_angle += 1;

        delay(1);
    }


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


    int32_t sensor_angle = get_total_sensor_angle();
    int32_t diff = target_sensor_angle - sensor_angle;

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
    angle_rad += PI / 2.0; // 90 deg offset
    angle_rad = fmod(angle_rad, 2 * PI);
    if (angle_rad < 0)
    {
        angle_rad += 2 * PI;
    }
    angle_rad -= PI / 2.0; // remove 90 deg offset

    // convert using the lookup table to counter any magnetic non-linearity
    // a full rotation is index 200
    float index_float = angle_rad / (2 * PI) * 200;
    uint32_t index = (uint32_t)(index_float);
    float ratio = index_float - index;
    index = index % 200;

    target_sensor_angle = angle_lookup_table[index] * (1 - ratio) + angle_lookup_table[(index + 1) % 200] * ratio;

}
float rotation_get(void){
    return get_total_sensor_angle() / (float)MT6835_CPR / 8 * 2 * PI;
}

float rotation_target_get(){
    return target_angle_rad;
}