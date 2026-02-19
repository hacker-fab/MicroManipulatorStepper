#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>


/**
 * MT6835 Sensor
 */

#define SENSOR_MOSI PB5
#define SENSOR_MISO PB4
#define SENSOR_SCK PB3
#define SENSOR_SS PA15

SPIClass SensorSPI(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCK);
static SPISettings MT6835SPISettings(1000000, MSBFIRST, SPI_MODE3);
#define MT6835_CPR 2097152

int32_t last_sensor_angle = 0;
int32_t sensor_angle_wraps = 0;



/**
 * PWM
 */

// The 72MHz clock cannot support 40kHz with 12-bit resolution, so the actual frequency will be lower
#define PWM_FREQ 40000
#define PWM_RESOLUTION RESOLUTION_12B_COMPARE_FORMAT
#define PWM_MAX_VALUE 4096

#define MOT_A_POS PA_0
#define MOT_A_NEG PA_1
#define MOT_B_POS PA_2
#define MOT_B_NEG PA_3


/**
 * I2C
 */
#define I2C_SDA PB7
#define I2C_SCL PB6



/**
 * Control
 */

#define STEPPER_STEPS 50
#define LOOKUP_TABLE_SIZE (STEPPER_STEPS * 4 + 10) // 10 extra for safety margin
int32_t angle_lookup_table[LOOKUP_TABLE_SIZE] = {0};

uint32_t last_time = 0;
// PID control variables
float i_term = 0;
int32_t last_diff = 0;
float filtered_d_term = 0;

int32_t target_sensor_angle = 0;



uint8_t calc_crc(uint32_t angle, uint8_t status) {
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


int32_t read_raw_angle() {

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

float raw_angle_to_radians(uint32_t raw_angle) {
    return raw_angle / (float)MT6835_CPR * PI * 2;
}





void poll_sensor()
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

int32_t get_total_sensor_angle()
{
    return last_sensor_angle + sensor_angle_wraps * MT6835_CPR;
}




void writeFieldAngle(uint32_t angle)
{
    // use integer angle directly corresponding to pwm value
    // Each quadrant has PWM_MAX_VALUE * 2 steps, full rotation has 4 quadrants
    angle = angle % (PWM_MAX_VALUE * 8);

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
        pwm_start(MOT_A_POS, PWM_FREQ, a_val, PWM_RESOLUTION);
        pwm_start(MOT_A_NEG, PWM_FREQ, 0, PWM_RESOLUTION);
    }
    else
    {
        pwm_start(MOT_A_POS, PWM_FREQ, 0, PWM_RESOLUTION);
        pwm_start(MOT_A_NEG, PWM_FREQ, -a_val, PWM_RESOLUTION);
    }
    if (b_val > 0)
    {
        pwm_start(MOT_B_POS, PWM_FREQ, b_val, PWM_RESOLUTION);
        pwm_start(MOT_B_NEG, PWM_FREQ, 0, PWM_RESOLUTION);
    }
    else
    {
        pwm_start(MOT_B_POS, PWM_FREQ, 0, PWM_RESOLUTION);
        pwm_start(MOT_B_NEG, PWM_FREQ, -b_val, PWM_RESOLUTION);
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

uint32_t sensorAngleToFieldAngle(int32_t sensor_angle)
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



void setup()
{
    Serial.begin(115200);


    SensorSPI.begin();
    pinMode(SENSOR_SS, OUTPUT);
    digitalWrite(SENSOR_SS, HIGH);


    // Serial.println("Building angle lookup table...");
    buildAngleLookupTable();
    // Serial.println("Angle lookup table:");
    printAngleLookupTable();
}


int i = 0;

char input_buffer[16] = {0};
uint16_t input_index = 0;

void loop()
{
    // calculate dt

    uint32_t current_time = micros();
    uint32_t dt = 0;
    if (current_time < last_time) {
        // handle micros() overflow
        dt = (uint32_t)(current_time + (1<<32 - last_time));
    } else {
        dt = current_time - last_time;
    }
    last_time = current_time;

    // poll sensor and get angle
    poll_sensor();







    if (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n')
        {
            input_buffer[input_index] = '\0';
            // input in degree
            // manual parse to avoid large size
            int temp = 0;
            for (int n = 0; n < input_index; n++)
            {
                temp = temp * 10 + (input_buffer[n] - '0');
            }
            
            float input_angle = temp / 100.0;

            float rad_angle = input_angle / 180.0 * PI;
            while (rad_angle < 0)
            {
                rad_angle += 2 * PI;
            }
            // convert using the lookup table to counter any magnetic non-linearity
            // a full rotation is index 200
            float index_float = rad_angle / (2 * PI) * 200;
            uint32_t index = (uint32_t)(index_float);
            float ratio = index_float - index;
            index = index % 200;

            target_sensor_angle = angle_lookup_table[index] * (1 - ratio) + angle_lookup_table[(index + 1) % 200] * ratio;

            input_index = 0;
        }
        else if (input_index < 15)
        {
            input_buffer[input_index] = c;
            input_index++;
        }
    }






    int32_t sensor_angle = get_total_sensor_angle();
    int32_t diff = target_sensor_angle - sensor_angle;

    i_term += ((float)diff / MT6835_CPR) * dt / 1000000.0 * 500; // integral term, with time in seconds
    i_term = constrain(i_term, -0.5, 0.5); // anti-windup

    float d_term = (float)(diff - last_diff) / MT6835_CPR / (dt / 1000000.0) * 2; // derivative term
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


    // Serial.println(get_total_sensor_angle());

    
    // delay(10);
    i += 1;
    if (i >= 100)
    {
        i = 0;
        Serial.println(dt);
        Serial.println(torque);
        Serial.println((float)diff / MT6835_CPR * 360, 5);
    }
}
