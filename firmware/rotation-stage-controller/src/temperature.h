#if !defined(TEMPERATURE_H)
#define TEMPERATURE_H

int temperature_init(void);
int temperature_poll(uint32_t dt_us);

void temperature_set(float temp);
float temperature_get(void);
float temperature_target_get(void);

#endif // TEMPERATURE_H
