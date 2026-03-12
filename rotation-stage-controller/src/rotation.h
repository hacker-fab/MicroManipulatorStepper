#if !defined(ROTATION_H)
#define ROTATION_H

int rotation_init(void);
int rotation_poll(uint32_t dt_us);

void rotation_set(float angle_rad);
float rotation_get(void);

uint8_t rotation_home();

void buildAngleLookupTable();
void printAngleLookupTable();

#endif // ROTATION_H
