#if !defined(ROTATION_H)
#define ROTATION_H

int rotation_init(void);
int rotation_poll(uint32_t dt_us);

// set target angle
void rotation_set(float angle_rad);
// get current angle
float rotation_get(void);
// get target angle
float rotation_target_get(void);

uint8_t rotation_home();

void buildAngleLookupTable();
void printAngleLookupTable();

#endif // ROTATION_H
