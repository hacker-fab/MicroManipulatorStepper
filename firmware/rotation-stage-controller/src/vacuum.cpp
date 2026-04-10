#include <Arduino.h>

#include "config.h"
#include "vacuum.h"


static uint8_t vacuum_state = 0;

void vacuum_init(){
    pinMode(VAC_PWM_PIN, OUTPUT);
    digitalWrite(VAC_PWM_PIN, LOW);
}

void vacuum_set(uint16_t duty){
    vacuum_state = duty;
    analogWrite(VAC_PWM_PIN, duty * VAC_PWM_MAX / 100);
}
uint16_t vacuum_get(void) {
    return vacuum_state;
}



// Vacuum runtime settings (tuneable by serial commands)
// uint16_t vac_pull_duty = (uint16_t)(VAC_PWM_MAX * 1.00f); // 100%
// uint16_t vac_hold_duty = (uint16_t)(VAC_PWM_MAX * 0.90f); // 90% default
// uint32_t vac_pull_ms   = 800;                             // pulldown duration then switch to hold

// enum VacMode : uint8_t { VAC_OFF=0, VAC_PULLDOWN=1, VAC_HOLD=2 };
// volatile VacMode vac_mode = VAC_OFF;
// uint32_t vac_mode_start_ms = 0;




// -------------------- ADDED: Vacuum functions --------------------
// void vacuum_apply_pwm(uint16_t duty)
// {
//     duty = constrain(duty, 0, VAC_PWM_MAX - 1);
//     analogWrite(VAC_PWM_PIN, duty);
// }

// void vacuum_off()
// {
//     vac_mode = VAC_OFF;
//     vacuum_apply_pwm(0);
// }

// void vacuum_start_pulldown()
// {
//     vac_mode = VAC_PULLDOWN;
//     vac_mode_start_ms = millis();
//     vacuum_apply_pwm(vac_pull_duty);
// }

// void vacuum_hold()
// {
//     vac_mode = VAC_HOLD;
//     vacuum_apply_pwm(vac_hold_duty);
// }

// void vacuum_update()
// {
//     if (vac_mode == VAC_PULLDOWN)
//     {
//         if ((millis() - vac_mode_start_ms) >= vac_pull_ms)
//         {
//             vacuum_hold();
//         }
//     }
// }
