/* stdin: "p v kp kd t" per line; stdout: 8 bytes as hex per line. */
#include <stdio.h>
#include "../app/dm_motor.h"

int main(void)
{
    float p, v, kp, kd, t;
    while (scanf("%f %f %f %f %f", &p, &v, &kp, &kd, &t) == 5) {
        uint8_t b[8];
        dm_encode_mit(b, p, v, kp, kd, t);
        printf("%02X %02X %02X %02X %02X %02X %02X %02X\n",
               b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
    }
    return 0;
}
