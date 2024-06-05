#include <stdio.h>
#include <unistd.h>
#include "fs-ai_api.h"

int main() {
    const char *CAN_interface = "can0";
    int debug = 1;
    int simulate = 0;

    if (fs_ai_api_init(CAN_interface, debug, simulate) != 0) {
        printf("Failed to initialize FS-AI API.\n");
        return -1;
    }

    fs_ai_api_vcu2ai vcu_data;
    fs_ai_api_ai2vcu ai_data;
    ai_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0;
    ai_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0.0;
    ai_data.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0.0;
    ai_data.AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED;
    ai_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
    ai_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
    ai_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;

    while (1) {
        fs_ai_api_vcu2ai_get_data(&vcu_data);
        // Process VCU data
        printf("Steer Angle: %.2f degrees\n", vcu_data.VCU2AI_STEER_ANGLE_deg);

        ai_data.AI2VCU_STEER_ANGLE_REQUEST_deg = vcu_data.VCU2AI_STEER_ANGLE_deg + 1.0;  // Increment steer angle
        fs_ai_api_ai2vcu_set_data(&ai_data);

        usleep(100000);  // 100 ms
    }

    return 0;
}
