#ifndef DJI_RONIN_CONTROLLER_HPP
#define DJI_RONIN_CONTROLLER_HPP

#include <ESP32-TWAI-CAN.hpp>
#include <Arduino.h>

#define DEBUGF(...) // Serial.printf(__VA_ARGS__)
#define WARNF(...) Serial.printf(__VA_ARGS__)

#define FRAME_LEN 8
#define NORMAL_SEND 0
#define DATA_FRAME 1
#define STD_FRAME 0

#define MASK_BIT_TYPE 0xFF
#define MAX_INT_8 256

class DJIRoninController {
    private:
        TwaiCAN esp32Can;
        int8_t tx_pin;
        int8_t rx_pin;
        int16_t rate_speed;
        uint16_t seq_num = 0x0002;
        uint32_t can_send_id = 0x223;
        uint32_t can_recv_id = 0x222;

        bool _send_data(uint8_t *data, size_t data_len);
        void _calc_crc16(const uint8_t *hex_seq, size_t length, uint8_t result[2]);
        void _calc_crc32(const uint8_t *hex_seq, size_t length, uint8_t result[4]);
        size_t _assemble_can_msg(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, const uint8_t* data, size_t data_len, uint8_t *can_msg);
        uint16_t _seq_num();
    public:
        DJIRoninController(int txPin, int rxPin, int rateSpeed) { tx_pin = txPin; rx_pin = rxPin; rate_speed=rateSpeed; }
        bool begin();
        bool set_position(float yaw, float roll, float pitch, bool absolute_position, uint16_t time_for_action_in_millis);
        void read_frame();
};

#endif
