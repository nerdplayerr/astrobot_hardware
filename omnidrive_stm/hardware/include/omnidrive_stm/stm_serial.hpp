#ifndef __ASTROBOT_HARDWARE__ASTROBOT_SERIAL_PORT_H__
#define __ASTROBOT_HARDWARE__ASTROBOT_SERIAL_PORT_H__

#include <string>
#include <vector>

// #define ASTROBOT_SERIAL_BUFFER_MAX_SIZE           200
// #define ASTROBOT_SERIAL_SERIAL_FRAME_MAX_SIZE     100


namespace astrobot
{
    namespace hardware
    {
        enum class return_type : std::uint8_t
        {
            SUCCESS = 0,
            ERROR = 1
        };

        // struct SerialHdlcFrame
        // {
        //     uint8_t data[ASTROBOT_SERIAL_SERIAL_FRAME_MAX_SIZE];
        //     size_t length;
        // };

        typedef struct {
            uint16_t start;
            int16_t  posX;
            int16_t  posY;
            int16_t  orientation;
            int16_t  speed;
            uint16_t checksum;
            
        } SerialCommand;

        typedef struct {
            uint16_t start;
            int16_t  cmd0;
            int16_t  cmd1;
            int16_t  cmd2;
            int16_t  cmd3;
            int16_t  speed0_meas;
            int16_t  speed1_meas;
            int16_t  speed2_meas;
            int16_t  speed3_meas;
            int16_t  wheel0_cnt;
            int16_t  wheel1_cnt;
            int16_t  wheel2_cnt;
            int16_t  wheel3_cnt;  
            int16_t  batVoltage;
            // int16_t  boardTemp;
            // uint16_t cmdLed;
            uint16_t checksum;
        } SerialFeedback;

        class AstrobotSerialPort
        {
        public:
            AstrobotSerialPort();
            ~AstrobotSerialPort();
            
            return_type open(const std::string & port_name);
            return_type close();
            // SerialFeedback msg, prev_msg;
            // return_type read_frames(std::vector<SerialFeedback>& frames);
            return_type read_frames();
            // return_type write_frame(const uint8_t* data, size_t size);
            // return_type write_frame();
            return_type write_frame(const int pos_x, const int pos_y, const int ori, const int spd);
            bool is_open() const;

            double wheel0_hall;
            double wheel1_hall;
            double wheel2_hall;
            double wheel3_hall;
            int vel0;
            int vel1;
            int vel2;
            int vel3;

        protected:
            // void encode_byte(uint8_t data);
            // void decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames);
            // uint16_t crc_update(uint16_t crc, uint8_t data);

        private:
            void protocol_recv(uint8_t byte);
            int msg_len = 0;
            uint8_t prev_byte = 0;
            uint16_t start_frame = 0;
            uint8_t* p;
            SerialFeedback msg, prev_msg;

            int serial_port_;
            // uint8_t rx_buffer_[ASTROBOT_SERIAL_BUFFER_MAX_SIZE];
            // uint8_t rx_frame_buffer_[ASTROBOT_SERIAL_SERIAL_FRAME_MAX_SIZE];
            // size_t rx_frame_length_;
            // uint16_t rx_frame_crc_;
            // bool rx_frame_escape_;
            // uint8_t tx_frame_buffer_[ASTROBOT_SERIAL_SERIAL_FRAME_MAX_SIZE];
            // size_t tx_frame_length_;
            // uint16_t tx_frame_crc_;

        };
    }
}

#endif // __ASTROBOT_HARDWARE__ASTROBOT_SERIAL_PORT_H__