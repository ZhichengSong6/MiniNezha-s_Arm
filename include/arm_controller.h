#include "dynamixel_sdk.h"
#include <map>
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#define ESC_ASCII_VALUE                 0x1b
#define MINIMUM_POSITION_LIMIT          0                   // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT        4095                // Refer to the Maximum Position Limit of product eManual

namespace  dynamixel
{
class ArmController
{
 private:
    PortHandler *port_;
    PacketHandler *ph_;
    GroupSyncWrite gw_;
    GroupSyncRead gr_;
    int initial_position[4];
    uint8_t param_goal_position[4];
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result
    std::vector<uint8_t>            id_list_;
    std::map<uint8_t, uint16_t>     address_list_;  // <id, start_address>
    std::map<uint8_t, uint16_t>     length_list_;   // <id, data_length>
    std::map<uint8_t, uint8_t *>    data_list_;     // <id, data>
    std::map<uint8_t, uint8_t *>    error_list_;    // <id, error>
    bool last_result_;
    bool is_param_changed_;
    uint8_t *param_;
    uint8_t dxl_error = 0;                            // Dynamixel error
 public:
    int dxl_present_position[4] = {0,0,0,0};
    ArmController(PortHandler *port, PacketHandler *ph, GroupSyncWrite gw, GroupSyncRead gr);
    ~ArmController(){ clearParam();}
    void portSetUp(int baudrate);
    void torqueEnable(int* id, uint16_t address, uint8_t data);
    void torqueDisable(int* id, uint16_t address, uint8_t data);
    void groupSyncReadInitialize(int *id);
    void addPosition(int* id, int* position);
    void readAndGetPosition(int* id, uint16_t address, uint8_t data);
    void writePosition();
    void initialPosition(int* id, uint16_t address, uint8_t data);
    int angleConvert(double angle);
    void closePort();
    void clearParam();
    int getch();
};
}
