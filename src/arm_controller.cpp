#include "arm_controller.h"

using namespace dynamixel;

ArmController::ArmController(PortHandler *port, PacketHandler *ph, GroupSyncWrite gw, GroupSyncRead gr)
  : port_(port),
    ph_(ph),
    gw_(gw),
    gr_(gr),
    last_result_(false),
    is_param_changed_(false),
    param_(0)
{
  clearParam();
}

void ArmController::portSetUp(int baudrate){
    if (port_->openPort())
    {
    printf("Succeeded to open the port!\n");
    }
    else
    {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return;
    }

    // Set port baudrate
    if (port_->setBaudRate(baudrate))
    {
    printf("Succeeded to change the baudrate!\n");
    }
    else
    {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return;
    }
}

void ArmController::torqueEnable(int* id, uint16_t address, uint8_t data){
    for(int i = 0; i < 4; i++){
        dxl_comm_result = ph_->write1ByteTxRx(port_, id[i], address, data, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        printf("1%s\n", ph_->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
        printf("2%s\n", ph_->getRxPacketError(dxl_error));
        }
        else
        {
        printf("Dynamixel#%d has been successfully connected \n", id[i]);
        }
    }
}

void ArmController::torqueDisable(int* id, uint16_t address, uint8_t data){
    for(int i = 0; i < 4; i++){
        dxl_comm_result = ph_->write1ByteTxRx(port_, id[i], address, data, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
        printf("%s\n", ph_->getRxPacketError(dxl_error));
        }
  }
}

void ArmController::groupSyncReadInitialize(int* id){
    for(int i = 0; i < 4; i++){
    dxl_addparam_result = gr_.addParam(id[i]);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", id[i]);
      return;
    }
    }
}

void ArmController::addPosition(int* id, int* position){
    for(int i = 0; i < 4; i++){
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position[i]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position[i]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position[i]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position[i]));
      dxl_addparam_result = gw_.addParam(id[i], param_goal_position);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id[i]);
        return;
      }
    }
}

void ArmController::writePosition(){
    dxl_comm_result = gw_.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
// 
    // Clear syncwrite parameter storage
    gw_.clearParam();
}

void ArmController::readAndGetPosition(int* id, uint16_t address, uint8_t data){
    // Syncread present position
    dxl_comm_result = gr_.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
      }
    else{
        for(int i = 0; i < 4; i++){
            if(gr_.getError(id[i], &dxl_error)){
                printf("[ID:%03d] %s\n", id[i], ph_->getRxPacketError(dxl_error));
                break;
            }
        }
    }
    // Check if groupsyncread data of Dynamixel is available
    for(int i = 0; i < 4; i++){
        dxl_getdata_result = gr_.isAvailable(id[i], address, data);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", id[i]);
            return;
        }
    }
    // Get Dynamixel present position value
    for(int i = 0; i < 4; i++){
    dxl_present_position[i] = gr_.getData(id[i], address, data);
    }

}

void ArmController::initialPosition(int* id, uint16_t address, uint8_t data){
    int dxl_initial_status_threshold = 5;
    initial_position[0] = 3365;
    initial_position[1] = 2048;
    initial_position[2] = 1724;
    initial_position[3] = 2048;
    int status_threshold = 0;
    int tmp_threshold = 0;
    groupSyncReadInitialize(id);
    while(1){
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;
        addPosition(id, initial_position);
        writePosition();
        do{
            readAndGetPosition(id, address, data);
            for(int i = 0; i < 4; i++){
                printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", id[i], initial_position[i], dxl_present_position[i]);
                tmp_threshold = abs(initial_position[i] - dxl_present_position[i]);
                if(tmp_threshold > status_threshold)
                    status_threshold = tmp_threshold; 
            }
        }while(status_threshold > dxl_initial_status_threshold);
    }
}

void ArmController::closePort(){
    port_->closePort();
}

int ArmController::angleConvert(double angle){
    return (angle / 360 * 4095);
}

void ArmController::clearParam(){
    if (id_list_.size() == 0)
    return;

    for (unsigned int i = 0; i < id_list_.size(); i++)
    {
        delete[] data_list_[id_list_[i]];
        delete[] error_list_[id_list_[i]];
    }

    id_list_.clear();
    address_list_.clear();
    length_list_.clear();
    data_list_.clear();
    error_list_.clear();
    if (param_ != 0)
        delete[] param_;
    param_ = 0;
}

int ArmController::getch(){
    #if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
    #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
    #endif
}