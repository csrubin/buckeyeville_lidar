#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * lidar)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = lidar->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        //printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            //fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // lidar->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    /*printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");*/

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3"

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/tty.USB0";
#endif
    }

    // create the driver instance
	RPlidarDriver * lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!lidar) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!lidar)
            lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(lidar->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = lidar->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
            }
            else
            {
                delete lidar;
                lidar = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!lidar)
                lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(lidar->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = lidar->getDeviceInfo(devinfo);

                if (IS_OK(op_result))
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete lidar;
                    lidar = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    /*
    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);


    }

     printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);*/



    // check health...
    if (!checkRPLIDARHealth(lidar)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);

    lidar->startMotor();
    // start scan...
    lidar->startScan(0,1);

    // fetch result and print it out...
    while (1) {

        FILE * outfile;
        outfile = fopen("lidar.csv","w");

        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = lidar->grabScanDataHq(nodes, count);

        if (IS_OK(op_result)) {
            lidar->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("Angle: %03.2f, Dist: %08.2f, Quality: %d \n",
                    nodes[pos].angle_z_q14 * 90.f / (1 << 14),
                    nodes[pos].dist_mm_q2 / 1.f / (1 << 2),
                    nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                fprintf(outfile,"%03.2f, %08.2f, %d \n",
                    nodes[pos].angle_z_q14 * 90.f / (1 << 14),
                    nodes[pos].dist_mm_q2 / 1.f / (1 << 2),
                    nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }

        fclose(outfile);

        if (ctrl_c_pressed){
            break;
        }

        delay(1000); // MODIFY THIS TO CHANGE SAMPLE RATE
    }

    lidar->stop();
    lidar->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
    return 0;
}
