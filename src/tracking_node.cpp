#include <memory>

#include "px4_interface.h"
#include "mpc_tracking.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node");

    std::shared_ptr<OffboardMode> offboardMode = 
        std::make_shared<OffboardMode>(new TrackingMpc());

    return offboardMode->mainLoop();
}