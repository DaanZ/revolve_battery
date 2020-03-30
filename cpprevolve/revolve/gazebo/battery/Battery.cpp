//
// Created by Roy Basmacier on 2019-07-09.
//

#include "Battery.h"

using namespace revolve::gazebo;

Battery::Battery(double initial_charge)
    : initial_charge(initial_charge), current_charge(initial_charge), time_init(std::to_string(time(0))), robot_name("")
    {}

void Battery::Update(double global_time, double delta_time)
{
    double sum = 0.0;
    for (const auto &consumer: this->PowerLoads()) {
        sum += consumer.second; // TODO add constant so its linear
    }
    this->current_charge += sum * delta_time; // charge is measured in joules

    //TODO properly save battery data somewhere
    std::ofstream b_info_file;
    b_info_file.open("output/cpg_bo/" + this->robot_name + "/" + this->time_init + "/battery.txt", std::ios_base::app);
    if (b_info_file.fail())
        std::cout << "Failed to open: " << b_info_file.fail() <<  " " << "output/cpg_bo/" + this->robot_name + "/" + this->time_init + "/battery.txt" << std::endl;
    b_info_file << global_time << " " << sum << " " << current_charge << std::endl;
}
