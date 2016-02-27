#include <ros/ros.h>

#include <pcpred/learning/benchmark_generator.h>

#include <tf/transform_broadcaster.h>

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

#include <cstdlib>
#include <vector>

using namespace pcpred;


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        ROS_FATAL("Usage: rosrun pcpred benchmark_generator [BENCHMARK_NUMBER]");
        return 0;
    }

    srand(time(NULL));

    const int benchmark_number = atoi(argv[1]);

    std::string bin_directory = argv[0];
    if (bin_directory.find_last_of('/') == std::string::npos)
        bin_directory = ".";
    else
        bin_directory = bin_directory.substr(0, bin_directory.find_last_of('/'));

    char directory[128];
    sprintf(directory, "%s/../data/B%02d", bin_directory.c_str(), benchmark_number);

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        ROS_FATAL("Failed to access directory [%s]", directory);
        return 0;
    }

    char benchmark_filename[128];
    sprintf(benchmark_filename, "%s/../benchmark%02d.txt", directory, benchmark_number);
    FILE* pfp = fopen(benchmark_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", benchmark_number);
        return 0;
    }
    fclose(pfp);

    ros::init(argc, argv, "benchmark_generator");
    ROS_INFO("benchmark_generator");
    ros::NodeHandle nh;

    BenchmarkGenerator generator;
    generator.loadScenario(benchmark_filename);
    generator.generate(directory);

    return 0;
}
