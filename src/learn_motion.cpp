#include <ros/ros.h>

#include <pcpred/learning/learning_motion.h>

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
        ROS_FATAL("Usage: rosrun pcpred learn_motion [BENCHMARK_NUMBER]");
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

    char params_filename[128];
    sprintf(params_filename, "%s/../learning_options.txt", directory);
    FILE* pfp = fopen(params_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", params_filename);
        return 0;
    }
    fclose(pfp);

    ros::init(argc, argv, "learn_motion");
    ROS_INFO("learn_motion");
    ros::NodeHandle nh;

    double start_time;


    LearningMotionOptions learning_options;
    learning_options.parse(params_filename);
    learning_options.print();
    fflush(stdout);

    LearningMotion learning_motion;
    learning_motion.setVerbose();
    learning_motion.setOptions(learning_options);

    ROS_INFO("Learning start"); fflush(stdout);
    start_time = ros::Time::now().toSec();
    learning_motion.learn(directory);
    ROS_INFO("Laerning complete in %lf sec\n", ros::Time::now().toSec() - start_time); fflush(stdout);

    return 0;
}
