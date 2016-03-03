#include <ros/ros.h>

#include <pcpred/learning/qlearning.h>

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
        ROS_FATAL("Usage: rosrun pcpred task_planner [BENCHMARK_NUMBER]");
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
    sprintf(directory, "%s/../data/mdp", bin_directory.c_str());

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        ROS_FATAL("Failed to access directory [%s]", directory);
        return 0;
    }

    char filename[128];
    sprintf(filename, "%s/qlearning%02d.txt", directory, benchmark_number);
    FILE* fp = fopen(filename, "r");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", filename);
        return 0;
    }
    fclose(fp);

    char params_filename[128];
    sprintf(params_filename, "%s/learning_options.txt", directory);
    FILE* pfp = fopen(filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", filename);
        return 0;
    }
    fclose(pfp);



    ros::init(argc, argv, "task_planner");
    ROS_INFO("task_planner");
    ros::NodeHandle nh;

    QLearning mdp;
    QLearningOptions options;

    options.parse(params_filename);
    options.print();
    fflush(stdout);

    mdp.setOptions(options);
    mdp.loadData(filename);

    mdp.setState(0);
    while (ros::ok())
    {
        mdp.reinforcementLearn();
        //mdp.saveData(filename);

        ros::spinOnce();
    }

    return 0;
}
