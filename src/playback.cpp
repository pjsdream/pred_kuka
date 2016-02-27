#include <ros/ros.h>

#include <pcpred/feature/human_motion_feature.h>

#include <sys/stat.h>
#include <unistd.h>

#include <vector>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        ROS_FATAL("Usage: rosrun pcpred playback [ACTION_NUMBER]");
        return 0;
    }

    const int action_number = atoi(argv[1]);

    std::string bin_directory = argv[0];
    if (bin_directory.find_last_of('/') == std::string::npos)
        bin_directory = ".";
    else
        bin_directory = bin_directory.substr(0, bin_directory.find_last_of('/'));

    char directory[128];
    char feature_filename[128];
    char human_filename[128];
    sprintf(directory, "%s/../data", bin_directory.c_str());

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)) && mkdir(directory, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) != 0)
    {
        ROS_FATAL("Failed to make directory [%s]", directory);
        return 0;
    }

    sprintf(human_filename, "%s/human_upper_body.txt", directory);
    FILE* fp = fopen(human_filename, "r");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", human_filename);
        return 0;
    }
    fclose(fp);

    sprintf(directory, "%s/J%02d", directory, action_number);
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)) && mkdir(directory, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) != 0)
    {
        ROS_FATAL("Failed to make directory [%s]", directory);
        return 0;
    }


    ros::init(argc, argv, "playback");
    ROS_INFO("playback");
    ros::NodeHandle nh;

    HumanMotionFeature feature;
    feature.setVisualizerTopic("human_motion");
    feature.loadHumanJoints(human_filename);

    ros::Duration duration(1.0);

    int i=0;
    while (ros::ok())
    {
        sprintf(feature_filename, "%s/sequence%03d.txt", directory, i);
        FILE* fp = fopen(feature_filename, "r");
        if (fp == 0)
            break;
        fclose(fp);

        ROS_INFO("Playback %d\n", i);
        fflush(stdout);

        feature.clearFeature();
        feature.loadFeature(feature_filename);
        feature.visualizeHumanMotion();

        duration.sleep();

        i++;
    }

    ROS_INFO("Playback complete\n");
    fclose(fp);

    return 0;
}
