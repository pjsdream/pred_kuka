#include <ros/ros.h>

#include <pcpred/learning/learning_motion.h>

#include <tf/transform_listener.h>

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
        ROS_FATAL("Usage: rosrun pcpred motion_prediction [BENCHMARK_NUMBER] ");
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

    char human_model_filename[128];
    sprintf(human_model_filename, "%s/../human_upper_body.txt", directory);
    FILE* hfp = fopen(human_model_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", human_model_filename);
        return 0;
    }
    fclose(hfp);

    char trained_filename[128];
    sprintf(trained_filename, "%s/trained.txt", directory);
    FILE* ofp = fopen(trained_filename, "r");
    if (ofp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", trained_filename);
        return 0;
    }
    fclose(ofp);

    ros::init(argc, argv, "motion_prediction");
    ROS_INFO("motion_prediction");
    ros::NodeHandle nh;

    double start_time;


    LearningMotionOptions learning_options;
    learning_options.parse(params_filename);
    learning_options.print();
    fflush(stdout);

    LearningMotion trained_motion;
    trained_motion.setVerbose();
    trained_motion.setOptions(learning_options);
    trained_motion.setHumanModelFilename(human_model_filename);

    // use motion
    //trained_motion.loadTrainedData(trained_filename);

    // train
    ROS_INFO("Learning start"); fflush(stdout);
    start_time = ros::Time::now().toSec();
    trained_motion.learn(directory);
    ROS_INFO("Laerning complete in %lf sec\n", ros::Time::now().toSec() - start_time); fflush(stdout);
    trained_motion.saveTrainedData(trained_filename);

    HumanMotionFeature feature;
    feature.setVisualizerTopic("human_motion");
    feature.loadHumanJoints(human_model_filename);


    tf::TransformListener listener;
    ros::Rate rate(15);

    const int state = 0;

    int it = 0;
    while (ros::ok())
    {
        ROS_INFO("iteration %d", it++);
        fflush(stdout);

        Eigen::VectorXd col(feature.numJoints() * 3);

        for (int i=0; i<feature.numJoints(); i++)
        {
            tf::StampedTransform transform;
            while (ros::ok())
            {
                try
                {
                    listener.lookupTransform("map", feature.jointName(i).c_str(),
                                             ros::Time(0), transform);
                    break;
                }
                catch (tf::TransformException ex)
                {
                }
            }

            tf::Vector3 tfx = transform.getOrigin();
            const Eigen::Vector3d x(tfx.x(), tfx.y(), tfx.z());

            col.block(3*i, 0, 3, 1) = x;

            std::cout << feature.jointName(i) << ' ' << x.transpose() << std::endl;
        }

        feature.addFrame(col);
        feature.retainLastFrames(15);
        feature.visualizeHumanMotion();

        if (feature.numFrames() >= 15)
        {
            ROS_INFO("Inference start");
            start_time = ros::Time::now().toSec();
            trained_motion.infer(feature.columnFeature(), state);
            ROS_INFO("Inferen ce completed in %lf sec\n", ros::Time::now().toSec() - start_time); fflush(stdout);

            trained_motion.visualizeInferenceResult();
        }


        rate.sleep();
    }

    return 0;
}
