#include <ros/ros.h>

#include <pcpred/learning/learning_motion.h>
#include <pcpred/learning/benchmark_generator.h>

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
        ROS_FATAL("Usage: rosrun pcpred validation [BENCHMARK_NUMBER] ");
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

    /*
    char trained_filename[128];
    sprintf(trained_filename, "%s/trained.txt", directory);
    FILE* ofp = fopen(trained_filename, "r");
    if (ofp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", trained_filename);
        return 0;
    }
    fclose(ofp);
    */

    char benchmark_filename[128];
    sprintf(benchmark_filename, "%s/../benchmark%02d.txt", directory, benchmark_number);
    FILE* bfp = fopen(benchmark_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", benchmark_number);
        return 0;
    }
    fclose(bfp);

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
    ROS_INFO("Learning complete in %lf sec\n", ros::Time::now().toSec() - start_time); fflush(stdout);
    //trained_motion.saveTrainedData(trained_filename);

    HumanMotionFeature feature;
    feature.setVisualizerTopic("human_motion");
    feature.loadHumanJoints(human_model_filename);


    ros::Rate rate(15);

    BenchmarkGenerator generator;
    generator.loadScenario(benchmark_filename);

    while (ros::ok())
    {
        int state = 0;

        generator.generateValidationMatrix(directory);
        Eigen::MatrixXd ksi = generator.getEpisodeMatrix();
        Eigen::VectorXi actions = generator.getEpisodeActions();

        for (int i=0; i<learning_options.input_frames - 1; i++)
            state |= 1 << actions(i);

        for (int i=learning_options.input_frames - 1; i<ksi.cols(); i++)
        {
            state |= 1 << actions(i);

            feature.clearFeature();

            for (int j=0; j<learning_options.input_frames; j++)
                feature.addFrame(ksi.col(i - learning_options.input_frames + 1 + j));

            trained_motion.infer(feature.columnFeature(), state, actions(i));
            trained_motion.visualizeInferenceResult();

            feature.visualizeCurrentHumanMotion();

            rate.sleep();
        }
    }

    return 0;
}
