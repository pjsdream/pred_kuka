#include <ros/ros.h>

#include <pcpred/learning/qlearning.h>
#include <pcpred/learning/learning_motion.h>
#include <pcpred/learning/benchmark_generator.h>
#include <pcpred/learning/future_motion.h>

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
    FILE* pfp = fopen(params_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", params_filename);
        return 0;
    }
    fclose(pfp);

    char learning_filename[128];
    sprintf(learning_filename, "%s/../learning_options.txt", directory);
    FILE* lfp = fopen(learning_filename, "r");
    if (lfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", learning_filename);
        return 0;
    }
    fclose(lfp);

    char human_model_filename[128];
    sprintf(human_model_filename, "%s/../human_upper_body.txt", directory);
    FILE* hfp = fopen(human_model_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", human_model_filename);
        return 0;
    }
    fclose(hfp);

    char benchmark_filename[128];
    sprintf(benchmark_filename, "%s/../benchmark%02d.txt", directory, benchmark_number);
    FILE* bfp = fopen(benchmark_filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", benchmark_number);
        return 0;
    }
    fclose(bfp);

    char bench_directory[128];
    sprintf(bench_directory, "%s/../B%02d", directory, benchmark_number);

    char motion_filename[128];
    sprintf(motion_filename, "%s/../motion.txt", directory);
    FILE* mfp = fopen(motion_filename, "w");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", motion_filename);
        return 0;
    }
    fclose(mfp);


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

    LearningMotionOptions learning_options;
    learning_options.parse(learning_filename);
    learning_options.print();
    fflush(stdout);

    LearningMotion trained_motion;
    trained_motion.setVerbose();
    trained_motion.setOptions(learning_options);
    trained_motion.setHumanModelFilename(human_model_filename);

    double start_time;


    // use motion
    //trained_motion.loadTrainedData(trained_filename);

    // train
    ROS_INFO("Learning start"); fflush(stdout);
    start_time = ros::Time::now().toSec();
    trained_motion.learn(bench_directory);
    ROS_INFO("Learning complete in %lf sec\n", ros::Time::now().toSec() - start_time); fflush(stdout);
    //trained_motion.saveTrainedData(trained_filename);

    HumanMotionFeature feature;
    feature.setVisualizerTopic("human_motion");
    feature.loadHumanJoints(human_model_filename);


    ros::Rate rate(15);

    BenchmarkGenerator generator;
    generator.setDirectory(bench_directory);

    int action = -1;

    int cnt = 0;
    double average_completion_time = 0;
    while (ros::ok())
    {
        mdp.setState(0);

        int state = 0;
        double completion_time = 0;
        while (ros::ok())
        {
            generator.clear();
            if (action != -1)
                generator.appendAction(action);
            else
            {
                for (int i=0; i<5; i++)
                    generator.appendIdle();
            }

            for (int i=0; i<7; i++)
                generator.appendIdle();




            Eigen::MatrixXd ksi = generator.getEpisodeMatrix();
            Eigen::VectorXi actions = generator.getEpisodeActions();

            sprintf(motion_filename, "%s/../motion.txt", directory);
            mfp = fopen(motion_filename, "w");
            fclose(mfp);

            for (int i=14; i<ksi.cols(); i++)
            {
                feature.clearFeature();

                for (int j=0; j<15; j++)
                    feature.addFrame(ksi.col(i - 14 + j));

                trained_motion.infer(feature.columnFeature(), state, actions(i));
                trained_motion.visualizeInferenceResult();
                trained_motion.saveFutureMotion(motion_filename);
            }

            if (mdp.reinforcementLearn() == -1)
                break;

            action = mdp.getLastAction();
            completion_time += mdp.getCompletionTime();

            state |= action;
        }

        //mdp.saveData(filename);

        average_completion_time += completion_time;
        cnt++;

        printf("Completion time: %9.6lf    Average: %9.6lf\n", completion_time, average_completion_time / cnt);
    }

    return 0;
}
