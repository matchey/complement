//
// dyawのdrift誤差を表示
//
//
#include <ros/ros.h>
#include "complement/drift_calculator.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "print_drift_dyaw");

	ros::Time::init();
	ros::Rate loop_rate(50);

	driftCalculator dc;

	const string path_runtest = "/home/amsl/ros_catkin_ws/params/run_test";
	int cnt_drift = 0;
	int cnt_per = 1;
	int cnt_max = 3000; //amu: 50hz, 3000[count] -> 60[s]

	if(argc == 2){
		istringstream ss(argv[1]);
		if (!(ss >> cnt_max)) cerr << "Invalid number " << argv[1] << '\n';
	}

	cout << "start..." << endl;
	cout << "[-------------------------]" << endl;
	while(ros::ok()){
		cnt_drift = dc.countGetter();
		cnt_per = int((1.0 * cnt_drift / cnt_max) * 25);
		if(cnt_drift > cnt_max){
			printf("\033[%dA" , 1);
			printf("\033[%dC" , 25);
			cout << "=] 100 %\n" << endl;
			// double drift = dc.driftErrorGetter();
			// ros::param::set("/dyaw/drift", drift);
			// cout << "ros param set [/dyaw/drift] " << drift << endl;
			dc.yamlWriter("/dyaw/drift", path_runtest + "/drift.yaml");
			dc.yamlWriter("/pitch/init", path_runtest + "/pitch_init.yaml");
			cout << "dump \"" << path_runtest << "/drift.yaml\"" << endl;
			cout << "[/dyaw/drift] " << dc.driftErrorGetter() << endl;
			cout << "dump \"" << path_runtest << "/pitch_init.yaml\"" << endl;
			cout << "[/pitch/init] " << dc.pitchInitGetter() << endl;
			break;
		}else if(!cnt_drift){
			printf("\033[%dA" , 1);
			cout << "[-------------------------]" << endl;
		}else{
			printf("\033[%dA" , 1);
			printf("\033[%dC" , 1);
			for(int i=0; i < cnt_per-1 ; ++i){
				printf("=");
			}
			printf(">");
			cout << endl;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

