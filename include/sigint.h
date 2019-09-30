#include <signal.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void mySigIntHandler(int sig){ g_request_shutdown = 1; }


// In the main node:
/*
	// Override SIGINT handler
	ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);

  .
  .
  .

  	while(!g_request_shutdown)
    {
      ...
      ros::spinOnce();
    }

    ros::shutdown();

*/