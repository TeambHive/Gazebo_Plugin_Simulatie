#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "main.h"

#define NUMBER_OF_DIMENSIONS 3


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
	private:double start_time = -1;
	    private:struct Drone drone = {
		{0, 0, 0},
		{0, 0, 0},
		{
		    (struct Goal) {
		        {0, 0, 5},
		        3
		    },
		    (struct Goal) {
		        {-5, 0, 5},
		        6
		},
		    (struct Goal) {
		        {5, 0, 5},
		        9
		},
		    (struct Goal) {
		        {5, -5, 5},
		        12
		},
		    (struct Goal) {
		        {5, 0, 5},
		        15
		},
		    (struct Goal) {
		        {5 * cos(0.5), 5 * sin(0.5), 5},
		        15.25
		},
		    (struct Goal) {
		        {5 * cos(1), 5 * sin(1), 4},
		        15.5
		},
		    (struct Goal) {
		        {5 * cos(1.5), 5 * sin(1.5), 3},
		        15.75
		},
		    (struct Goal) {
		        {5 * cos(2), 5 * sin(2), 2},
		        16
		},
		    (struct Goal) {
		        {5 * cos(2.5), 5 * sin(2.5), 1},
		        16.25
		},
		    (struct Goal) {
		        {5 * cos(3), 5 * sin(3), 2},
		        16.5
		},
		    (struct Goal) {
		        {5 * cos(3.5), 5 * sin(3.5), 3},
		        16.75
		},
		    (struct Goal) {
		        {5 * cos(4), 5 * sin(4), 4},
		        17
		},
		    (struct Goal) {
		        {5 * cos(4.5), 5 * sin(4.5), 5},
		        17.25
		},
		    (struct Goal) {
		        {5 * cos(5), 5 * sin(5), 6},
		        17.5
		},
		    (struct Goal) {
		        {5 * cos(5.5), 5 * sin(5.5), 7},
		        17.75
		},
		    (struct Goal) {
		        {5 * cos(6), 5 * sin(6), 8},
		        18
		},
		    (struct Goal) {
		        {5 * cos(6.5), 5 * sin(6.5), 7},
		        18.25
		},
		    (struct Goal) {
		        {5 * cos(7), 5 * sin(7), 6},
		        18.5
		},
		    (struct Goal) {
		        {5 * cos(7.5), 5 * sin(7.5), 5},
		        18.75
		},
		    (struct Goal) {
		        {5 * cos(8), 5 * sin(8), 4},
		        19
		},
		    (struct Goal) {
		        {0, 0, 5},
		        21
		},
		    (struct Goal) {
		        {0, 0, 0.15},
		        24
		}
		},
		0,
		0
	    };
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }
	//Get the time
	private:double get_time_since_start(){
	    struct timeval currentTime;
	    gettimeofday(&currentTime, NULL);

	    if (start_time == -1) {
		start_time =  ((currentTime.tv_sec * (int)1e6 + currentTime.tv_usec)) / 1e6;
	    }

	    return ((currentTime.tv_sec * (int)1e6 + currentTime.tv_usec)) / 1e6 - start_time;

	}

	//update drone behavior
	private:void update_drone(struct Drone *drone) {

	    double current_time = get_time_since_start();

	    // Update the position of the drone
	    for (int i = 0; i < NUMBER_OF_DIMENSIONS; i++) {
		drone->position[i] += drone->velocity[i] * (current_time - drone->last_updated_time);
	    }

	    // Check if  goal needs to be updated
	    if (drone->goal[drone->goal_index].time < current_time & drone->goal_index + 1 < sizeof(drone->goal)/sizeof(drone->goal[0])) {
		drone->goal_index++;
	    } 
	    // Update velocity of drone
	    for (int i = 0; i < NUMBER_OF_DIMENSIONS; i++) {
		double distance = drone->goal[drone->goal_index].position[i] - drone->position[i];
		double time_left = drone->goal[drone->goal_index].time - get_time_since_start();
		drone->velocity[i] = distance / time_left;
	    }

	    drone->last_updated_time = current_time;

	}

    // Called by the world update start event
    public: void OnUpdate()
    {
	update_drone(&drone);

      // Apply a small linear velocity to the model.
 	this->model->SetLinearVel(ignition::math::Vector3d(drone.velocity[0], drone.velocity[1], drone.velocity[2]));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
