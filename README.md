# boun_affordance_service


Calculates and publishes the points of affordance on the HDD scene.

Prerequisites:
* This package depends on the boun_perception package. 
* Please follow the instructions there , make sure it is up and running.

Usage:
* Clone the repo into your catkin workspace ```/src``` directory.
* Run ```catkin_make```

* Initialize the service by 
```bash
rosrun boun_affordance_service affordance_srv.py
```

Now the affordance service is available, scene can be queried by providing the HDD and PCB locations.
* Run the example client by following command

```bash
rosrun boun_affordance_service affordance_client.py
```

* Afforded points are published as custom message ```AffordanceList``` from the ```/computed_affordances``` topic.
* Check it by

```
rostopic echo /computed_affordances
```
