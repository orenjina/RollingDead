# Design Lab 2
## Thursday 11/7/2019

### Changes to spec
* Addition of new sensors
* Trees and walls

### Overall framework notes
* A mix of motor schema and DAMN probably
* DAMN to suggest which general behavior to do, such as going to this place or that
* Motor schema to determine the exact action

### Sensor notes
* Option 1: use LIDAR, GPS or receiver, and a camera maybe?
* Option 2: use GPS, receiver, and a high res camera

### Basic behavior ideas
* Explore + Map
  - Default behavior
  - Wander around and store location information of unconsumed food, aqua zombies
* Search for food
* Go to previously seen food
* Avoid zombies (should have high precedence)
  - Different behavior based on zombie type?

### Implementation notes
* Behavior: looking for food, avoiding zombies (Glenn)
* Behavior: Exploring
* Sensor capabilities 
