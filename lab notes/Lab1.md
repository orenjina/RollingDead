# Design Lab 1
## Monday 11/4/2019

### Changes to spec
* Addition of new zombie types (blue, purple, aqua, etc.)
* Local variables

### Overall framework notes
* Unclear whether to use subsumption-style or motor schema-style architecture
* Resolved to get basic behaviors running before determining exactly how they'll interact

### Basic behavior ideas
* Explore + Map
  - Default behavior
  - Wander around and store location information of unconsumed food, aqua zombies
* Search for food
* Go to previously seen food
* Avoid zombies (should have high precedence)
  - Different behavior based on zombie type?

### Implementation notes
* Basic vision capabilities using color thresholding (Kaleb)
* Basic robot controls, simple exploration behavior (Glenn)
