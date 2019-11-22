HOW TO USE:
- First you need to INITIALIZE the map by calling initialize() function
- Set the variable cameraMaxView to how long you estimate the camera sees into the distance
- For every timestamp:
  - You need to call clearSector("LEFT"), clearSector("RIGHT"), clearSector("UP"), clearSector("DOWN")
    - This just removes what you can see from each camera view point before you replace it
  - Then use addToMap(object, x, y) for every object that is seen from the cameras
    - x,y are relative to the robot being at (0,0)
    - NOTE: you will need to determine if the map needs to be expanded if the x,y values are too large for the current map
    
    
LEFT TO DO:
- Add different object options for the zombies/berries
- Add object options for the walls/edge
- Determine when to expand the map



Basics:  A 2D array of struct pointers are created with dimensions MAP_HEIGHT and MAP_WIDTH. While the 2D array is litterally indexed by row and column, it can be described instead as (x,y) coordinates with respect to the centerpoint (0,0). In this way, the map can always be displayed with the robot at the center point (0,0). Each node in the map is a mapNode struct that has an object (and some other junk that is only important for other functions), and will probably be added to later on.

Explanation of functions used:

initialize(height, width):  Sets the starting dimensions of the map depending on args. Mallocs space for central map (a 2D array of struct mapNode pointers).

expand(amount, direction): When the robot gets close to the edge of the map, it will need to be expanded. Direction of “UP” for +y (as in now there are X more spots with coordinate in positive y direction), “DOWN” for -y, “RIGHT” for +x, and “LEFT” for -x Method:  Add new rows or columns. In case of “UP” or “LEFT”, then shift all objects to the bottom/right by X amount.

 printMap():  Debugging tool that will print color coded map with all the objects on it.

addToMap(object, x, y):  Adds the object to coordinate x, y. For now it will erase the old one.

clearSector(direction): Clears objects from field of view of camera in direction specified.

Testing:  I scrape the command line arguments and convert them into objects on the map: ./map [object x y]* Here x and y are the coordinates for the object. The object is either “b” for berry, or “z” for zombie.

To do: What to do if try and add object but something already there?  Could have array of objects in every grid space, with order determining which on in front. (This problem would be a result of having imperfect location data of objects)
