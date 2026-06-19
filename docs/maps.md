# SunoBot Maps

## Map

Raw Map Dimensionality: `(X, Y)`

### World and Grid Coordinates

### Update Map and Localize

### Expandability
As this map is built without knowning the full extent of the environment, it is possible the environment is larger that the initial size of the map. If an update step tries to update a map cell that is outside the bounds of the current size of the map, the map will autoexpand in the X and Y dimensions to at least encompass the cell the new lidar readings were trying to update.

### Inflate Obstacles


## Advanced Map

Raw Map Dimensionality: `(X, Y, 2)`

The Advanced Map stores the count of times each cell was recorded as empty or occupied. Using these counter, we compute a probability that the cell is occupied.

This is a more robust solution as a single reading does not permanently define a cell as occupied. A cell is only considered occupied if it strictly has more occupied readings than unoccupied readings. This means the cell must have a probability of being occupied of more than 50%.
 
$`\text{Occupancy Probablity}_{i,j}`$ = $`\frac{\text{Occupied}_{i,j} + 1}{\text{Occupied}_{i,j} + \text{Unoccupied}_{i,j} + 2}`$ 


### Benefits
This offers greater reliability for objects that may have moved through the scene while mapping, such as a human or pet walking, as these entities will eventually be cleared from the map. 

Additionally, this helps to smooth out noisy lidar readings. Suppose a wall is directly forward and exactly 5 meters away. The lidar may read this distance as 4.7 meters which leaves us with a 30 centimeter error. In the previous map implementation, we would mark the cell corresponding to 4.7 meters forward as occupied and move on, never to be updated or corrected. With the implementation in the advanced map, we will increment the occupied counter of the cell 4.7 meters forward. Since there is not really an object there, as we get more accurate readings from the lidar, it will hopefully show the wall being closer to 5 meters away and we will increment the unoccupied counter for the cell 4.7 meters forward, eventually converging to a occupied probability of 0%.

### Computing Frontiers

Using this method we can compute the boundaries of where the robot has explored.
<!-- Example Image of Frontiers -->


## Semantic Map

The Semantic Map adds an additional layer to store information about the semantic information of each obstacle. Semanic information includes what room the obstacle is associated with and what the obstacle is. For example, if the robot registers an oven next to it, it would label the semantic information as (Room: kitchen) & (Object: Oven).

### FloodFill

## Basic Map (Deprecated)?
Why even write this section
