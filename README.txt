Author: Matthew Mancino
Submission Entry: 1.0
Python Version Used: 2.7
Additional Python libraries used? N/A

Description: I used the Queue class from the Python Queue package to build out the BFS and UCS implementation.
As discussed in class, the BFS and UCS use FIFO Queues to keep track of the current search
throughout the search space. Additionally, I used a stack to building of the IDDFS solution.


For the sensor problem, I designed it so the initial state to be the sensor and
  For a BFS solution, the search algorithm would search through until it found the edge (InfraredBeam) with the shortest
    Euclidean distance between the sensor and the target.
  For the UCS solution, the search algorithm would search the same way however the data structure used would be a PQ
   with the Power loss function used as evaluating a path cost.
  For the IDDFS solution, the 

For the data aggregation problem,
  For the BFS solution,
  For the UCS solution,
  For the IDDFS solution,
