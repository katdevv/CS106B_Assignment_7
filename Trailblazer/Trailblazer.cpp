/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include "random.h"
using namespace std;

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
Vector<Loc>
shortestPath(Loc start, Loc end,
    Grid<double>& world,
    double costFunction(Loc one, Loc two, Grid<double>& world),
    double heuristic(Loc start, Loc end, Grid<double>& world)) {
    // Result path
    Vector<Loc> path;

    // Locs to be processed
    TrailblazerPQueue<Loc> yellow;
     
    // Processed (visited) Locs
    Set<Loc> green;

    // Init candidate distances (aka costs from start to Loc)
    Map<Loc, double> costs;
    costs[start] = 0;

    // Store each Loc's parent
    Map<Loc, Loc> parents;

    // Color start yellow
    yellow.enqueue(start, heuristic(start, end, world));
    colorCell(world, start, YELLOW);

    while (!yellow.isEmpty()) {
        // Loc to process
        Loc curLoc = yellow.dequeueMin();

        // Push and color Loc green
        green.add(curLoc);
        colorCell(world, curLoc, GREEN);

        // End found
        if (curLoc == end) break;
        
        // Check neighbours
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i == 0 && j == 0) continue;
                Loc neighbour = makeLoc(curLoc.row + i, curLoc.col + j);

                if (!world.inBounds(neighbour.row, neighbour.col) 
                    || green.contains(neighbour)) continue;

                // Calculate cost to get to the neighbour
                double newCost = costs[curLoc] + costFunction(curLoc, neighbour, world);
               
                // Estiamate cost from neighbour to end 
                double newH = heuristic(neighbour, end, world);
                
                // If the neigbour hasn't been discovered (is gray)
                if (!costs.containsKey(neighbour)) {
                    costs[neighbour] = newCost;
                    parents[neighbour] = curLoc;
                    yellow.enqueue(neighbour, newCost + newH);
                    colorCell(world, neighbour, YELLOW);
                }
                // If the neigbour is already yellow, but a better path to it has been found
                else if (newCost < costs[neighbour]) {
                    costs[neighbour] = newCost;
                    parents[neighbour] = curLoc;
                    yellow.decreaseKey(neighbour, newCost + newH); 
                }
            }
        }
    }
	
    // Error handling
    if (!parents.containsKey(end)) error("No path found");

    // Reconstruct path
    Loc current = end;
    while (current != start) {
        path.insert(0, current);
        current = parents[current];
    }
    path.insert(0, start);
    
    return path;
}

Loc findParent(Loc point, Map<Loc, Loc>& parents) {
    // Find the parent of the cluster
    if (parents[point] != point)
        parents[point] = findParent(parents[point], parents);
    return parents[point];
}

void unitePoints(Loc one, Loc two, Map<Loc, Loc>& parents) {
    // Merge the two clusters
    if (one != two) {
        parents[findParent(one, parents)] = findParent(two, parents);
    }
}

Set<Edge> createMaze(int numRows, int numCols) {
    Set<Edge> edges;

    // For clusters
    Map<Loc, Loc> parents; 

    // Create edges and a grid graph
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            Loc curLoc = makeLoc(i, j);

            // Place each node into its own cluster
            parents[curLoc] = curLoc;

            // Connect to the right Loc
            if (j + 1 < numCols) {
                Loc rightLoc = makeLoc(i, j + 1);
                Edge newEdge = makeEdge(curLoc, rightLoc);
                edges.add(newEdge);
            }
            
            // Connect to the bottom Loc
            if (i + 1 < numRows) {
                Loc downLoc = makeLoc(i + 1, j);
                Edge newEdge = makeEdge(curLoc, downLoc);
                edges.add(newEdge);
            }
        }
    }

    // Add edges to pqueue (with random weights)
    TrailblazerPQueue<Edge> pqueue;
    for (Edge e : edges) {
        pqueue.enqueue(e, randomInteger(0, 10));
    }
    
    // The result - spanning tree/maze
    Set<Edge> maze;

    // Kruskal's Algorithm
    int numCluster = numRows * numCols;

    while (numCluster > 1) {
        Edge curEdge = pqueue.dequeueMin();
        // If the endpoints are not in the same cluster
        if (findParent(curEdge.start, parents) != findParent(curEdge.end, parents)) {
            // Merge their clusters
            unitePoints(curEdge.start, curEdge.end, parents);

            // Add edge to the tree
            maze.add(curEdge);
            numCluster--;
        }
    }
    
    return maze;
}