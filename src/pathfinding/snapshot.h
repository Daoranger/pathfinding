// Copyright 2026, Hoang Nguyen
//
// snapshot.h
//
// Defines the Snapshot class, which is used to store the snapshots of
// the grid's state during pathfinding algorithm process. Each snapshot
// store a list frontier cells and the explored cell. The application,
// will then loop through the list of snapshot to draw the frontier cells
// and explore cells for animation.

#ifndef PATHFINDING_SNAPSHOT_H
#define PATHFINDING_SNAPSHOT_H

#include <vector>
#include "../grid/cell.h"


class Snapshot
{
public:
    Snapshot();

    std::vector<Cell*> frontier_;
    std::vector<Cell*> explored_;
};


#endif //PATHFINDING_SNAPSHOT_H