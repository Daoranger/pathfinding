// Copyright 2026, Hoang Nguyen
//
// grid.h
//
// Defines the Grid class, which represents the 2D grid consisted
// of a m x n cells, uses for interaction and visualization

#ifndef PATHFINDING_GRID_H
#define PATHFINDING_GRID_H

#include "grid.h"

#include <cmath>
#include <set>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include "../pathfinding/snapshot.h"
#include "cell.h"

class Grid
{
public:

    // Construct cells on grid and assign them with
    // their corresponding row and column on the grid
    Grid(std::size_t rows, std::size_t cols, float cellSize);

    // Use offset to draw the grid in the middle of the window.
    // Loop through each cell in the grid and draw at the correct
    // position
    void draw(sf::RenderWindow& window);

    // Get number of rows in the grid
    std::size_t getRows() const;

    // Get number of columns in the grid
    std::size_t getCols() const;

    // Get cell size
    float getCellSize() const;

private:
    std::size_t rows_;
    std::size_t cols_;
    float cell_size_;

public:
    std::vector<std::vector<Cell>> cells_;
};


#endif //PATHFINDING_GRID_H