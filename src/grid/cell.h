// Copyright 2026, Hoang Nguyen
//
// cell.h
//
// Defines the Cell class, which represents a cell in a grid
// and provides functions to render it correctly based on its
// shape, position, and color.

#ifndef PATHFINDING_CELL_H
#define PATHFINDING_CELL_H

#include "SFML/Graphics/RectangleShape.hpp"
#include "SFML/Graphics/RenderWindow.hpp"

enum class CellType
{
    start = 0,
    goal = 1,
    obstacle = 2,
    open = 3,
};

class Cell
{
public:

    // Initialize the data members, as well as
    // set the cell's size, outline thickness and color.
    Cell(float cellSize, int x, int y);

    // Draw the cell's square, the fill color of the cell
    // is set based on the current cell type.
    // start    = green
    // goal     = red
    // open     = white
    // path     = orange
    // frontier = cyan
    // explored = purple
    void draw(sf::RenderWindow& window);

    // Set the cell's square position to draw
    void setPosition(const sf::Vector2f& pos);

    // Set and get cell's type.
    void setType(CellType cellType);
    CellType getType() const;

    // Set the fill color of cell's square.
    void setColor(sf::Color color);

    // Return true if current cell and other cell's x and y are equals
    bool operator==(const Cell& other) const;

    // Reset cell
    void reset();

    int x_;
    int y_;

    double g_;
    double h_;
    double f_;       // f = g + h
    Cell* parent_;

private:
    sf::RectangleShape square_;
    CellType cell_type_;
    sf::Color cell_color_;

};

#endif //PATHFINDING_CELL_H