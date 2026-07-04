//
// Created by hoang on 2/13/2026.
//

#include "grid.h"
#include "../pathfinding/snapshot.h"
#include <iostream>

Grid::Grid(std::size_t rows, std::size_t cols, float cellSize)
    : rows_(rows)
    , cols_(cols)
    , cell_size_(cellSize)
    , cells_(rows_, std::vector<Cell>(cols_, Cell(cell_size_, 0, 0)))
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            cells_[i][j].x_ = i;
            cells_[i][j].y_ = j;
        }
    }
}

void Grid::draw(sf::RenderWindow &window)
{
    sf::Vector2f offset(
        window.getSize().x / 2.f - cell_size_ * rows_ / 2.f,
        window.getSize().y / 2.f - cell_size_ * cols_ / 2.f
    );

    for (std::size_t row = 0; row < rows_; ++row)
    {
        for (std::size_t col = 0; col < cols_; ++col)
        {
            cells_[row][col].setPosition(
                sf::Vector2f(row * cell_size_, col * cell_size_) + offset
            );
            cells_[row][col].draw(window);
        }
    }
}

std::size_t Grid::getRows() const
{
    return rows_;
}

std::size_t Grid::getCols() const
{
    return cols_;
}

float Grid::getCellSize() const
{
    return cell_size_;
}