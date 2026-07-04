#include "Cell.h"
#include <limits>

Cell::Cell(const float cellSize, int x, int y)
    : cell_type_(CellType::open)
    , x_(x)
    , y_(y)
    , g_(std::numeric_limits<double>::infinity())
    , h_(0.0)
    , f_(std::numeric_limits<double>::infinity())
    , parent_(nullptr)
{
    square_.setSize(sf::Vector2f(cellSize, cellSize));
    constexpr auto grey = sf::Color(49, 49, 49);
    square_.setOutlineColor(grey);
    square_.setOutlineThickness(5);
}

void Cell::draw(sf::RenderWindow& window)
{
    switch (cell_type_)
    {
        case CellType::start:
            setColor(sf::Color::Green);
            break;
        case CellType::goal:
            setColor(sf::Color::Red);
            break;
        case CellType::obstacle:
            setColor(sf::Color::Black);
            break;
        case CellType::open:
            setColor(sf::Color::White);
            break;
    }

    square_.setFillColor(cell_color_);
    window.draw(square_);
}

void Cell::setPosition(const sf::Vector2f &pos)
{
    square_.setPosition(pos);
}

void Cell::setType(CellType cellType)
{
    cell_type_ = cellType;
}

CellType Cell::getType() const
{
    return cell_type_;
}

void Cell::setColor(sf::Color color)
{
    cell_color_ = color;
}

bool Cell::operator==(const Cell& other) const
{
    return x_ == other.x_ && y_ == other.y_;
}

void Cell::reset()
{
    g_ = std::numeric_limits<double>::infinity();
    h_ = 0.0;
    f_ = std::numeric_limits<double>::infinity();
    parent_ = nullptr;
    cell_type_ = CellType::open;
}
