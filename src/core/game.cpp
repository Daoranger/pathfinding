#include "../core/game.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <future>
#include <thread>
#include <fstream>

#include "imgui.h"
#include "imgui-SFML.h"

#include <omp.h>

#include "SFML/Audio/Music.hpp"

Game::Game()
    : window_(sf::VideoMode( { 1200, 700 } ), "A* Pathfinding")
    , view_(sf::FloatRect(sf::Vector2f(0, 0), sf::Vector2f(window_.getPosition().x, window_.getPosition().y)))
    , grid_(50, 50, 50)
{
    window_.setView(view_);
    ImGui::SFML::Init(window_);
}

void Game::run()
{
    while (window_.isOpen() )
    {
        processEvents();
        update();
        draw();
    }
}

void Game::processEvents()
{
    while ( const std::optional event = window_.pollEvent() )
    {
        ImGui::SFML::ProcessEvent(window_, *event);

        if (event->is<sf::Event::Closed>())
            window_.close();

        // on resized event
        if (const auto* resizedEvent = event->getIf<sf::Event::Resized>())
        {
            // created a view from fixed visible area using FloatRect
            sf::FloatRect visibleArea(sf::Vector2f(0, 0), sf::Vector2f(resizedEvent->size.x, resizedEvent->size.y));
            view_ = sf::View(visibleArea);
            window_.setView(view_);
        }

        // Mouse Button Pressed Event
        if (const auto* mouseButtonPressedEvent = event->getIf<sf::Event::MouseButtonPressed>())
        {
            is_dragging_ = true;

            sf::Vector2i pixelPos = mouseButtonPressedEvent->position;
            sf::Vector2f worldPos = window_.mapPixelToCoords(pixelPos, view_);

            sf::Vector2f gridOffset = getGridOffset();
            sf::Vector2f localPos = worldPos - gridOffset;

            onMouseClick(*mouseButtonPressedEvent, localPos);
        }

        // Mouse Button Released Event
        if (const auto* mouseButtonReleasedEvent = event->getIf<sf::Event::MouseButtonReleased>())
        {
            is_dragging_ = false;
        }

        // Mouse Drag Event
        if (const auto* mouseMovedEvent = event->getIf<sf::Event::MouseMoved>())
        {
            sf::Vector2i pixelPos = mouseMovedEvent->position;
            sf::Vector2f worldPos = window_.mapPixelToCoords(pixelPos, view_);

            sf::Vector2f gridOffset = getGridOffset();
            sf::Vector2f localPos = worldPos - gridOffset;

            if (is_dragging_)
            {
                onDrag(*mouseMovedEvent, localPos);
            }
        }

        // Mouse Wheel Scrolled Event
        if (const auto* mouseWheelScrolled = event->getIf<sf::Event::MouseWheelScrolled>())
        {
            if (mouseWheelScrolled->delta > 0)
            {
                view_.zoom(0.9f);
            }
            else if (mouseWheelScrolled->delta < 0)
            {
                view_.zoom(1.1f);
            }

            window_.setView(view_);
        }
    }
}

void Game::update()
{
    if (game_mode_ == GameMode::kSinglePathfinding && app_state_ == AppState::kAnimating
        && !agents_.empty())
    {
        if (snapshot_clock_.getElapsedTime().asSeconds() > delay_)
        {
            bool all_done = true;
            for (auto& agent : agents_)
            {
                if (agent->snapshots_.empty())
                {
                    continue;
                }
                if (agent->snapshot_index_ < static_cast<int>(agent->snapshots_.size()) - 1)
                {
                    agent->snapshot_index_++;
                    all_done = false;
                }
            }
            if (all_done)
                app_state_ = AppState::kDone;
            snapshot_clock_.restart();
        }
    }

    ImGui::SFML::Update(window_, imgui_clock_.restart());
    renderImGuiPanels();
}

void Game::renderImGuiPanels()
{
    ImGuiIO& io = ImGui::GetIO();
    // repeat=false: OS key-repeat would otherwise toggle every repeat tick while F1 is held.
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
        show_pathfinding_panel_ = !show_pathfinding_panel_;
    }

    const float edgeMargin = 16.f;

    if (!show_pathfinding_panel_)
    {
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - edgeMargin, io.DisplaySize.y * 0.5f),
                                ImGuiCond_Always, ImVec2(1.f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(48.f, 112.f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f, 0.12f, 0.16f, 0.92f));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.42f, 0.72f, 1.f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.52f, 0.88f, 1.f));
        ImGui::Begin("##panel_toggle", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar
                         | ImGuiWindowFlags_NoMove);
        ImGui::SetWindowFontScale(1.5f);
        if (ImGui::Button("<<", ImVec2(36.f, 96.f)))
        {
            show_pathfinding_panel_ = true;
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Show panel (F1)");
        }
        ImGui::End();
        ImGui::PopStyleColor(3);
        ImGui::PopStyleVar();
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - edgeMargin, edgeMargin), ImGuiCond_Always,
                            ImVec2(1.f, 0.f));
    {
        const float panelW = 480.f;
        const float panelH = std::max(420.f, io.DisplaySize.y - 2.f * edgeMargin);
        ImGui::SetNextWindowSize(ImVec2(panelW, panelH), ImGuiCond_FirstUseEver);
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 12.f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(26, 22));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(12, 14));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14, 10));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.07f, 0.08f, 0.11f, 0.96f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.28f, 0.50f, 0.78f, 0.45f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.10f, 0.22f, 0.38f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.14f, 0.36f, 0.62f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.42f, 0.72f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.52f, 0.88f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.12f, 0.32f, 0.55f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.16f, 0.30f, 0.48f, 0.65f));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.22f, 0.40f, 0.62f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.35f, 0.48f, 0.65f, 0.45f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.12f, 0.14f, 0.20f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.16f, 0.20f, 0.28f, 1.f));

    ImGui::Begin("Pathfinding", nullptr, ImGuiWindowFlags_NoMove);

    ImGui::SetWindowFontScale(1.6f);

    if (ImGui::Button("Hide panel", ImVec2(-1.f, 44.f)))
    {
        show_pathfinding_panel_ = false;
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Or press F1");
    }
    ImGui::TextDisabled("F1: show / hide panel");
    ImGui::Spacing();

    ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.f, 1.f), "Scene");
    ImGui::Separator();
    ImGui::Spacing();

    int modePick = (game_mode_ == GameMode::kSinglePathfinding) ? 0 : 1;
    ImGui::RadioButton("Single agent", &modePick, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Multi agent", &modePick, 1);
    if (modePick == 0 && game_mode_ != GameMode::kSinglePathfinding)
    {
        game_mode_ = GameMode::kSinglePathfinding;
        reset();
    }
    else if (modePick == 1 && game_mode_ != GameMode::kMultiPathfinding)
    {
        game_mode_ = GameMode::kMultiPathfinding;
        reset();
    }

    if (game_mode_ == GameMode::kMultiPathfinding)
    {
        ImGui::Spacing();
        ImGui::TextUnformatted("Multi-agent");
        ImGui::PushItemWidth(-1.f);
        const int prevCount = multi_agent_count_;
        int countEdit = multi_agent_count_;
        ImGui::InputInt("Agent count", &countEdit, 1, 100);
        multi_agent_count_ = std::clamp(countEdit, 1, kMaxMultiAgents);
        ImGui::PopItemWidth();

        ImGui::TextUnformatted("Presets");
        static constexpr int kCountPresets[] = { 10, 100, 1000, 10000 };
        static constexpr const char* kPresetLabels[] = { "10", "100", "1k", "10k" };
        constexpr std::size_t kPresetCount = sizeof(kCountPresets) / sizeof(kCountPresets[0]);
        for (std::size_t i = 0; i < kPresetCount; ++i)
        {
            if (i > 0)
            {
                ImGui::SameLine();
            }
            char id[48];
            std::snprintf(id, sizeof(id), "%s##preset%zu", kPresetLabels[i], i);
            if (ImGui::SmallButton(id))
            {
                multi_agent_count_ = kCountPresets[i];
            }
        }

        ImGui::TextDisabled("Same start/goal for every agent (corner to corner). Max %d.", kMaxMultiAgents);
        if (prevCount != multi_agent_count_ && app_state_ == AppState::kIdle)
        {
            agents_.clear();
            agents_.shrink_to_fit();
            if (multi_agent_count_ <= kEagerMultiAgentBuildLimit)
            {
                initAgents();
            }
        }
    }

    ImGui::Spacing();
    const bool prevSnapshots = record_search_snapshots_;
    ImGui::Checkbox("Record search snapshots", &record_search_snapshots_);
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Stores frontier/explored each step for animation. Off = faster (path only).");
    }
    if (prevSnapshots != record_search_snapshots_ && game_mode_ == GameMode::kMultiPathfinding
        && app_state_ == AppState::kIdle)
    {
        agents_.clear();
        agents_.shrink_to_fit();
        if (multi_agent_count_ <= kEagerMultiAgentBuildLimit)
        {
            initAgents();
        }
    }

    ImGui::Spacing();
    int brushPick = (input_mode_ == InputMode::kSelecting) ? 0 : 1;
    ImGui::TextUnformatted("Brush");
    ImGui::RadioButton("Paint##brush", &brushPick, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Erase##brush", &brushPick, 1);
    if (brushPick == 0 && input_mode_ != InputMode::kSelecting)
    {
        input_mode_ = InputMode::kSelecting;
    }
    else if (brushPick == 1 && input_mode_ != InputMode::kDeselecting)
    {
        input_mode_ = InputMode::kDeselecting;
    }

    if (game_mode_ == GameMode::kSinglePathfinding)
    {
        ImGui::Spacing();
        const char* hint = placement_state_ == PlacementState::kNeedsStart
            ? "Next: click to place the start cell."
            : placement_state_ == PlacementState::kNeedsGoal
                  ? "Next: click to place the goal cell."
                  : "Paint obstacles; drag to paint many cells.";
        ImGui::TextWrapped("%s", hint);
    }

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.f, 1.f), "Actions");
    ImGui::Separator();
    ImGui::Spacing();

    const bool canRun = app_state_ == AppState::kIdle
        && ((game_mode_ == GameMode::kSinglePathfinding
                && placement_state_ == PlacementState::kPlacingObstacles)
            || game_mode_ == GameMode::kMultiPathfinding);

    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Run search", ImVec2(-1.f, 50.f)))
    {
        runAlgorithm();
        app_state_ = game_mode_ == GameMode::kMultiPathfinding ? AppState::kDone : AppState::kAnimating;
    }
    ImGui::EndDisabled();
    if (!canRun && app_state_ == AppState::kIdle && game_mode_ == GameMode::kSinglePathfinding)
    {
        ImGui::TextDisabled("Place start, then goal, before running.");
    }

    if (ImGui::Button("Reset", ImVec2(-1.f, 42.f)))
    {
        reset();
    }

    ImGui::Spacing();
    if (ImGui::Button("Save grid to Grid.txt", ImVec2(-1.f, 32.f)))
    {
        saveGridToFile();
    }
    if (ImGui::Button("Load grid from Grid.txt", ImVec2(-1.f, 40.f)))
    {
        loadGridFromFile();
    }

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.f, 1.f), "Algorithm");
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::PushItemWidth(-1.f);
    static constexpr std::array<const char*, 6> kAlgorithmLabels = {
        "A*", "Dijkstra", "Greedy", "BFS", "DFS", "BA*"
    };
    int algoIndex = static_cast<int>(algorithm_);
    ImGui::Combo("##algo", &algoIndex, kAlgorithmLabels.data(), static_cast<int>(kAlgorithmLabels.size()));
    algorithm_ = static_cast<Algorithm>(algoIndex);
    ImGui::PopItemWidth();

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.f, 1.f), "Heuristic (A*, Greedy, BA*)");
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Checkbox("Tie-break nudge (×1.001 on h)", &heuristic_tie_nudge_);
    ImGui::SliderInt("h weight (f = g + w·h)", &heuristic_weight_, 1, 10, "%d");
    ImGui::TextDisabled("w = 1: admissible A*. w > 1: weighted (faster, may be suboptimal).");

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.f, 1.f), "Parallel strategy");
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::PushItemWidth(-1.f);
    static constexpr std::array<const char*, 3> kParallelLabels = {
        "Sequential", "Per-agent thread", "OpenMP"
    };
    int parallelIndex = static_cast<int>(parallel_strategy_);
    ImGui::Combo("##par", &parallelIndex, kParallelLabels.data(),
                 static_cast<int>(kParallelLabels.size()));
    parallel_strategy_ = static_cast<ParallelStrategy>(parallelIndex);
    ImGui::PopItemWidth();

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.f, 1.f), "Metrics");
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Path found:     %s", metrics_.path_found ? "yes" : "no");
    ImGui::Text("Path length:    %zu", metrics_.path_size);
    if (game_mode_ == GameMode::kSinglePathfinding)
    {
        ImGui::Text("Nodes expanded: %zu", metrics_.nodes_expanded);
    }
    ImGui::Text("Search time:    %.2f ms", metrics_.search_time);

    ImGui::End();

    ImGui::PopStyleColor(12);
    ImGui::PopStyleVar(6);
}

void Game::draw()
{
    window_.clear(sf::Color::White);
    grid_.draw(window_);
    drawAgents();
    ImGui::SFML::Render(window_);
    window_.display();
}

void Game::drawAgents()
{
    if (game_mode_ == GameMode::kMultiPathfinding)
    {
        //return;
    }
    for (auto& agent : agents_)
    {
        drawAgent(*agent);
    }
}

void Game::drawAgent(Agent& agent)
{
    sf::Color color = agent.getColor();
    sf::Vector2f offset = getGridOffset();
    sf::RectangleShape overlay(sf::Vector2f(grid_.getCellSize(), grid_.getCellSize()));

    const Cell* startCell = agent.getStartCell();
    const Cell* goalCell = agent.getGoalCell();

    if (!startCell || !goalCell) return;

    color.a = 255;
    overlay.setFillColor(color);
    overlay.setPosition(sf::Vector2f(startCell->x_ * grid_.getCellSize(), startCell->y_ * grid_.getCellSize()) + offset);
    window_.draw(overlay);

    color.a = 128;
    overlay.setFillColor(color);
    overlay.setPosition(sf::Vector2f(goalCell->x_ * grid_.getCellSize(), goalCell->y_ * grid_.getCellSize()) + offset);
    window_.draw(overlay);

    switch (app_state_)
    {

        case AppState::kAnimating:
        {
            if (agent.snapshots_.empty()) return;
            Snapshot& snapshot = agent.snapshots_[agent.snapshot_index_];

            color.a = 128;
            overlay.setFillColor(color);
            for (Cell* cell : snapshot.frontier_)
            {
                overlay.setPosition(sf::Vector2f(cell->x_ * grid_.getCellSize(), cell->y_ * grid_.getCellSize()) + offset);
                window_.draw(overlay);
            }

            color.a = 64;
            overlay.setFillColor(color);
            for (Cell* cell : snapshot.explored_)
            {
                overlay.setPosition(sf::Vector2f(cell->x_ * grid_.getCellSize(), cell->y_ * grid_.getCellSize()) + offset);
                window_.draw(overlay);
            }

            break;
        }
        case AppState::kDone:
        {
            if (agent.path_.empty()) return;
            sf::Color pathColor = sf::Color(255, 165, 0); // orange
            color.a = 150;
            overlay.setFillColor(pathColor);
            for (Cell* cell : agent.path_)
            {
                overlay.setPosition(sf::Vector2f(cell->x_ * grid_.getCellSize(), cell->y_ * grid_.getCellSize()) + offset);
                window_.draw(overlay);
            }

            if (agent.snapshots_.empty()) return;
            Snapshot& snapshot = agent.snapshots_[agent.snapshots_.size() - 1];

            color.a = 128;
            overlay.setFillColor(color);
            for (Cell* cell : snapshot.frontier_)
            {
                overlay.setPosition(sf::Vector2f(cell->x_ * grid_.getCellSize(), cell->y_ * grid_.getCellSize()) + offset);
                window_.draw(overlay);
            }

            color.a = 64;
            overlay.setFillColor(color);
            for (Cell* cell : snapshot.explored_)
            {
                overlay.setPosition(sf::Vector2f(cell->x_ * grid_.getCellSize(), cell->y_ * grid_.getCellSize()) + offset);
                window_.draw(overlay);
            }
            break;
        }
    }
}


void Game::onDrag(const sf::Event::MouseMoved &mouseEvent, const sf::Vector2f &worldPos)
{
    if (app_state_ != AppState::kIdle) return;

    const int row = worldPos.x / grid_.getCellSize();
    const int col = worldPos.y / grid_.getCellSize();

    if (row >= grid_.getRows() || row < 0 || col >= grid_.getCols() || col < 0)
    {
        return;
    }

    switch (input_mode_)
    {
        case InputMode::kSelecting:
        {
            selectCell(row, col);
            break;
        }
        case InputMode::kDeselecting:
        {
            deselectCell(row, col);
            break;
        }
    }
}

void Game::onMouseClick(const sf::Event::MouseButtonPressed &mouseEvent, const sf::Vector2f &worldPos)
{
    if (app_state_ != AppState::kIdle) return;

    const int row = worldPos.x / grid_.getCellSize();
    const int col = worldPos.y / grid_.getCellSize();

    if (row >= grid_.getRows() || row < 0 || col >= grid_.getCols() || col < 0)
    {
        return;
    }

    switch (input_mode_)
    {
        case InputMode::kSelecting:
        {
            selectCell(row, col);
            break;
        }
        case InputMode::kDeselecting:
        {
            deselectCell(row, col);
            break;
        }
    }
}

void Game::runAlgorithm()
{
    if (game_mode_ == GameMode::kSinglePathfinding)
    {
        if (placement_state_ != PlacementState::kPlacingObstacles) return;
        agents_.clear();
        agents_.push_back(std::make_unique<Agent>(
            start_cell_, goal_cell_, grid_, sf::Color::Blue, record_search_snapshots_));
    }
    else
    {
        const int n = std::clamp(multi_agent_count_, 1, kMaxMultiAgents);
        if (agents_.size() != static_cast<std::size_t>(n))
        {
            initAgents();
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    switch (parallel_strategy_)
    {
        case ParallelStrategy::kSequential:
        {
            for (auto& agent : agents_)
            {
                runAlgorithmOnAgent(agent.get(), algorithm_);
            }
            break;
        }
        case ParallelStrategy::kOpenMP:
        {
            //std::cout << "OpenMP Threads: " << omp_get_max_threads() << std::endl;
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(agents_.size()); ++i)
            {
                this->runAlgorithmOnAgent(agents_[i].get(), algorithm_);
            }
            break;
        }
        case ParallelStrategy::kPerAgentThread:
        {
            std::vector<std::thread> threads;
            threads.reserve(agents_.size());
            for (auto& agent : agents_)
            {
                Agent* a = agent.get();
                const Algorithm algo = algorithm_;
                threads.emplace_back([this, a, algo]()
                {
                    this->runAlgorithmOnAgent(a, algo);
                });
            }
            for (auto& t : threads)
            {
                t.join();
            }
            break;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();


    metrics_.search_time = std::chrono::duration<double, std::milli>(end - start).count();

    if (!agents_.empty())
    {
        if (game_mode_ == GameMode::kSinglePathfinding)
        {
            const Agent& a = *agents_.front();
            metrics_.path_size = a.path_.size();
            metrics_.nodes_expanded = a.metrics_.nodes_expanded;
            metrics_.path_found = a.metrics_.path_found;
        }
        else
        {
            std::size_t total_nodes = 0;
            for (const auto& ag : agents_)
            {
                total_nodes += ag->metrics_.nodes_expanded;
            }
            metrics_.nodes_expanded = total_nodes;
            metrics_.path_found = agents_.front()->metrics_.path_found;
            metrics_.path_size = agents_.front()->path_.size();
        }
    }
}

void Game::runAlgorithmOnAgent(Agent* agent, Algorithm algorithm)
{
    agent->setHeuristicOptions(heuristic_tie_nudge_, static_cast<double>(heuristic_weight_));
    switch (algorithm)
    {
        case Algorithm::kAStar:
            agent->runAStar();
            break;
        case Algorithm::kDijkstra:
            agent->runDijkstra();
            break;
        case Algorithm::kGreedy:
            agent->runGreedy();
            break;
        case Algorithm::kBFS:
            agent->runBFS();
            break;
        case Algorithm::kDFS:
            agent->runDFS();
            break;
        case Algorithm::kBidirectionalAStar:
            agent->runBidirectionalAStar();
            break;
    }
}

void Game::initAgents()
{
    agents_.clear();
    const int n = std::clamp(multi_agent_count_, 1, kMaxMultiAgents);
    const int lastRow = grid_.getRows() - 1;
    const int lastCol = grid_.getCols() - 1;
    for (int i = 0; i < n; ++i)
    {
        agents_.push_back(std::make_unique<Agent>(
            &grid_.cells_[0][lastCol],
            &grid_.cells_[lastRow][0],
            grid_,
            sf::Color::Red,
            record_search_snapshots_));
    }
}

void Game::saveGridToFile()
{
    std::ofstream outGridFile{"Grid.txt"};

    if (outGridFile.is_open())
    {
        outGridFile << grid_.getRows() << " " << grid_.getCols() << "\n";

        for (int i = 0; i < grid_.getRows(); i++)
        {
            for (int j = 0; j < grid_.getCols(); j++)
            {
                if (grid_.cells_[i][j].getType() != CellType::obstacle)
                {
                    outGridFile << "0";
                }
                else
                {
                    outGridFile << "1";
                }
            }
            outGridFile << "\n";
        }
    }
    outGridFile.close();
}

void Game::loadGridFromFile()
{
    reset();
    std::ifstream inGridFile{"Grid.txt"};

    if (!inGridFile)
    {
        std::cerr << "Failed to open Grid.txt\n";
    }

    std::string firstLine;
    int row {};
    int col {};
    if (std::getline(inGridFile, firstLine))
    {
        std::stringstream ss(firstLine);
        ss >> row >> col;
    }

    char c;
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            if (inGridFile >> c)
            {
                if (c == '1')
                    grid_.cells_[i][j].setType(CellType::obstacle);
            }
        }
    }

    std::cout << "row: " << row << " col: " << col;
    inGridFile.close();

}

sf::Vector2f Game::getGridOffset() const
{
    float x = window_.getSize().x / 2.f - grid_.getCellSize() * grid_.getRows() / 2.f;
    float y = window_.getSize().y / 2.f - grid_.getCellSize() * grid_.getCols() / 2.f;
    return sf::Vector2f(x, y);
}

void Game::selectCell(int row, int col)
{
    if (game_mode_ != GameMode::kSinglePathfinding)
    {
        if (grid_.cells_[row][col].getType() == CellType::open)
        {
            grid_.cells_[row][col].setType(CellType::obstacle);
        }
        return;
    }

    switch (placement_state_)
    {
        case PlacementState::kNeedsStart:
        {
            if (grid_.cells_[row][col].getType() == CellType::goal)
            {
                break;
            }

            if (start_cell_)
                start_cell_->setType(CellType::open);

            grid_.cells_[row][col].setType(CellType::start);
            start_cell_ = &grid_.cells_[row][col];
            placement_state_ = PlacementState::kNeedsGoal;

            break;
        }
        case PlacementState::kNeedsGoal:
        {
            if (grid_.cells_[row][col].getType() == CellType::start)
            {
                break;
            }

            if (goal_cell_)
                goal_cell_->setType(CellType::open);

            grid_.cells_[row][col].setType(CellType::goal);
            goal_cell_ = &grid_.cells_[row][col];
            placement_state_ = PlacementState::kPlacingObstacles;
            break;
        }
        case PlacementState::kPlacingObstacles:
        {
            if (grid_.cells_[row][col].getType() != CellType::start &&
                    grid_.cells_[row][col].getType() != CellType::goal)
            {
                grid_.cells_[row][col].setType(CellType::obstacle);
            }
            break;
        }
    }
}

void Game::deselectCell(int row, int col)
{
    switch (grid_.cells_[row][col].getType())
    {
        case CellType::start:
        {
            grid_.cells_[row][col].setType(CellType::open);
            start_cell_ = nullptr;
            placement_state_ = PlacementState::kNeedsStart;
            break;
        }
        case CellType::goal:
        {
            grid_.cells_[row][col].setType(CellType::open);
            goal_cell_ = nullptr;
            placement_state_ = start_cell_ ? PlacementState::kNeedsGoal : PlacementState::kNeedsStart;
            break;
        }
        case CellType::obstacle:
        {
            grid_.cells_[row][col].setType(CellType::open);
            break;
        }
        default:
        {
            break;
        }
    }
}

void Game::reset()
{
    for (int i = 0; i < grid_.getRows(); ++i)
    {
        for (int j = 0; j < grid_.getCols(); ++j)
        {
            grid_.cells_[i][j].reset();
        }
    }

    const int agentCount = static_cast<int>(agents_.size());
    if (agentCount > 0)
    {
        #pragma omp parallel for schedule(static) if(agentCount > 256)
        for (int i = 0; i < agentCount; ++i)
        {
            if (agents_[static_cast<std::size_t>(i)])
            {
                agents_[static_cast<std::size_t>(i)]->clearSearchState();
            }
        }
    }
    agents_.clear();
    agents_.shrink_to_fit();

    metrics_ = {};

    start_cell_ = nullptr;
    goal_cell_ = nullptr;
    placement_state_ = PlacementState::kNeedsStart;
    app_state_ = AppState::kIdle;
}
