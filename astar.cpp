#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <array>
#include <cmath>
#include <cfloat>
#include <optional>

#define OBSTACLE 'o'

using Coords = std::pair<int32_t, int32_t>;

class Tile
{
private:
    Coords m_parent;
    bool m_closed;
    double m_gval;
    double m_hval;
    char m_data;
public:
    Tile(char data)
    {
        m_parent = {-1, -1};
        m_closed = false;
        m_gval = DBL_MAX;
        m_hval = DBL_MAX; 
        m_data = data;
    }
    Coords parent() const
    {
        return m_parent;
    }
    bool closed() const
    {
        return m_closed;
    }
    double gval() const
    {
        return m_gval;
    }
    double hval() const
    {
        return m_hval;
    }
    double fval() const
    {
        return m_gval + m_hval;
    }
    char data() const
    {
        return m_data;
    }
    void parent(Coords parent)
    {
        m_parent = parent;
    }
    void closed(bool closed)
    {
        m_closed = closed;
    }
    void gval(double gval)
    {
        m_gval = gval;
    }
    void hval(double hval)
    {
        m_hval = hval;
    }
    void data(char data)
    {
        m_data = data; 
    }
};

using TileLine = std::vector<Tile>;
using TileLines = std::vector<std::vector<Tile>>;

Coords findStart(const TileLines& tiles)
{
    for(size_t i = 0; i != tiles.size(); ++i)
    {
        for(size_t j = 0; j != tiles[i].size(); ++j)
        {
            if(tiles[i][j].data() == 'S')
                return Coords{i, j};
        }
    }
    throw std::runtime_error{"Unable to find start"};
}

Coords findFinish(const TileLines& tiles)
{
    for(size_t i = 0; i != tiles.size(); ++i)
    {
        for(size_t j = 0; j != tiles[i].size(); ++j)
        {
            if(tiles[i][j].data() == 'F')
                return Coords{i, j};
        }
    }
    throw std::runtime_error{"Unable to find finish"};
}

void writePath(TileLines& tiles, const Coords& finish)
{
    Coords co{finish.first, finish.second};
    for(; co.first != -1 && co.second != -1; co = {tiles[co.first][co.second].parent().first, tiles[co.first][co.second].parent().second})
    {
        tiles[co.first][co.second].data('*');
    }
}

bool isValidTile(const TileLines& tiles, const Coords& co)
{
    auto& tile = tiles[co.first][co.second];
    return co.first >= 0 && co.first < tiles.size() && co.second >= 0 && co.second < tiles[0].size() - 1 && tile.data() != 'o' && tile.closed() == false;
}

double calcHval(const TileLines& tiles, const Coords& co, const Coords& finish)
{
    return std::sqrt((co.first - finish.first) * (co.first - finish.first) + (co.second - finish.second) * (co.second - finish.second));
}

void printTiles(const TileLines& tiles)
{
    for(int i = 0; i < tiles.size(); ++i)
    {
        for(int j = 0; j < tiles[i].size(); ++j)
            std::cout << tiles[i][j].data();
        std::cout << std::endl;
    }
}

std::optional<Coords> astar(TileLines& tiles, std::set<std::pair<double, Coords>>& openList, const Coords& oriCo, const Coords& finish)
{
    static std::vector<std::pair<double, Coords>> dirs = {
        std::make_pair(1.4, Coords{-1, -1}),
        std::make_pair(1.0, Coords{-1, 0}),
        std::make_pair(1.4, Coords{-1, 1}),
        std::make_pair(1.0, Coords{0, -1}),
        std::make_pair(1.0, Coords{0, 1}),
        std::make_pair(1.4, Coords{1, -1}),
        std::make_pair(1.0, Coords{1, 0}),
        std::make_pair(1.4, Coords{1, 1})
    };

    auto oriTile = tiles[oriCo.first][oriCo.second];

    for(auto it = dirs.cbegin(); it != dirs.cend(); ++it)
    {
        auto dir = *it;
        if(!isValidTile(tiles, std::make_pair(oriCo.first + dir.second.first, oriCo.second + dir.second.second)))
            continue;
        auto newCo = std::make_pair(oriCo.first + dir.second.first, oriCo.second + dir.second.second);
        auto& newTile = tiles[newCo.first][newCo.second];

        auto newHval = calcHval(tiles, newCo, finish);
        auto newGval = oriTile.gval() + dir.first;

        if(newHval + newGval < newTile.fval() || newTile.gval() == DBL_MAX || newTile.hval() == DBL_MAX)
        {
            newTile.hval(newHval);
            newTile.gval(newGval);
            newTile.parent(oriCo);
            if(newCo.first == finish.first && newCo.second == finish.second)
                return {newCo};
            openList.insert({newTile.hval() + newTile.gval(), newCo});
        }
    }
    return std::nullopt;
}

TileLines createTiles(const std::string& path)
{
    std::ifstream mapFile{path};
    TileLines tiles;
    for(std::string line; std::getline(mapFile, line); )
    {
        TileLine tileLine;
        for(auto data = line.cbegin(); data != line.cend(); ++data)
            tileLine.push_back(Tile{*data});
        tiles.push_back(std::move(tileLine));
        if(tiles.size() >= 2 && tiles[tiles.size() - 1].size() != tiles[tiles.size() - 2].size())
            throw std::runtime_error{"Invalid map"};
    }
    return tiles;
}


int main(int argc, char* argv[])
{
    auto tiles = createTiles("map.txt");

    std::set<std::pair<double, Coords>> openList;

    auto start = findStart(tiles);
    tiles[start.first][start.second].hval(0);
    tiles[start.first][start.second].gval(0);
    openList.insert({0, start});

    auto finish = findFinish(tiles);

    std::optional<Coords> lastTile;

    while(!openList.empty())
    {
        auto it = openList.begin();
        auto prio = (*it).first;
        auto co = (*it).second;
        openList.erase(it);
        tiles[co.first][co.second].closed(true);
        lastTile = astar(tiles, openList, co, finish);
        if(lastTile.has_value())
            break;
    }
    if(lastTile.has_value())
    {
        writePath(tiles, lastTile.value());
        printTiles(tiles);
    }
    else
    {
        std::cout << "Not found!" << std::endl;
    }
}
