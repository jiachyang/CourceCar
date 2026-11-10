// 本文件由 scripts/gen_maze_fixture.py 生成，请勿手工编辑。
// 源 JSON：test/data/maze_model.json
// 源文件 SHA256：5bed05050cc36a2813fd2360d4b2c390e2eb847d7f7b316e62c63cf0fe10b601
// 用途：主机端车道保持/地图射线模拟测试；不参与固件实时控制。
#pragma once

#include <cstdint>

namespace maze_fixture {

struct VerticalSegment {
    std::int32_t x = 0;
    std::int32_t yStart = 0;
    std::int32_t yEnd = 0;
};

struct HorizontalSegment {
    std::int32_t y = 0;
    std::int32_t xStart = 0;
    std::int32_t xEnd = 0;
};

struct MovableSegment {
    const char* name = "";
    char orientation = 'V';
    std::int32_t position = 0;
    std::int32_t start = 0;
    std::int32_t end = 0;
};

struct Site {
    const char* id = "";
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    double xM = 0.0;
    double yM = 0.0;
};

inline constexpr std::int32_t kCols = 8;
inline constexpr std::int32_t kRows = 8;
inline constexpr double kCellSizeM = 0.45;
inline constexpr double kFieldSizeM = kCols * kCellSizeM;
inline constexpr const char* kSourceSha256 = "5bed05050cc36a2813fd2360d4b2c390e2eb847d7f7b316e62c63cf0fe10b601";

inline constexpr VerticalSegment kFixedVertical[] = {
    VerticalSegment{3, 0, 1},
    VerticalSegment{1, 1, 2},
    VerticalSegment{2, 2, 3},
    VerticalSegment{4, 0, 4},
    VerticalSegment{5, 1, 2},
    VerticalSegment{6, 0, 1},
    VerticalSegment{6, 2, 4},
    VerticalSegment{7, 1, 5},
    VerticalSegment{3, 3, 5},
    VerticalSegment{2, 5, 7},
    VerticalSegment{5, 5, 6},
    VerticalSegment{4, 6, 7},
    VerticalSegment{1, 6, 8},
    VerticalSegment{3, 7, 8},
    VerticalSegment{5, 7, 8},
    VerticalSegment{6, 5, 7},
};
inline constexpr int kFixedVerticalCount =
    static_cast<int>(sizeof(kFixedVertical) / sizeof(kFixedVertical[0]));

inline constexpr HorizontalSegment kFixedHorizontal[] = {
    HorizontalSegment{1, 1, 3},
    HorizontalSegment{2, 2, 4},
    HorizontalSegment{2, 5, 7},
    HorizontalSegment{3, 0, 1},
    HorizontalSegment{3, 4, 5},
    HorizontalSegment{4, 1, 3},
    HorizontalSegment{4, 5, 6},
    HorizontalSegment{5, 3, 5},
    HorizontalSegment{6, 2, 5},
    HorizontalSegment{6, 7, 8},
    HorizontalSegment{7, 6, 7},
};
inline constexpr int kFixedHorizontalCount =
    static_cast<int>(sizeof(kFixedHorizontal) / sizeof(kFixedHorizontal[0]));

inline constexpr MovableSegment kMovable[] = {
    MovableSegment{"M1", 'V', 1, 3, 4},
    MovableSegment{"M2", 'V', 5, 4, 5},
    MovableSegment{"M3", 'H', 5, 6, 7},
};
inline constexpr int kMovableCount =
    static_cast<int>(sizeof(kMovable) / sizeof(kMovable[0]));

inline constexpr Site kSites[] = {
    Site{"S", 4, 7, 2.025, 3.375},
    Site{"F", 3, 0, 1.575, 0.225},
    Site{"A", 0, 3, 0.225, 1.575},
    Site{"B", 5, 4, 2.475, 2.025},
    Site{"C", 3, 2, 1.575, 1.125},
};
inline constexpr int kSiteCount =
    static_cast<int>(sizeof(kSites) / sizeof(kSites[0]));

}  // namespace maze_fixture
