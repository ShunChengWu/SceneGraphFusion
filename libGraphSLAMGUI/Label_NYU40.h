#pragma once
#include <map>
#include <ORUtils/Vector.h>


static std::map<unsigned short, std::string> LabelsNYU40 {
        {0, "None"},
        {1, "wall"},
        {2, "floor"},
        {3, "cabinet"},
        {4, "bed"},
        {5, "chair"},
        {6, "sofa"},
        {7, "table"},
        {8, "door"},
        {9, "window"},
        {10, "bookshelf"},
        {11, "picture"},
        {12, "counter"},
        {13, "blinds"},
        {14, "desk"},
        {15, "shelves"},
        {16, "curtain"},
        {17, "dresser"},
        {18, "pillow"},
        {19, "mirror"},
        {20, "floor mat"},
        {21, "clothes"},
        {22, "ceiling"},
        {23, "books"},
        {24, "refridgerator"},
        {25, "television"},
        {26, "paper"},
        {27, "towel"},
        {28, "shower curtain"},
        {29, "box"},
        {30, "whiteboard"},
        {31, "person"},
        {32, "night stand"},
        {33, "toilet"},
        {34, "sink"},
        {35, "lamp"},
        {36, "bathtub"},
        {37, "bag"},
        {38, "otherstructure"},
        {39, "otherfurniture"},
        {40, "otherprop"},
};

static std::map<std::string, size_t> NYU40Name2Labels {
        {"none", 0},
        {"wall", 1},
        {"floor", 2},
        {"cabinet", 3},
        {"bed", 4},
        {"chair", 5},
        {"sofa", 6},
        {"table", 7},
        {"door", 8},
        {"window", 9},
        {"bookshelf", 10},
        {"picture", 11},
        {"counter", 12},
        {"blinds", 13},
        {"desk", 14},
        {"shelves", 15},
        {"curtain", 16},
        {"dresser", 17},
        {"pillow", 18},
        {"mirror", 19},
        {"floor mat", 20},
        {"clothes", 21},
        {"ceiling", 22},
        {"books", 23},
        {"refridgerator", 24},
        {"television", 25},
        {"paper", 26},
        {"towel", 27},
        {"shower curtain", 28},
        {"box", 29},
        {"whiteboard", 30},
        {"person", 31},
        {"night stand", 32},
        {"toilet", 33},
        {"sink", 34},
        {"lamp", 35},
        {"bathtub", 36},
        {"bag", 37},
        {"otherstructure", 38},
        {"otherfurniture", 39},
        {"otherprop", 40},
};

static std::map<unsigned short, ORUtils::Vector4<int>> NYU40LabelColors{
{0, ORUtils::Vector4<int>(0,0,0,255)}, //
{1, ORUtils::Vector4<int>(174, 199, 232,255)}, // wall
{2, ORUtils::Vector4<int>(152, 223, 138,255)}, // floor
{3, ORUtils::Vector4<int>(31, 119, 180,255)}, // cabinet
{4, ORUtils::Vector4<int>(255, 187, 120,255)}, // bed
{5, ORUtils::Vector4<int>(188, 189, 34,255)},  // chair
{6, ORUtils::Vector4<int>(140, 86, 75,255)}, // sofa
{7, ORUtils::Vector4<int>(255, 152, 150,255)}, // table
{8, ORUtils::Vector4<int>(214, 39, 40,255)}, // door
{9, ORUtils::Vector4<int>(197, 176, 213,255)}, // window
{10, ORUtils::Vector4<int>(148, 103, 189,255)}, // bookshelf
{11, ORUtils::Vector4<int>(196, 156, 148,255)}, // picture
{12, ORUtils::Vector4<int>(23, 190, 207,255)}, // counter
{13, ORUtils::Vector4<int>(178, 76, 76,255)}, // blinds
{14, ORUtils::Vector4<int>(247, 182, 210,255)}, // desk
{15, ORUtils::Vector4<int>(66, 188, 102,255)}, // shelves
{16, ORUtils::Vector4<int>(219, 219, 141,255)}, // curtain
{17, ORUtils::Vector4<int>(140, 57, 197,255)}, // dresser
{18, ORUtils::Vector4<int>(202, 185, 52,255)}, // pillow
{19, ORUtils::Vector4<int>(51, 176, 203,255)}, // mirror
{20, ORUtils::Vector4<int>(200, 54, 131,255)}, // floor mat
{21, ORUtils::Vector4<int>(92, 193, 61,255)}, // clothes
{22, ORUtils::Vector4<int>(78, 71, 183,255)}, // ceiling
{23, ORUtils::Vector4<int>(172, 114, 82,255)}, // books
{24, ORUtils::Vector4<int>(255, 127, 14,255)}, // refrigerator
{25, ORUtils::Vector4<int>(91, 163, 138,255)}, // television
{26, ORUtils::Vector4<int>(153, 98, 156,255)}, // paper
{27, ORUtils::Vector4<int>(140, 153, 101,255)}, // towel
{28, ORUtils::Vector4<int>(158, 218, 229,255)}, // shower curtain
{29, ORUtils::Vector4<int>(100, 125, 154,255)}, // box
{30, ORUtils::Vector4<int>(178, 127, 135,255)}, // whiteboard
{31, ORUtils::Vector4<int>(120, 185, 128,255)}, // person
{32, ORUtils::Vector4<int>(146, 111, 194,255)}, // night stand
{33, ORUtils::Vector4<int>(44, 160, 44,255)}, // toilet
{34, ORUtils::Vector4<int>(112, 128, 144,255)}, // sink
{35, ORUtils::Vector4<int>(96, 207, 209,255)}, // lamp
{36, ORUtils::Vector4<int>(227, 119, 194,255)}, // bathtub
{37, ORUtils::Vector4<int>(213, 92, 176,255)}, // bag
{38, ORUtils::Vector4<int>(94, 106, 211,255)}, // otherstructure
{39, ORUtils::Vector4<int>(82, 84, 163,255)}, // otherfurn
{40, ORUtils::Vector4<int>(100, 85, 144,255)}, // otherprop
};


static std::map<ORUtils::Vector3<int>, unsigned short> NYU40ColorToLabels{
        {ORUtils::Vector3<int>(0,0,0), 0}, //
        {ORUtils::Vector3<int>(174, 199, 232), 1}, // wall
        {ORUtils::Vector3<int>(152, 223, 138), 2}, // floor
        {ORUtils::Vector3<int>(31, 119, 180), 3}, // cabinet
        {ORUtils::Vector3<int>(255, 187, 120), 4}, // bed
        {ORUtils::Vector3<int>(188, 189, 34), 5},  // chair
        {ORUtils::Vector3<int>(140, 86, 75), 6}, // sofa
        {ORUtils::Vector3<int>(255, 152, 150), 7}, // table
        {ORUtils::Vector3<int>(214, 39, 40), 8}, // door
        {ORUtils::Vector3<int>(197, 176, 213), 9}, // window
        {ORUtils::Vector3<int>(148, 103, 189), 10}, // bookshelf
        {ORUtils::Vector3<int>(196, 156, 148), 11}, // picture
        {ORUtils::Vector3<int>(23, 190, 207), 12}, // counter
        {ORUtils::Vector3<int>(178, 76, 76), 13}, // blinds
        {ORUtils::Vector3<int>(247, 182, 210), 14}, // desk
        {ORUtils::Vector3<int>(66, 188, 102), 15}, // shelves
        {ORUtils::Vector3<int>(219, 219, 141), 16}, // curtain
        {ORUtils::Vector3<int>(140, 57, 197), 17}, // dresser
        {ORUtils::Vector3<int>(202, 185, 52), 18}, // pillow
        {ORUtils::Vector3<int>(51, 176, 203), 19}, // mirror
        {ORUtils::Vector3<int>(200, 54, 131), 20}, // floor mat
        {ORUtils::Vector3<int>(92, 193, 61), 21}, // clothes
        {ORUtils::Vector3<int>(78, 71, 183), 22}, // ceiling
        {ORUtils::Vector3<int>(172, 114, 82), 23}, // books
        {ORUtils::Vector3<int>(255, 127, 14), 24}, // refrigerator
        {ORUtils::Vector3<int>(91, 163, 138), 25}, // television
        {ORUtils::Vector3<int>(153, 98, 156), 26}, // paper
        {ORUtils::Vector3<int>(140, 153, 101), 27}, // towel
        {ORUtils::Vector3<int>(158, 218, 229), 28}, // shower curtain
        {ORUtils::Vector3<int>(100, 125, 154), 29}, // box
        {ORUtils::Vector3<int>(178, 127, 135), 30}, // whiteboard
        {ORUtils::Vector3<int>(120, 185, 128), 31}, // person
        {ORUtils::Vector3<int>(146, 111, 194), 32}, // night stand
        {ORUtils::Vector3<int>(44, 160, 44), 33}, // toilet
        {ORUtils::Vector3<int>(112, 128, 144), 34}, // sink
        {ORUtils::Vector3<int>(96, 207, 209), 35}, // lamp
        {ORUtils::Vector3<int>(227, 119, 194), 36}, // bathtub
        {ORUtils::Vector3<int>(213, 92, 176), 37}, // bag
        {ORUtils::Vector3<int>(94, 106, 211), 38}, // otherstructure
        {ORUtils::Vector3<int>(82, 84, 163), 39}, // otherfurn
        {ORUtils::Vector3<int>(100, 85, 144), 40}, // otherprop
};

