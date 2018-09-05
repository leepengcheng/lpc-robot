#pragma once

#ifndef IMAGE_H_
#define IMAGE_H_

#include "rpc/msgpack.hpp"
#include <vector>

//struct pixel {
//    unsigned char r, g, b;
//    MSGPACK_DEFINE_ARRAY(r, g, b)
//};
//using pixel_data = std::vector<pixel>;

struct ImageMsg
{
    int width;
    int height;
    std::vector<unsigned char> data;
    MSGPACK_DEFINE_MAP(width, height, data)
};
struct HalconImageMsg
{
    int width;
    int height;
    std::vector<unsigned char> r;
    std::vector<unsigned char> g;
    std::vector<unsigned char> b;
    MSGPACK_DEFINE_MAP(width, height, r,g,b)
};


#endif /* end of include guard: IMAGE_H_ */

