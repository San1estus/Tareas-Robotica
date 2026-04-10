#ifndef SHAPES_H
#define SHAPES_H

#include <cmath>
#include <vector>

const float PI = acos(-1);

struct ShapeData{

    std::vector<float> vertices;
    std::vector<unsigned int> indices;
};

ShapeData createRectangle(float width, float height){

    float hw = width*0.5f;
    float hh = height*0.5f;

    ShapeData data;

    data.vertices = {
        -hw,-hh,
         hw,-hh,
        -hw, hh,
         hw, hh
    };

    data.indices = {
        0,1,2,
        2,3,1
    };

    return data;
}

ShapeData createEquilateralTriangle(float size){

    float h = size * sqrt(3.0f)/2.0f;

    ShapeData data;

    data.vertices = {
        -size*0.5f, -h/3.0f,
         size*0.5f, -h/3.0f,
         0.0f,       2.0f*h/3.0f
    };

    data.indices = {0,1,2};

    return data;
}

ShapeData createCircle(float radius,int segments){

    ShapeData data;

    data.vertices.push_back(0);
    data.vertices.push_back(0);

    for(int i=0;i<=segments;i++){

        float theta = 2*PI*i/segments;

        float x = radius*cos(theta);
        float y = radius*sin(theta);

        data.vertices.push_back(x);
        data.vertices.push_back(y);

        if(i>0){
            data.indices.push_back(0);
            data.indices.push_back(i);
            data.indices.push_back(i+1);
        }
    }

    return data;
}

#endif