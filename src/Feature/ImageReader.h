#pragma once
#include "Bitmap.h"
#include "../Scene/Camera.h"
#include "../Scene/Image.h"

class ImageReader
{
public:
    enum class Status 
    {
        FAILURE,
        SUCCESS,
        IMAGE_EXISTS,
        BITMAP_ERROR,
        CAMERA_SINGLE_DIM_ERROR,
        CAMERA_EXIST_DIM_ERROR,
        CAMERA_PARAM_ERROR
    };

    ImageReader();

    Status Read(const std::string& imagePath, Camera& camera, std::string& cameraModel, Image& image, Bitmap& bitmap);
private:
    const double defaultFocalLengthFactor;
    const size_t maxImageSize;
};













