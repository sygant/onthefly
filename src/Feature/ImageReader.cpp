#include "ImageReader.h"
#include "../Base/string.h"

ImageReader::ImageReader() :defaultFocalLengthFactor(1.2), maxImageSize(3200)
{

}
ImageReader::Status ImageReader::Read(const std::string& imagePath, Camera& camera, std::string& cameraModel, Image& image, Bitmap& bitmap)
{
	const std::string imagePath_ = StringReplace(imagePath, "\\", "/");
	const size_t pos = imagePath_.find_last_of('/');
	const std::string imageName = imagePath_.substr(pos + 1);
	image.SetName(imageName);

	if (!bitmap.Read(imagePath, false))
	{
		return Status::BITMAP_ERROR;
	}
	bitmap.ExifCameraModel(&cameraModel);
	double focalLength = 0.0;
	auto useDefaultFocal = StringArgParse(R"(D:\ECCV\Software\settings.ini)", "enableFocalLengthFactor");
	if (useDefaultFocal == "true")
	{
		if (bitmap.ExifFocalLength(&focalLength) && abs(focalLength) > 1e-5)
		{
		camera.SetPriorFocalLength(true);
		}
		else
		{
			focalLength = defaultFocalLengthFactor * std::max(bitmap.Width(), bitmap.Height());
			camera.SetPriorFocalLength(false);
		}
	}
	else
	{
		auto FocalLengthFactor = StringArgParse(R"(D:\ECCV\Software\settings.ini)", "defaultFocalLengthFactor");
		focalLength = 0.7*std::max(bitmap.Width(), bitmap.Height());
	}
	camera.InitializeWithId(2, focalLength, bitmap.Width(), bitmap.Height());
	if (!camera.VerifyParams())
	{
		return Status::CAMERA_PARAM_ERROR;
	}

	Eigen::Vector3d& translationPrior = image.CamFromWorldPrior().translation;
	if (!bitmap.ExifLatitude(&translationPrior.x()) || !bitmap.ExifLongitude(&translationPrior.y()) || !bitmap.ExifAltitude(&translationPrior.z()))
	{
		translationPrior.setConstant(std::numeric_limits<double>::quiet_NaN());
	}

	if (bitmap.Width() > maxImageSize || bitmap.Height() > maxImageSize)
	{
		const double scale = static_cast<double>(maxImageSize) / std::max(bitmap.Width(), bitmap.Height());
		const int newWidth = static_cast<int>(bitmap.Width() * scale);
		const int newHeight = static_cast<int>(bitmap.Height() * scale);
		bitmap.Rescale(newWidth, newHeight);
	}

	return Status::SUCCESS;
}



