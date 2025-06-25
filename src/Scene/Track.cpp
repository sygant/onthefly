#include "Track.h"


Track::Track() {}

TrackElement::TrackElement()
    : image_id(kInvalidImageId), point2D_idx(kInvalidPoint2DIdx) {}

TrackElement::TrackElement(const image_t image_id, const point2D_t point2D_idx)
    : image_id(image_id), point2D_idx(point2D_idx) {}

void Track::DeleteElement(const image_t image_id, const point2D_t point2D_idx) {
    elements_.erase(
        std::remove_if(elements_.begin(),
            elements_.end(),
            [image_id, point2D_idx](const TrackElement& element) {
                return element.image_id == image_id &&
                    element.point2D_idx == point2D_idx;
            }),
        elements_.end());
}