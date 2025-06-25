#include "ReconstructionManagerWidget.h"

const size_t ReconstructionManagerWidget::kNewestReconstructionIdx =
std::numeric_limits<size_t>::max();

ReconstructionManagerWidget::ReconstructionManagerWidget(
    QWidget* parent,
    std::shared_ptr<const ReconstructionManager> reconstruction_manager)
    : QComboBox(parent),
    reconstruction_manager_(std::move(reconstruction_manager)) {
    QFont font;
    font.setPointSize(10);
    setFont(font);
}

void ReconstructionManagerWidget::Update()
{
    std::lock_guard<std::mutex> lock(mu);
    __try
    {
        if (view()->isVisible()) {
            return;
        }

        blockSignals(true);

        const int prev_idx = currentIndex() == -1 ? 0 : currentIndex();

        clear();

        addItem("Newest model");

        int max_width = 0;
        for (size_t i = 0; i < reconstruction_manager_->Size(); ++i) {
            const QString item = QString().asprintf(
                "Model %d (%d images, %d points)",
                static_cast<int>(i + 1),
                static_cast<int>(reconstruction_manager_->Get(i)->NumRegImages()),
                static_cast<int>(reconstruction_manager_->Get(i)->NumPoints3D()));
            QFontMetrics font_metrics(view()->font());
#if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
            const auto width = font_metrics.horizontalAdvance(item);
#else
            const auto width = font_metrics.width(item);
#endif
            max_width = std::max(max_width, width);
            addItem(item);
        }

        view()->setMinimumWidth(max_width);

        if (reconstruction_manager_->Size() == 0) {
            setCurrentIndex(0);
        }
        else {
            const int max_idx = static_cast<int>(reconstruction_manager_->Size());
            if (prev_idx <= max_idx) {
                setCurrentIndex(prev_idx);
            }
            else {
                setCurrentIndex(max_idx);
            }
        }

        blockSignals(false);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        std::cout << "listwidget error!" << endl;
    }
    
}

size_t ReconstructionManagerWidget::SelectedReconstructionIdx() const 
{
    std::lock_guard<std::mutex> lock(mu);
    if (reconstruction_manager_->Size() == 0) 
    {
        return kNewestReconstructionIdx;
    }
    else 
    {
        if (currentIndex() == 0) 
        {
            return kNewestReconstructionIdx;
        }
        else 
        {
            return currentIndex() - 1;
        }
    }
}

void ReconstructionManagerWidget::SelectReconstruction(const size_t idx) 
{
    std::lock_guard<std::mutex> lock(mu);
    __try
    {
        if (reconstruction_manager_->Size() == 0) 
        {
            blockSignals(true);
            setCurrentIndex(0);
            blockSignals(false);
        }
        else 
        {
            blockSignals(true);
            setCurrentIndex(idx + 1);
            blockSignals(false);
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        std::cout << "listwidget error!" << endl;
    }
    
}