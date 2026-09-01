#ifndef ASYNCDC_GCS_UI_H
#define ASYNCDC_GCS_UI_H
#include <memory>
#include "gcs_comm.h"

namespace ui {
    void DrawMainWindow(std::unique_ptr<gcs_comm::Connection>& connection);
}
#endif //ASYNCDC_GCS_UI_H
