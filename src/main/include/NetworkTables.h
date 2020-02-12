#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

class VisionTable {
    public:
    VisionTable();
    ~VisionTable();

    private:
    NetworkTable* m_pVisionTable;
};

