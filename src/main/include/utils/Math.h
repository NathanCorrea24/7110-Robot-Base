#pragma once

namespace hb {

    double sgn(double x) {
        return x < 0 ? -1.0 : 1.0;
    }

}