#pragma once

#include "RegressionBase.h"

namespace hb {

    /**
   * The values entered in the constructor shoudld be the same ones coming from this desmos:
   * https://www.desmos.com/calculator/jjwe4f3fl0
   * 
   * The numbers should be entered into the contructor in the order found in desmos but for reference they will be this:
   * ax + b
   * 
   */
  class Linear : public RegressionBase {
    
    public:

      explicit Linear(double a, double b);

      double Calculate(double x) const override;

    private:

      double m_a, m_b;
  };
}

