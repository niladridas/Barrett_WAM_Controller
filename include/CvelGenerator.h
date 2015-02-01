/*
 * CvelGenerator.h
 *
 *  Created on: 14-Dec-2014
 *      Author: mobman
 */

#ifndef CVELGENERATOR_H_
#define CVELGENERATOR_H_
#include <eigen3/Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/wam.h>
using namespace barrett;
using namespace systems;

namespace Sam {



    template <size_t DOF>
    class CvelGenerator: public System,
                         public SingleOutput <units::CartesianVelocity::type> {

      BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    public:
      CvelGenerator(const Wam<DOF>* const w, const std::string& sysName="CvelGenerator"):
      System(sysName),SingleOutput<units::CartesianVelocity::type>(this),
      cVel(0.0), jv(math::Matrix<DOF,1>()), J(math::Matrix<6,DOF>()), vw(math::Matrix<6,1>())
      {
        wam = w;
      }
      virtual  ~CvelGenerator(){this->mandatoryCleanUp();}

    private:
      cv_type cVel;
      math::Matrix<DOF,1> jv;
      math::Matrix<6,DOF> J;
      math::Matrix<6,1> vw;
      const systems::Wam<DOF>* wam;

    protected:
      virtual void operate(){
        jv=wam->getJointVelocities();
        J=wam->getToolJacobian();
        vw = J*jv;
        for (size_t i=0; i<3; ++i){
          cVel[i]=vw(i,0);
        }
        this->outputValue->setData(&cVel);

      }

    };

   /* namespace systems */
} /* namespace barrett */
#endif /* CVELGENERATOR_H_ */
