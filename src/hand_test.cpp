#include <iostream>
#include <string>
#include <cstdlib>
#include <barrett/os.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>





using namespace barrett;

void assertPosition(Hand& hand, Hand::jp_type innerLinkJp, double tolerance = 0.05) {
    hand.update();
    if (math::abs(hand.getInnerLinkPosition() - innerLinkJp).maxCoeff() > tolerance) {
        std::cout << hand.getInnerLinkPosition() << " is not close enough to " << innerLinkJp << "\n"<<"moving to home"<<std::endl;
        exit(2);
    }
}

#include <barrett/standard_main_function.h>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef Hand::jp_type hjp_t;
    const char* path = pm.getWamDefaultConfigPath();
    std::cout<<"got configuration "<<std::endl;
    size_t i=0;
    while(path[i]!='\0'){
      std::cout<<path[i];
      i++;
    }
    jp_type zero(0.0);

    double O = 0.0;
    double C = 2.4;
    double SC = M_PI;
    hjp_t open(O);
    hjp_t closed(C);
    closed[3] = SC;

    jp_type PickPos, placePos;
    PickPos<<-0.002,  1.200,  0.002,  1.536, -0.054, -1.262, -1.517;
    placePos<<0.545,  1.199,  0.532,  1.481, -0.478, -0.917, -1.879;
    double OR = -0.75;
    double CR = 0.75;
    Hand::jv_type opening(OR);
    Hand::jv_type closing(CR);
    wam.gravityCompensate();
    wam.moveTo(zero);

    if ( !pm.foundHand() ) {
        printf("ERROR: No Hand found on bus!\n");
        return 1;
    }
    Hand& hand = *pm.getHand();
    std::cout<<"[Enter] to initialize Hand"<<std::endl;
    detail::waitForEnter();
    hand.initialize();


    std::cout<<"operate on Hand"<<std::endl;
    detail::waitForEnter();
    {
            assertPosition(hand, open);
            std::cout<<"[Enter] to close Hand"<<std::endl;
            detail::waitForEnter();
            hand.close();
            assertPosition(hand, closed);
            std::cout<<"[Enter] to open Hand"<<std::endl;
            detail::waitForEnter();
            hand.open();
            assertPosition(hand, open);
            std::cout<<"[Enter] to close spread Hand"<<std::endl;
            detail::waitForEnter();
            hand.close(Hand::SPREAD);
            assertPosition(hand, hjp_t(O,O,O,SC));
            std::cout<<"[Enter] to close grasp Hand"<<std::endl;
            detail::waitForEnter();
            hand.close(Hand::GRASP);
            assertPosition(hand, closed);
            std::cout<<"[Enter] to open spread Hand"<<std::endl;
            detail::waitForEnter();
            hand.open(Hand::GRASP, false);
            btsleep(0.5);
            assertPosition(hand, hjp_t(1.6,1.6,1.6,SC));
            std::cout<<"[Enter] to open Hand"<<std::endl;
            detail::waitForEnter();
            hand.open();
            assertPosition(hand, open);
            wam.moveTo(PickPos );
            btsleep(0.5);
            assertPosition(hand, open);
            std::cout<<"[Enter] to close Hand"<<std::endl;
            detail::waitForEnter();
            hand.close();
        }
    std::cout<<"[Enter] to move home"<<std::endl;
    detail::waitForEnter();

    wam.moveHome();


    std::cout<<"configuration should be printed"<<std::endl;
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    return 0;
}
