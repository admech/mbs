within MbsLite.Test.Omni.Cases;

model OmniVehicleWrenchViscousJointsFrictionSmallTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmWrenchViscousJointsFrictionSmall
        /*
        TestParams.pmmAtRestViscous
        TestParams.pmmSelfRotViscous
        TestParams.pmmStraightViscous
        TestParams.pmmWrenchViscous
        */
    );

end OmniVehicleWrenchViscousJointsFrictionSmallTest;
