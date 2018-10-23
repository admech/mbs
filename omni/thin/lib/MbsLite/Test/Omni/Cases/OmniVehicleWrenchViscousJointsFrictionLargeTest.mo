within MbsLite.Test.Omni.Cases;

model OmniVehicleWrenchViscousJointsFrictionLargeTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmWrenchViscousJointsFrictionLarge
        /*
        TestParams.pmmAtRestViscous
        TestParams.pmmSelfRotViscous
        TestParams.pmmStraightViscous
        TestParams.pmmWrenchViscous
        */
    );

end OmniVehicleWrenchViscousJointsFrictionLargeTest;
