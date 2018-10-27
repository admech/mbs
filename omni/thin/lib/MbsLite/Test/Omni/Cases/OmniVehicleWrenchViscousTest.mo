within MbsLite.Test.Omni.Cases;

model OmniVehicleWrenchViscousTest

  OmniVehicleTest t
    (final m.ovp =
        TestParams.pmmWrenchViscous
        /*
        TestParams.pmmAtRestViscous
        TestParams.pmmSelfRotViscous
        TestParams.pmmStraightViscous
        TestParams.pmmWrenchViscous
        */
    );

end OmniVehicleWrenchViscousTest;
