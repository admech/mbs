within MbsLite.Test.Omni.Cases;

model OmniVehicleSelfRotViscousTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmSelfRotViscous
        /*
        TestParams.pmmAtRestViscous
        TestParams.pmmSelfRotViscous
        TestParams.pmmStraightViscous
        TestParams.pmmWrenchViscous
        */
    );

end OmniVehicleSelfRotViscousTest;
