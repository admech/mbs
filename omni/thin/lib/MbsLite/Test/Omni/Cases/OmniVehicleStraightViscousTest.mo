within MbsLite.Test.Omni.Cases;

model OmniVehicleStraightViscousTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmStraightViscous
        /*
        TestParams.pmmAtRestViscous
        TestParams.pmmSelfRotViscous
        TestParams.pmmStraightViscous
        TestParams.pmmWrenchViscous
        */
    );

end OmniVehicleStraightViscousTest;
