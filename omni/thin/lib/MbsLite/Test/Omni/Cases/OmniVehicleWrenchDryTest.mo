within MbsLite.Test.Omni.Cases;

model OmniVehicleWrenchDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmWrenchDry
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmSelfRotDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end OmniVehicleWrenchDryTest;
