within MbsLite.Test.Omni.Cases;

model Eta1e3OmniVehicleSelfRotDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmSelfRotDryEta1e3
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmSelfRotDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e3OmniVehicleSelfRotDryTest;
