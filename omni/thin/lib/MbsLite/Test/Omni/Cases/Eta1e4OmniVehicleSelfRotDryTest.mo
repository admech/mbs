within MbsLite.Test.Omni.Cases;

model Eta1e4OmniVehicleSelfRotDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmSelfRotDryEta1e4
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmSelfRotDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e4OmniVehicleSelfRotDryTest;
