within MbsLite.Test.Omni.Cases;

model Eta1e2OmniVehicleSelfRotDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmSelfRotDryEta1e2
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmSelfRotDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e2OmniVehicleSelfRotDryTest;
