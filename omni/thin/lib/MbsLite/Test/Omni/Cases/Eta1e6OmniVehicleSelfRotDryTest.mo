within MbsLite.Test.Omni.Cases;

model Eta1e6OmniVehicleSelfRotDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmSelfRotDryEta1e6
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmSelfRotDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e6OmniVehicleSelfRotDryTest;
