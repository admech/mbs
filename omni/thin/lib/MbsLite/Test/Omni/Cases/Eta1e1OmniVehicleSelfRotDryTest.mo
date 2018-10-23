within MbsLite.Test.Omni.Cases;

model Eta1e1OmniVehicleSelfRotDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmSelfRotDryEta1e1
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmSelfRotDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e1OmniVehicleSelfRotDryTest;
