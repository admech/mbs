within MbsLite.Test.Omni.Cases;

model Eta1e6OmniVehicleStraightDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmStraightDryEta1e6
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmStraightDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e6OmniVehicleStraightDryTest;
