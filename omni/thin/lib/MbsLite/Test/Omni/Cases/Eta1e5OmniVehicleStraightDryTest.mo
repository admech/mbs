within MbsLite.Test.Omni.Cases;

model Eta1e5OmniVehicleStraightDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmStraightDryEta1e5
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmStraightDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e5OmniVehicleStraightDryTest;
