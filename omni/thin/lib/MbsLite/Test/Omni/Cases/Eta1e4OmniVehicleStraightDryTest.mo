within MbsLite.Test.Omni.Cases;

model Eta1e4OmniVehicleStraightDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmStraightDryEta1e4
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmStraightDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e4OmniVehicleStraightDryTest;
