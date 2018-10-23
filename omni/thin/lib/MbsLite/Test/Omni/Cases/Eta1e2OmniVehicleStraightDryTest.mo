within MbsLite.Test.Omni.Cases;

model Eta1e2OmniVehicleStraightDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmStraightDryEta1e2
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmStraightDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e2OmniVehicleStraightDryTest;
