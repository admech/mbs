within MbsLite.Test.Omni.Cases;

model Eta1e3OmniVehicleStraightDryTest

  OmniVehicleTest t
    ( m.ovp =
        TestParams.pmmStraightDryEta1e3
        /*
        TestParams.pmmAtRestDry
        TestParams.pmmStraightDry
        TestParams.pmmStraightDry
        TestParams.pmmWrenchDry
        */
    );

end Eta1e3OmniVehicleStraightDryTest;
